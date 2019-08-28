#include <string>

#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/pcl_visualizer.h>

#include "tviewer/tviewer.h"
#include "hungarian/hungarian.h"

using namespace tviewer;

class SegmentationEvaluation
{

public:

  SegmentationEvaluation(const std::string& segmentation, const std::string& groundtruth)
  : filename_segmentation_ (segmentation)
  , filename_groundtruth_ (groundtruth)
  {
  }

  int run (bool with_gui = false)
  {
    if (!loadCloud (filename_segmentation_, segmentation_cloud_) ||
        !loadCloud (filename_groundtruth_, groundtruth_cloud_))
      return (1);

    assert (segmentation_cloud_.points.size () == groundtruth_cloud_.points.size ());

    computeOverlap ();
    computeError ();

    size_t error = error_indices_.size ();

    if (with_gui)
    {
      auto viewer = create ();
      size_t size = getRealCloudSize (groundtruth_cloud_);
      float p = 100.0 * error / size;
      pcl::console::print_error ("Erroneus points: %i (%.2f%% of %zu)\n", error, p, size);
      viewer->add
      ( CreatePointCloudObject<pcl::PointXYZRGBA> ("incorrect", "i")
      . description                               ("Incorrectly segmented points")
      . data                                      (getCloud (MODE_BY_ERROR_FLAG))
      . pointSize                                 (2)
      . visibility                                (0.95)
      );
      viewer->add
      ( CreatePointCloudWithColorShufflingObject ("groundtruth", "t")
      . description                              ("Groundtruth labeling")
      . data                                     (getCloud (MODE_BY_GROUNDTRUTH_LABEL))
      . pointSize                                (2)
      . visibility                               (0.95)
      );
      viewer->add
      ( CreatePointCloudWithColorShufflingObject ("segmentation", "s")
      . description                              ("Segmentation labeling")
      . data                                     (getCloud (MODE_BY_SEGMENTATION_LABEL))
      . pointSize                                (2)
      . visibility                               (0.95)
      );
      viewer->show ("incorrect");
      viewer->run ();
    }
    else
    {
      std::cout << error << "\n";
    }

    return (0);
  }

private:

  typedef uint32_t Label;
  typedef std::map<Label, Color> LabelColorMap;
  typedef std::map<size_t, Color> IndexColorMap;

  enum Mode
  {
    MODE_BY_SEGMENTATION_LABEL,
    MODE_BY_GROUNDTRUTH_LABEL,
    MODE_BY_ERROR_FLAG,
  };

  struct Segment
  {
    // Number of vertices
    size_t size;

    // Overlaps with other labels
    std::map<Label, std::vector<size_t>> overlap;

    // Register an overlap with a point
    void overlapsWith (Label l, size_t index)
    {
      size += 1;
      if (overlap.count (l))
        overlap[l].push_back (index);
      else
        overlap[l] = {index};
    }

    // Get number of different labels with which segment overlaps
    size_t getNumberOfOverlaps () const
    {
      return overlap.size ();
    }

    std::pair<Label, size_t> getLargestOverlap () const
    {
      Label label;
      size_t max = 0;
      for (const auto& li_pair : overlap)
      {
        if (li_pair.second.size () > max)
        {
          label = li_pair.first;
          max = li_pair.second.size ();
        }
      }
      return std::make_pair (label, max);
    }
  };

  static bool
  loadCloud (const std::string& filename, pcl::PointCloud<pcl::PointXYZL>& cloud)
  {
    if (pcl::io::loadPCDFile (filename, cloud) < 0)
      return (false);
    return (true);
  }

  void computeOverlap ()
  {
    for (size_t i = 0; i < groundtruth_cloud_.size (); ++i)
    {
      auto& pg = groundtruth_cloud_.at (i);
      auto& ps = segmentation_cloud_.at (i);
      if (pcl::isFinite (pg))
        overlap_map_[pg.label].overlapsWith (ps.label, i);
    }
  }

  void computeError ()
  {
    typedef std::tuple<Label, Label, size_t> Triple;
    std::vector<Triple> triples;
    for (const auto& label_segment_pair : overlap_map_)
      for (const auto& label_indices_pair : label_segment_pair.second.overlap)
        triples.push_back (Triple (label_segment_pair.first,
                                   label_indices_pair.first,
                                   label_indices_pair.second.size ()));
    auto assignment = findAssignment (triples);
    error_indices_.clear ();
    for (const auto& label_segment_pair : overlap_map_)
      for (const auto& label_indices_pair : label_segment_pair.second.overlap)
        if (label_indices_pair.first != assignment[label_segment_pair.first])
            error_indices_.insert (error_indices_.end (), label_indices_pair.second.begin (), label_indices_pair.second.end ());
  }

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colorizeByLabel (const pcl::PointCloud<pcl::PointXYZL>& original, LabelColorMap& color_map)
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::copyPointCloud (original, *cloud);
    for (size_t i = 0; i < cloud->size (); i++)
    {
      cloud->at (i).rgba = color_map[original.at (i).label];
    }
    return cloud;
  }

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colorizeByIndex (const pcl::PointCloud<pcl::PointXYZL>& original, IndexColorMap& color_map)
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::copyPointCloud (original, *cloud);
    for (size_t i = 0; i < cloud->size (); i++)
    {
      cloud->at (i).rgba = color_map[i];
    }
    return cloud;
  }

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getCloud (Mode mode)
  {
    switch (mode)
    {
      case MODE_BY_SEGMENTATION_LABEL:
      {
        std::map<Label, Color> label_map;
        for (const auto& pt : segmentation_cloud_)
        {
          Label label = pt.label;
          if (!label_map.count (label))
            label_map[label] = generateRandomColor ();
        }
        return colorizeByLabel (segmentation_cloud_, label_map);
      }
      case MODE_BY_GROUNDTRUTH_LABEL:
      {
        std::map<Label, Color> label_map;
        for (const auto& pt : groundtruth_cloud_)
        {
          Label label = pt.label;
          if (!label_map.count (label))
            label_map[label] = generateRandomColor ();
        }
        return colorizeByLabel (groundtruth_cloud_, label_map);
      }
      case MODE_BY_ERROR_FLAG:
      {
        std::map<size_t, Color> index_map;
        Color normal = 0x0EEBA1;
        Color error = 0xEB0E58;
        for (size_t i = 0; i < groundtruth_cloud_.size (); i++)
        {
          index_map[i] = normal;
        }
        for (size_t i = 0; i < error_indices_.size (); i++)
        {
          index_map[error_indices_[i]] = error;
        }
        return colorizeByIndex (groundtruth_cloud_, index_map);
      }
    }
  }

  size_t
  getRealCloudSize (const pcl::PointCloud<pcl::PointXYZL>& cloud)
  {
    size_t cnt = 0;
    for (const auto& point : cloud.points)
      if (pcl::isFinite (point))
        ++cnt;
    return cnt;
  }

  std::map<Label, Segment> overlap_map_;
  std::vector<size_t> error_indices_;

  std::string filename_segmentation_;
  std::string filename_groundtruth_;

  pcl::PointCloud<pcl::PointXYZL> segmentation_cloud_;
  pcl::PointCloud<pcl::PointXYZL> groundtruth_cloud_;

};


int main(int argc, char** argv)
{
  std::string segmentation, groundtruth;

  if (argc >= 3)
  {
    segmentation = argv[1];
    groundtruth = argv[2];
  }
  else
  {
    pcl::console::print_error ("Usage: %s segmentation groundtruth [options]\n", argv[0]);
    return (1);
  }

  bool with_gui = pcl::console::find_switch (argc, argv, "--gui");

  SegmentationEvaluation se (segmentation, groundtruth);

  return (se.run (with_gui));
}
