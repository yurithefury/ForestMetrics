#ifndef LABEL_UTILS_H
#define LABEL_UTILS_H

#include <boost/property_map/property_map.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

namespace labels
{

  template <typename PointT, typename ColorMap> pcl::PointCloud<pcl::PointXYZL>::Ptr
  createLabeledCloudFromColorMap (const typename pcl::PointCloud<PointT>& cloud,
                                  const std::vector<typename boost::property_traits<ColorMap>::key_type>& point_to_vertex_map,
                                  ColorMap color_map)
  {
    typedef typename boost::property_traits<ColorMap>::key_type VertexId;
    const VertexId NIL = std::numeric_limits<VertexId>::max ();

    pcl::PointCloud<pcl::PointXYZL>::Ptr labeled (new pcl::PointCloud<pcl::PointXYZL>);
    pcl::copyPointCloud (cloud, *labeled);

    for (size_t i = 0; i < labeled->size (); ++i)
    {
      const VertexId& v = point_to_vertex_map[i];
      labeled->at (i).label = (v == NIL) ? 0 : color_map[v];
    }

    return labeled;
  }

  /** Creates a new cloud of PointXYZL points with labels that match colors.
    *
    * The source cloud is copied over. NaN points get 0 label, finite points
    * get labels from 1 up depending on their RGBA color. */
  template <typename PointT> pcl::PointCloud<pcl::PointXYZL>::Ptr
  createLabeledCloudFromColoredCloud (const typename pcl::PointCloud<PointT>& colored)
  {
    pcl::PointCloud<pcl::PointXYZL>::Ptr labeled (new pcl::PointCloud<pcl::PointXYZL>);
    pcl::copyPointCloud (colored, *labeled);

    std::map<uint32_t, uint32_t> color_label_map;
    for (size_t i = 0; i < labeled->size (); ++i)
    {
      if (pcl::isFinite (colored[i]))
      {
        auto color = colored[i].rgba;
        if (color_label_map.count (color) == 0)
          color_label_map.insert (std::make_pair (color, color_label_map.size () + 1));
        labeled->at (i).label = color_label_map[color];
      }
    }

    return labeled;
  }

}


#endif /* LABEL_UTILS_H */

