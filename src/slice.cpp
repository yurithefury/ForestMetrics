#include <string>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

#include "tviewer/tviewer.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointXYZCloud;
typedef pcl::PointCloud<pcl::PointXYZL> PointXYZLCloud;
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointXYZRGBACloud;

template <typename PointT> using Cells = std::vector<std::vector<pcl::PointCloud<PointT>>>;

PointXYZRGBACloud::Ptr
highlightGrid (const PointXYZCloud& cloud, float min_x, float min_y, double size, double overlap)
{
  PointXYZRGBACloud::Ptr highlighted (new PointXYZRGBACloud);
  pcl::copyPointCloud (cloud, *highlighted);
  double step = size - overlap;
  for (auto& point : highlighted->points)
  {
    if (pcl::isFinite (point))
    {
      double x = point.x - min_x;
      double y = point.y - min_y;
      int x_cell = std::floor (x / step);
      int y_cell = std::floor (y / step);
      if ((x_cell > 0 && x - step * x_cell < overlap) ||
          (y_cell > 0 && y - step * y_cell < overlap))
        point.rgba = 0xFF00F0F0;
      else
        point.rgba = 0xFFFFFF00;
    }
  }
  return highlighted;
}

template <typename PointT> void
addPointToCell (const PointT& point, int cell_x, int cell_y, Cells<PointT>& cells)
{
  if (cell_x < 0 || cell_y < 0)
    return;
  if (cells.size () <= cell_y)
    cells.resize (cell_y + 1);
  if (cells[cell_y].size () <= cell_x)
    cells[cell_y].resize (cell_x + 1);
  cells[cell_y][cell_x].points.push_back (point);
}

template <typename PointT> void
sliceGrid (const pcl::PointCloud<PointT>& cloud, float min_x, float min_y, double size, double overlap, const std::string& name_template)
{
  double step = size - overlap;
  int columns = 0;
  Cells<PointT> cells;
  for (auto& point : cloud.points)
  {
    if (pcl::isFinite (point))
    {
      double x = point.x - min_x;
      double y = point.y - min_y;
      int x_cell = std::floor (x / step);
      int y_cell = std::floor (y / step);
      if (x_cell + 1 > columns)
        columns = x_cell + 1;
      addPointToCell (point, x_cell, y_cell, cells);
      if (x - step * x_cell < overlap)
        addPointToCell (point, x_cell - 1, y_cell, cells);
      if (y - step * y_cell < overlap)
        addPointToCell (point, x_cell, y_cell - 1, cells);
    }
  }
  boost::format fname (name_template);
  pcl::PointCloud<PointT> empty (0, 0);
  for (size_t i = 0; i < cells.size (); ++i)
  {
    for (size_t j = 0; j < columns; ++j)
    {
      if (cells[i].size () > j && cells[i][j].size ())
      {
        cells[i][j].width = cells[i][j].size ();
        cells[i][j].height = 1;
        pcl::io::savePCDFile (boost::str (fname % i % j), cells[i][j]);
      }
    }
  }
}

int
main (int argc, char** argv)
{
  if (argc < 2 || pcl::console::find_switch (argc, argv, "--help"))
  {
    pcl::console::print_error ("Usage: %s <pointcloud.pcd> [seeds.pcd]\n", argv[0]);
    pcl::console::print_value ("\n"
                               "  --size <double>\n"
                               "  --overlap <double>\n");
    pcl::console::print_info  ("\n"
                               "  Dissect point cloud into square cells. Each resulting cell is\n"
                               "  saved in a separate file 'cell-X-Y.pcd', where 'X' and 'Y' are \n"
                               "  the cell coordinates in the grid.\n"
                               "\n"
                               "  An optional second file (seeds) can be supplied, in this case\n"
                               "  it will dissected with the same parameters.\n"
                               "\n"
                               "  The application will present a GUI for visual inspection. The\n"
                               "  parameters can then be adjusted with a/A and s/S keys. Press 'e'\n"
                               "  to output cells to the disk and exit when satisfied with the\n"
                               "  result.\n"
                               "\n");
    return (1);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile (argv[1], *cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file %s\n", argv[1]);
    return (2);
  }

  pcl::PointCloud<pcl::PointXYZL>::Ptr seeds (new pcl::PointCloud<pcl::PointXYZL>);
  if (argc > 2 && boost::algorithm::ends_with (argv[2], ".pcd"))
  {
    if (pcl::io::loadPCDFile (argv[2], *seeds) == -1)
    {
      PCL_ERROR ("Couldn't read file %s\n", argv[2]);
      return (3);
    }
  }

  float size = 1.0f;
  pcl::console::parse (argc, argv, "--size", size);

  float overlap = 0.1f;
  pcl::console::parse (argc, argv, "--overlap", overlap);

  if (size <= overlap)
  {
    PCL_ERROR ("Grid size should be larger than overlap\n");
    return (4);
  }

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_with_highlight (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::copyPointCloud (*cloud, *cloud_with_highlight);

  pcl::PointXYZ min;
  pcl::PointXYZ max;
  pcl::getMinMax3D (*cloud, min, max);

  using namespace tviewer;
  auto viewer = create (argc, argv);

  typedef UpDownCounter<double> Counter;

  Counter::Ptr size_counter = CreateUpDownCounter<double> ("size", "s")
                    . init (size)
                    . step (1.0)
                    . min (1.0)
                    . max (100)
                    . printOnChange ();

  Counter::Ptr overlap_counter = CreateUpDownCounter<double> ("overlap", "a")
                       . init (overlap)
                       . step (0.5)
                       . min (0.5)
                       . max (10)
                       . printOnChange ();

  viewer->add
  ( CreatePointCloudObject<pcl::PointXYZRGBA> ("pointcloud", "p")
  . description                               ("Pointcloud")
  . pointSize                                 (1)
  . onUpdate                                  ([&]{ return highlightGrid (*cloud, min.x, min.y, *size_counter, *overlap_counter); })
  , true
  , true
  );

  viewer->addListener (size_counter, { "pointcloud" });
  viewer->addListener (overlap_counter, { "pointcloud" });

  viewer->run ();

  sliceGrid (*cloud, min.x, min.y, *size_counter, *overlap_counter, "cell-%i-%i.pcd");
  if (seeds->size ())
    sliceGrid (*seeds, min.x, min.y, *size_counter, *overlap_counter, "cell-%i-%i-seeds.pcd");

  return (0);
}

