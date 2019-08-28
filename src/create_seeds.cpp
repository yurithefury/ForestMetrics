#include <string>

#include <boost/filesystem.hpp>

#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

int
main (int argc, char** argv)
{
  if (argc < 3 || pcl::console::find_switch (argc, argv, "--help"))
  {
    pcl::console::print_error ("Usage: %s <directory> <output>\n", argv[0]);
    pcl::console::print_info  ("Directory should contain multiple PCD files, one file for each label\n");
    return (1);
  }

  namespace fs = boost::filesystem;
  fs::path directory (argv[1]);
  fs::directory_iterator end_iter;

  typedef std::multimap<std::time_t, fs::path> result_set_t;
  result_set_t result_set;

  if (!fs::exists (directory) || !fs::is_directory (directory))
  {
    pcl::console::print_error ("\"%s\" does not exist or is not a directory\n", argv[1]);
    return (2);
  }

  typedef pcl::PointCloud<pcl::PointXYZL> SeedCloud;
  SeedCloud seeds;
  int label = 1;
  for (fs::directory_iterator dir_iter(directory) ; dir_iter != end_iter ; ++dir_iter)
  {
    if (fs::is_regular_file (dir_iter->status ()) && fs::extension (*dir_iter) == ".pcd")
    {
      pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
      if (pcl::io::loadPCDFile (dir_iter->path ().string (), xyz_cloud) >= 0)
      {
        SeedCloud seed_cloud;
        pcl::copyPointCloud (xyz_cloud, seed_cloud);
        for (size_t i = 0; i < seed_cloud.size (); ++i)
          seed_cloud[i].label = label;
        seeds += seed_cloud;
        ++label;
      }
    }
  }

  pcl::io::savePCDFile (argv[2], seeds);

  return (0);
}

