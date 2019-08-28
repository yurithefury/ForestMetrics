#ifndef SEED_UTILS_H
#define SEED_UTILS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/utils.h>
#include <pcl/kdtree/io.h>

#include "tviewer/color.h"

typedef pcl::PointXYZL SeedT;
typedef pcl::PointCloud<SeedT> SeedCloudT;
typedef SeedCloudT::Ptr SeedCloudTPtr;

namespace seeds
{

typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
createColoredCloudFromSeeds (const SeedCloudT& seeds)
{
  typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  cloud->resize (seeds.size ());
  std::map<uint32_t, tviewer::Color> color_map;
  for (size_t i = 0; i < seeds.size (); ++i)
  {
    const auto& s = seeds.at (i);
    auto& p = cloud->at (i);
    p.x = s.x;
    p.y = s.y;
    p.z = s.z;
    if (color_map.count (s.label) == 0)
      color_map[s.label] = tviewer::generateRandomColor ();
    p.rgba = color_map[s.label];
  }
  return cloud;
}

/* For each point in the given cloud create a seed in the output cloud.
 * Each seed gets a distinct label starting from 1. */
template <typename PointT> typename SeedCloudT::Ptr
createSeedCloudFromPointCloud (const pcl::PointCloud<PointT>& cloud)
{
  typename SeedCloudT::Ptr seeds (new SeedCloudT);
  seeds->resize (cloud.size ());
  uint32_t label = 1;
  for (size_t i = 0; i < cloud.size (); ++i)
  {
    seeds->at (i).getVector3fMap () = cloud.at (i).getVector3fMap ();
    seeds->at (i).label = label++;
  }
  return seeds;
}

/* For each distinct label in the seeds cloud this function will create
 * PointIndices with the indices of points of the original cloud that
 * have exactly the same coordinates (up to epsilon) as seeds with this
 * label. The return value is true only if a corresponding point index
 * was found for every seed. */
template <typename PointT> bool
findSeedIndicesInCloud (const typename pcl::PointCloud<PointT>::Ptr& cloud,
                        const typename SeedCloudT::Ptr& seeds,
                        std::vector<pcl::PointIndices>& indices)
{
  using namespace pcl::utils;
  std::map<uint32_t, size_t> label_map;
  indices.clear ();
  bool all_found = true;
  for (const auto& seed : seeds->points)
  {
    if (label_map.count (seed.label) == 0)
    {
      label_map[seed.label] = indices.size ();
      indices.push_back (pcl::PointIndices ());
    }
    size_t group = label_map[seed.label];
    bool point_found = false;
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
      const auto& pt = cloud->points[i];
      const float EPS = 0.00001;
      if (equal (seed.x, pt.x, EPS) && equal (seed.y, pt.y, EPS) && equal (seed.z, pt.z, EPS))
      {
        indices[group].indices.push_back (i);
        point_found = true;
        break;
      }
    }
    all_found &= point_found;
  }
  return all_found;
}

/* For each distinct label in the seeds cloud this function will create
 * PointIndices with the indices of points of the original cloud that
 * have approximately the same coordinates as seeds with this label. The
 * return value is always true. */
template <typename PointT> bool
findSeedIndicesInCloudApproximate (const typename pcl::PointCloud<PointT>::Ptr& cloud,
                                   const typename SeedCloudT::Ptr& seeds,
                                   std::vector<pcl::PointIndices>& indices)
{
  indices.clear ();
  std::vector<int> v;
  typename pcl::PointCloud<PointT>::Ptr seeds_copy (new pcl::PointCloud<PointT>);
  pcl::copyPointCloud (*seeds, *seeds_copy);
  pcl::getApproximateIndices<PointT> (seeds_copy, cloud, v);
  assert (seeds->size () == v.size ());
  std::map<uint32_t, size_t> label_map;
  for (size_t i = 0; i < seeds->size (); ++i)
  {
    const auto& seed = seeds->at (i);
    if (label_map.count (seed.label) == 0)
    {
      label_map[seed.label] = indices.size ();
      indices.push_back (pcl::PointIndices ());
    }
    indices[label_map[seed.label]].indices.push_back (v[i]);
  }
  return true;
}

}

#endif /* SEED_UTILS_H */

