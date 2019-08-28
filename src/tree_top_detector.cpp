#include <vector>
#include <algorithm>

#include <flann/flann.hpp>

#include "tree_top_detector.h"

TreeTopDetector::TreeTopDetector ()
: cell_size_ (0.05)
{
  setRadius (0.5);
}

void
TreeTopDetector::setInputCloud (typename PointCloud::ConstPtr cloud)
{
  input_ = cloud;
}

void
TreeTopDetector::setRadius (float radius)
{
  radius_ = radius;
  cell_radius_ = std::ceil (radius_ / cell_size_);
}

void
TreeTopDetector::detect (std::vector<pcl::PointIndices>& tree_top_indices)
{
  tree_top_indices.clear ();

  Point min, max;
  pcl::getMinMax3D (*input_, min, max);

  float range_x = max.x - min.x;
  float range_y = max.y - min.y;
  int size_x = std::ceil (range_x / cell_size_);
  int size_y = std::ceil (range_y / cell_size_);

  density_.resize (size_x * size_y, 0);

  std::vector<int> indices (input_->size ());
  size_t n = 0;
  std::generate (std::begin (indices), std::end (indices), [&]{ return n++; });

  // Sort in descending order of Z coordinate
  std::sort (std::begin (indices),
             std::end (indices),
             [&](int i1, int i2) { return input_->at (i1).z > input_->at (i2).z; });

  for (size_t i = 0; i < indices.size (); ++i)
  {
    auto current_index = indices[i];
    auto current_point = input_->at (current_index);

    int cell_x = std::round ((current_point.x - min.x) / cell_size_);
    int cell_y = std::round ((current_point.y - min.y) / cell_size_);

    if (density_[cell_y * size_x + cell_x] == 0.0)
    {
      // candidates_.push ({cell_x, cell_y, current_index, current_point.z - 0.1f});
      pcl::PointIndices pi;
      pi.indices.push_back (current_index);
      tree_top_indices.push_back (pi);
    }

    for (int cx = std::max (0, cell_x - cell_radius_); cx < std::min (cell_x + cell_radius_, size_x); ++cx)
    {
      for (int cy = std::max (0, cell_y - cell_radius_); cy < std::min (cell_y + cell_radius_, size_y); ++cy)
      {
        float mcx = (0.5f + cx) * cell_size_ + min.x;
        float mcy = (0.5f + cy) * cell_size_ + min.y;

        if (std::pow (mcx - current_point.x, 2) + std::pow (mcy - current_point.y, 2) < radius_ * radius_)
        {
          density_[cy * size_x + cx] += 0.1;
        }
      }
    }
  }
}

const std::vector<float>&
TreeTopDetector::getDensity () const
{
  return density_;
}

