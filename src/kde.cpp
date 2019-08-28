#include "kde.h"

KDE::KDE (std::vector<float>&& dataset)
: dataset_ (std::move (dataset))
{
  assert (dataset_.size () % 2 == 0);
  flann::Matrix<float> D (dataset_.data (), dataset_.size () / 2, 2);
  kdtree_.reset (new KdTree (flann::KDTreeSingleIndexParams ()));
  kdtree_->buildIndex (D);
  setBandwidth (1.0);
}

void
KDE::setBandwidth (float bandwidth)
{
  bandwidth_ = bandwidth;
  auto m = -0.5 / (bandwidth_ * bandwidth_);
  // Gaussian kernel sans coefficient in front
  kernel_ = [=](float dist){ return std::exp (m * dist * dist); };
  float M = 0.01; // not interested in points contributing less than this
  radius_ = bandwidth_ * std::sqrt (-2 * std::log (M));
}

float
KDE::evaluateAt (float x, float y) const
{
  // TODO: this finds distance to nearest neighbor, not density estimate
  float query[] = {x, y};
  flann::Matrix<float> Q (query, 1, 2);
  int index;
  flann::Matrix<int> I (&index, 1, 1);
  float distance;
  flann::Matrix<float> D (&distance, 1, 1);
  kdtree_->knnSearch(Q, I, D, 1, flann::SearchParams (128));
  return distance;
}

std::vector<float>
KDE::evaluateAt (std::vector<float>& query) const
{
  assert (query.size () % 2 == 0);
  size_t size = query.size () / 2;
  flann::Matrix<float> Q (query.data (), size, 2);
  std::vector<std::vector<int>> indices;
  std::vector<std::vector<float>> distances;
  kdtree_->radiusSearch(Q, indices, distances, radius_, flann::SearchParams (32, 0, false));
  std::vector<float> densities (size, 0.0);
  for (size_t i = 0; i < size; ++i)
    for (const auto& d : distances[i])
      densities[i] += kernel_ (d);
  return densities;
}

