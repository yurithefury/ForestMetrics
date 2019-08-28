/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PCL_GRAPH_IMPL_NEAREST_NEIGHBORS_GRAPH_BUILDER_HPP
#define PCL_GRAPH_IMPL_NEAREST_NEIGHBORS_GRAPH_BUILDER_HPP

#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/point_tests.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>

#include "graph/nearest_neighbors_graph_builder.h"

template <typename PointT, typename GraphT> void
pcl::graph::NearestNeighborsGraphBuilder<PointT, GraphT>::compute (GraphT& graph)
{
  if (!initCompute ())
  {
    graph = GraphT ();
    deinitCompute ();
    return;
  }

  size_t k = 0;
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    if (!pcl::isFinite (input_->operator[] (indices_->operator[] (i))))
      fake_indices_ = false;
    else
      indices_->operator[] (k++) = indices_->operator[] (i);
  }
  indices_->resize (k);

  // Create a new point cloud which will be the basis for the constructed graph.
  // All the fields that are also present in the output point type will be
  // copied over from the original point cloud.
  typename pcl::PointCloud<PointOutT>::Ptr cloud (new pcl::PointCloud<PointOutT>);
  pcl::copyPointCloud (*input_, *indices_, *cloud);
  graph = GraphT (cloud);

  // In case a search method has not been given, initialize it using defaults
  if (!search_)
  {
    // For organized datasets, use OrganizedNeighbor
    if (cloud->isOrganized ())
      search_.reset (new pcl::search::OrganizedNeighbor<PointOutT>);
    // For unorganized data, use KdTree
    else
      search_.reset (new pcl::search::KdTree<PointOutT>);
  }

  // Establish edges with nearest neighbors.
  std::vector<int> neighbors (num_neighbors_ + 1);
  std::vector<float> distances (num_neighbors_ + 1);
  search_->setInputCloud (cloud);
  for (size_t i = 0; i < cloud->size (); ++i)
  {
    switch (search_type_)
    {
      case KNN:
        {
          // Search for num_neighbors_ + 1 because the first neighbor output by KdTree
          // is always the query point itself.
          search_->nearestKSearch (i, num_neighbors_ + 1, neighbors, distances);
          break;
        }
      case RADIUS:
        {
          search_->radiusSearch (i, radius_, neighbors, distances, num_neighbors_ + 1);
          break;
        }
    }
    for (size_t j = 1; j < neighbors.size (); ++j)
      if (!boost::edge (i, neighbors[j], graph).second)
        boost::add_edge (i, neighbors[j], graph);
  }

  // Create point to vertex map
  point_to_vertex_map_.resize (input_->size (), std::numeric_limits<VertexId>::max ());
  VertexId v = 0;
  for (size_t i = 0; i < indices_->size (); ++i)
    point_to_vertex_map_[indices_->operator[] (i)] = v++;
}

#endif /* PCL_GRAPH_IMPL_NEAREST_NEIGHBORS_GRAPH_BUILDER_HPP */

