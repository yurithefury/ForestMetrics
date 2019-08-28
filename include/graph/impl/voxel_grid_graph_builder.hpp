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

#ifndef PCL_GRAPH_IMPL_VOXEL_GRID_GRAPH_BUILDER_HPP
#define PCL_GRAPH_IMPL_VOXEL_GRID_GRAPH_BUILDER_HPP

#include <boost/unordered_map.hpp>

#include <pcl/common/io.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/octree/octree_impl.h>

#include "graph/voxel_grid_graph_builder.h"

/* The function below is required in order to use boost::unordered_map with
 * pcl::octree::OctreeKey key type. It simply hashes the x, y, z array of
 * indices, because it uniquely defines the key. */

namespace pcl
{

  namespace octree
  {

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-function"

    static size_t hash_value (const OctreeKey& b)
    {
      return boost::hash_value (b.key_);
    }

#pragma clang diagnostic pop

  }

}


template <typename PointT, typename GraphT> void
pcl::graph::VoxelGridGraphBuilder<PointT, GraphT>::compute (GraphT& graph)
{
  if (!initCompute ())
  {
    graph = GraphT ();
    deinitCompute ();
    return;
  }

  typename pcl::PointCloud<PointT>::Ptr transformed (new pcl::PointCloud<PointT>);
  pcl::copyPointCloud (*input_, *transformed);
  for (size_t i = 0; i < transformed->size (); ++i)
  {
    PointT& p = transformed->points[i];
    p.x /= p.z;
    p.y /= p.z;
    p.z = std::log (p.z);
  }

  Eigen::Vector4f min, max;
  pcl::getMinMax3D (*transformed, *indices_, min, max);

  // Create and initialize an Octree that stores point indices
  typedef pcl::octree::OctreePointCloud<PointT> Octree;
  Octree octree (voxel_resolution_);
  octree.defineBoundingBox (min (0), min (1), min (2), max (0), max (1), max (2));
  octree.setInputCloud (transformed, indices_);
  octree.addPointsFromInputCloud ();

  graph = GraphT (octree.getLeafCount ());

  typedef boost::unordered_map<pcl::octree::OctreeKey, VertexId> KeyVertexMap;
  KeyVertexMap key_to_vertex_map;

  point_to_vertex_map_.clear ();
  point_to_vertex_map_.resize (transformed->size (), std::numeric_limits<VertexId>::max ());

  typename Octree::LeafNodeIterator leaf_itr = octree.leaf_begin ();
  for (VertexId v = 0; leaf_itr != octree.leaf_end (); ++leaf_itr, ++v)
  {
    // Step 1: compute leaf centroid and fill in corresponding elements of the
    // point to vertex map.
    pcl::CentroidPoint<PointInT> centroid;
    std::vector<int>& indices = leaf_itr.getLeafContainer ().getPointIndicesVector ();
    for (size_t i = 0; i < indices.size (); ++i)
    {
      centroid.add (input_->operator[] (indices[i]));
      point_to_vertex_map_[indices[i]] = v;
    }
    centroid.get (graph[v]);

    // Step 2: fill in octree key to vertex map.
    octree::OctreeKey key = leaf_itr.getCurrentOctreeKey ();
    key_to_vertex_map[key] = v;

    // Step 2: find neighbors and insert edges.
    octree::OctreeKey neighbor_key;
    for (int dx = (key.x > 0) ? -1 : 0; dx <= 1; ++dx)
    {
      neighbor_key.x = static_cast<uint32_t> (key.x + dx);
      for (int dy = (key.y > 0) ? -1 : 0; dy <= 1; ++dy)
      {
        neighbor_key.y = static_cast<uint32_t> (key.y + dy);
        for (int dz = (key.z > 0) ? -1 : 0; dz <= 1; ++dz)
        {
          neighbor_key.z = static_cast<uint32_t> (key.z + dz);
          typename KeyVertexMap::iterator f = key_to_vertex_map.find (neighbor_key);
          if (f != key_to_vertex_map.end () && v != f->second)
            boost::add_edge (v, f->second, graph);
        }
      }
    }
  }
}

#endif /* PCL_GRAPH_IMPL_VOXEL_GRID_GRAPH_BUILDER_HPP */

