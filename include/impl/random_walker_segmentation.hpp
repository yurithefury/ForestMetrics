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

#ifndef PCL_SEGMENTATION_IMPL_RANDOM_WALKER_SEGMENTATION_HPP
#define PCL_SEGMENTATION_IMPL_RANDOM_WALKER_SEGMENTATION_HPP

#include <boost/make_shared.hpp>

#include <pcl/search/kdtree.h>
#include <pcl/kdtree/io.h>

#include "random_walker.h"
#include "random_walker_segmentation.h"
#include "graph/common.h"
#include "graph/edge_weight_computer.h"

#include "measure_runtime.h"

template <typename PointT>
pcl::segmentation::RandomWalkerSegmentation<PointT>::RandomWalkerSegmentation (bool store_potentials)
: zero_label_replacement_ (0xFFFFFFFF)
, input_as_cloud_ (true)
, graph_builder_ (0.006f)
, labeling_method_ (DIFFERENT_GENERATED_LABELS)
, store_potentials_ (store_potentials)
{
}

template <typename PointT>
pcl::segmentation::RandomWalkerSegmentation<PointT>::~RandomWalkerSegmentation ()
{
}

template <typename PointT> void
pcl::segmentation::RandomWalkerSegmentation<PointT>::setInputCloud (const PointCloudConstPtr &cloud)
{
  pcl::PCLBase<PointT>::setInputCloud (cloud);
  input_as_cloud_ = true;
  graph_components_.clear (); // invalidate connected components
}

template <typename PointT> void
pcl::segmentation::RandomWalkerSegmentation<PointT>::setInputGraph (const GraphPtr& graph)
{
  graph_ = graph;
  input_as_cloud_ = false;
  graph_components_.clear (); // invalidate connected components
}

template <typename PointT> void
pcl::segmentation::RandomWalkerSegmentation<PointT>::setSeeds (const pcl::PointCloud<PointXYZL>::ConstPtr& seeds)
{
  seeds_ = seeds;

  // Find unique labels
  labels_.clear ();
  for (size_t i = 0; i < seeds_->size (); ++i)
    labels_.insert (seeds_->at (i).label);

  // Associate labels with (ascending) indices
  label_index_map_.clear ();
  for (std::set<uint32_t>::iterator iter = labels_.begin (); iter != labels_.end (); ++iter)
    label_index_map_.insert (std::make_pair (*iter, label_index_map_.size ()));
}

template <typename PointT> void
pcl::segmentation::RandomWalkerSegmentation<PointT>::preComputeGraph ()
{
  if (input_as_cloud_)
  {
    if (!initCompute ())
    {
      deinitCompute ();
      PCL_THROW_EXCEPTION (ComputeFailedException, "unable to pre-compute graph due to invalid input");
    }
    graph_.reset (new Graph);
    graph_builder_.setInputCloud (input_);
    graph_builder_.setIndices (indices_);
    MEASURE_RUNTIME ("Building graph... ", graph_builder_.compute (*graph_));
    MEASURE_RUNTIME ("Computing normals... ", pcl::graph::computeNormalsAndCurvatures (*graph_));
    MEASURE_RUNTIME ("Computing curvature signs... ", pcl::graph::computeSignedCurvatures (*graph_));
    {
      using namespace pcl::graph;
      typedef EdgeWeightComputer<Graph> EWC;
      EWC computer;
      computer.template addTerm<terms::XYZ> (3.0f, EWC::NORMALIZATION_GLOBAL);
      computer.template addTerm<terms::Normal> (0.01f, 0.0f);
      computer.template addTerm<terms::Curvature> (0.0001f, 0.0f);
      computer.template addTerm<terms::RGB> (3.0f, EWC::NORMALIZATION_GLOBAL);
      computer.setSmallWeightThreshold (1e-5);
      computer.setSmallWeightPolicy (EWC::SMALL_WEIGHT_COERCE_TO_THRESHOLD);
      MEASURE_RUNTIME ("Computing edge weights... ", computer.compute (*graph_));
    }
  }

  if (graph_components_.size () == 0)
    MEASURE_RUNTIME ("Computing connected components... ", pcl::graph::createSubgraphsFromConnectedComponents (*graph_, graph_components_));
}

template <typename PointT> void
pcl::segmentation::RandomWalkerSegmentation<PointT>::segment (std::vector<PointIndices>& clusters)
{
  preComputeGraph ();

  // Reset colors in graph
  VertexColorMap colors = boost::get (boost::vertex_color, *graph_);
  for (VertexId i = 0; i < boost::num_vertices (*graph_); ++i)
    colors[i] = 0; // TODO: use memset?

  // Find seeds
  std::vector<int> v;
  pcl::getApproximateIndices<PointXYZL, PointWithNormal> (seeds_, pcl::graph::point_cloud (*graph_), v);
  for (size_t i = 0; i < v.size (); ++i)
    colors[v[i]] = seeds_->at (i).label == 0
                 ? zero_label_replacement_
                 : seeds_->at (i).label;

  typedef std::map<uint32_t, size_t> ColorColumnMap;

  MEASURE_RUNTIME ("Random walker segmentation... ", {

  if (store_potentials_)
    // One row per vertex, one column per label
    potentials_ = Eigen::MatrixXf::Zero (boost::num_vertices (*graph_), labels_.size ());
  else
    // So that we have something to return if someone accidentally queries potentials
    potentials_ = Eigen::MatrixXf::Zero (0, 0);

  for (size_t i = 0; i < graph_components_.size (); ++i)
  {
    Graph& g = graph_components_.at (i).get ();
    bool success;
    if (store_potentials_)
    {
      Eigen::MatrixXf p;
      ColorColumnMap colors_to_columns_map;
      success = pcl::segmentation::randomWalker (g,
                                                 boost::get (boost::edge_weight, g),
                                                 boost::get (boost::vertex_color, g),
                                                 p,
                                                 colors_to_columns_map);
      for (ColorColumnMap::iterator iter = colors_to_columns_map.begin ();
           iter != colors_to_columns_map.end ();
           ++iter)
      {
        uint32_t label = iter->first != zero_label_replacement_ ? iter->first : 0;
        const size_t& local_column = iter->second;
        size_t global_column = label_index_map_[label];
        for (VertexId v = 0; v < boost::num_vertices (g); ++v)
          potentials_ (g.local_to_global (v), global_column) = p (v, local_column);
      }
    }
    else
    {
      success = pcl::segmentation::randomWalker (g,
                                                 boost::get (boost::edge_weight, g),
                                                 boost::get (boost::vertex_color, g));
    }
    if (success)
    {
      if (colors[g.local_to_global (0)] != 0)
        continue;
      uint32_t label;
      switch (labeling_method_)
      {
        case SAME_FIXED_LABEL: label = fixed_label_; break;
        case SAME_GENERATED_LABEL: label = findUnusedLabel(); break;
        case DIFFERENT_GENERATED_LABELS:
        {
          label = findUnusedLabel();
          labels_.insert (label);
          label_index_map_.insert (std::make_pair (label, label_index_map_.size ()));
          break;
        }
      }
      for (VertexId v = 0; v < boost::num_vertices (g); ++v)
        colors[g.local_to_global (v)] = label;
    }
    else
    {
      pcl::console::print_error ("Random walker segmentation failed in component #%zu\n", i);
    }
  }

  });

  for (VertexId v = 0; v < boost::num_vertices (*graph_); ++v)
    if (colors[v] == zero_label_replacement_)
      colors[v] = 0;

  clusters.clear ();
  clusters.resize (labels_.size ());

  if (input_as_cloud_)
  {
    const std::vector<VertexId>& point_to_vertex_map = graph_builder_.getPointToVertexMap ();
    for (size_t i = 0; i < input_->size (); ++i)
    {
      const VertexId& v = point_to_vertex_map[i];
      if (v >= boost::num_vertices (*graph_)) // "nil" vertex
        continue;
      else
        clusters[label_index_map_[colors[v]]].indices.push_back (v);
    }
  }
  else
  {
    for (VertexId v = 0; v < boost::num_vertices (*graph_); ++v)
      clusters[label_index_map_[colors[v]]].indices.push_back (v);
  }
}

template <typename PointT> const Eigen::MatrixXf&
pcl::segmentation::RandomWalkerSegmentation<PointT>::getPotentials () const
{
  if (!store_potentials_)
    PCL_WARN ("[pcl::segmentation::RandomWalkerSegmentation::getPotentials] "
              "Potential computation was disabeled at construction time, returning a zero matrix.");
  return (potentials_);
}

template <typename PointT> uint32_t
pcl::segmentation::RandomWalkerSegmentation<PointT>::findUnusedLabel () const
{
  for (size_t i = std::numeric_limits<uint32_t>::max () - 1; i != 0; --i)
    if (!labels_.count (i))
      return (i);
  return (std::numeric_limits<uint32_t>::max ());
}

#define PCL_INSTANTIATE_RandomWalkerSegmentation(T) template class pcl::segmentation::RandomWalkerSegmentation<T>;

#endif /* PCL_SEGMENTATION_IMPL_RANDOM_WALKER_SEGMENTATION_HPP */

