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

#ifndef PCL_GRAPH_IMPL_EDGE_WEIGHT_COMPUTER_HPP
#define PCL_GRAPH_IMPL_EDGE_WEIGHT_COMPUTER_HPP

#include "graph/point_cloud_graph.h"
#include "graph/edge_weight_computer.h"
#include "graph/utils.h"

namespace pcl
{

  namespace graph
  {

    namespace detail
    {

      /** A predicate to be used with pcl::graph::remove_edge_if, returns
        * \c true when the edge wight is below a certain threshold.
        *
        * Properly handles both plain graphs and graphs wrapped in
        * boost::subgraph. */
      template <typename EdgeWeightMap>
      struct remove_edge_predicate
      {

        remove_edge_predicate (const EdgeWeightMap& weights, float threshold)
        : weights_ (weights)
        , threshold_ (threshold)
        {
        }

        template <typename EdgeDescriptor> bool
        operator () (EdgeDescriptor e) const
        {
          return (!std::isfinite (weights_[e]) || weights_[e] < threshold_);
        }

        const EdgeWeightMap& weights_;
        float threshold_;

      };

      /** A helper functor to determine if two points are convex with respect
        * to each other.
        *
        * Returns \c false if the point type does not have *curvature* field.
        * Otherwise the points are assumed to be relatively convex if both
        * curvature values are positive. */
      template <typename PointT, typename Enable = void>
      struct IsConvex
      {
        bool operator () (const PointT& p1, const PointT& p2) const
        {
          return false;
        }
      };

      template <typename PointT>
      struct IsConvex<PointT, typename boost::enable_if<pcl::traits::has_curvature<PointT> >::type>
      {
        bool operator () (const PointT& p1, const PointT& p2) const
        {
          return (p1.curvature > 0.0f && p2.curvature > 0.0f);
        }
      };

    }

  }

}

template <typename GraphT>
template <class EdgeWeightMap> void
pcl::graph::EdgeWeightComputer<GraphT>::compute (GraphT& graph, EdgeWeightMap weights)
{
  typename boost::graph_traits<GraphT>::edge_iterator ei, ee;

  // Step 1: do precomputation for normalized terms (if any).
  if (g_terms_.size () || l_terms_.size ())
  {
    for (size_t i = 0; i < g_terms_.size (); ++i)
      g_terms_[i].init (boost::num_edges (graph));
    for (size_t i = 0; i < l_terms_.size (); ++i)
      l_terms_[i].init (boost::num_edges (graph), boost::num_vertices (graph));

    size_t edge_id = 0;
    for (boost::tie (ei, ee) = boost::edges (graph); ei != ee; ++ei, ++edge_id)
    {
      typename boost::graph_traits<GraphT>::vertex_descriptor v1, v2;
      v1 = boost::source (*ei, graph), v2 = boost::target (*ei, graph);
      for (size_t i = 0; i < g_terms_.size (); ++i)
        g_terms_[i].round1 (graph[v1], graph[v2], edge_id);
      for (size_t i = 0; i < l_terms_.size (); ++i)
        l_terms_[i].round1 (graph[v1], graph[v2], v1, v2, edge_id);
    }

    for (size_t i = 0; i < g_terms_.size (); ++i)
      g_terms_[i].extract ();
    for (size_t i = 0; i < l_terms_.size (); ++i)
      l_terms_[i].extract ();
  }

  // Step 2: compute weight for each edge.
  detail::IsConvex<PointT> is_convex;

  size_t edge_id = 0;
  for (boost::tie (ei, ee) = boost::edges (graph); ei != ee; ++ei, ++edge_id)
  {
    typename boost::graph_traits<GraphT>::vertex_descriptor v1, v2;
    v1 = boost::source (*ei, graph), v2 = boost::target (*ei, graph);
    const PointT& p1 = graph[v1];
    const PointT& p2 = graph[v2];
    float a = 1.0;
    for (size_t i = 0; i < terms_.size (); ++i)
      a *= balancing_function_ (terms_[i].compute_ (p1, p2), terms_[i].getInfluence (is_convex (p1, p2)));
    for (size_t i = 0; i < g_terms_.size (); ++i)
      a *= balancing_function_ (g_terms_[i].round2 (edge_id), g_terms_[i].getInfluence (is_convex (p1, p2)));
    for (size_t i = 0; i < l_terms_.size (); ++i)
      a *= balancing_function_ (l_terms_[i].round2 (v1, v2, edge_id), l_terms_[i].getInfluence (is_convex (p1, p2)));
    weights[*ei] = a;
  }

  // Step 3: find edges with very small weight and modify them according to the
  // policy set by the user.
  switch (policy_)
  {
    case SMALL_WEIGHT_IGNORE:
      {
        break;
      }
    case SMALL_WEIGHT_COERCE_TO_THRESHOLD:
      {
        typedef typename boost::graph_traits<GraphT>::edge_iterator EdgeIterator;
        EdgeIterator ei, ee;
        for (boost::tie (ei, ee) = boost::edges (graph); ei != ee; ++ei)
          if (!std::isfinite(weights[*ei]) || weights[*ei] < threshold_)
            weights[*ei] = threshold_;
        break;
      }
    case SMALL_WEIGHT_REMOVE_EDGE:
      {
        // Note: it is only okay to remove edges this way before any subgraph
        // was created.
        detail::remove_edge_predicate<EdgeWeightMap> predicate (weights, threshold_);
        remove_edge_if<GraphT> re;
        re (predicate, graph);
        break;
      }
  }
}

#endif /* PCL_GRAPH_IMPL_EDGE_WEIGHT_COMPUTER_HPP */

