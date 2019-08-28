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

#ifndef PCL_GRAPH_UTILS_H
#define PCL_GRAPH_UTILS_H

#include <boost/mpl/has_xxx.hpp>

namespace pcl
{

  namespace graph
  {

    namespace detail
    {
      // Create a meta-function that checks if a type has "graph_type" member
      // typedef. This will be used to distinguish between boost::subgraph (does
      // have), and normal graph types (do not have).
      BOOST_MPL_HAS_XXX_TRAIT_NAMED_DEF(has_root_graph, graph_type, false)
    }

    /** remove_edge_if structure is an "extended" version of,
      * boost::remove_edge_if that incorporates a workaround to allow edge
      * removal from both plain graphs and subgraphs.
      *
      * Plain graphs should be passed to boost::remove_edge_if as is, whereas
      * member field `m_graph` should be used in place of the subgraphs
      * (otherwise expect segfaults). */
    template <typename Graph, typename Enable = void>
    struct remove_edge_if
    {
      template <typename Predicate> void
      operator () (const Predicate& predicate, Graph& graph) const
      {
        boost::remove_edge_if (predicate, graph);
      }
    };

    template <typename Graph>
    struct remove_edge_if<Graph, typename boost::enable_if<detail::has_root_graph<Graph> >::type>
    {
      template <typename Predicate> void
      operator () (const Predicate& predicate, Graph& graph) const
      {
        boost::remove_edge_if (predicate, graph.m_graph);
      }
    };

  }

}

#endif /* PCL_GRAPH_UTILS_H */

