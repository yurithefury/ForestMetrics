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

#ifndef PCL_GRAPH_IMPL_COMMON_HPP
#define PCL_GRAPH_IMPL_COMMON_HPP

#include <set>

#include <boost/concept_check.hpp>
#include <boost/graph/connected_components.hpp>

#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>

#include "graph/common.h"
#include "graph/point_cloud_graph.h"
#include "graph/point_cloud_graph_concept.h"

template <typename Graph> void
pcl::graph::computeNormalsAndCurvatures (Graph& graph, bool neighborhood_1ring)
{
  BOOST_CONCEPT_ASSERT ((pcl::graph::PointCloudGraphConcept<Graph>));

  typedef typename point_cloud_graph_traits<Graph>::point_type PointT;
  typedef typename Graph::adjacency_iterator AdjacencyIterator;
  typedef typename Graph::vertex_descriptor VertexId;

  typename pcl::PointCloud<PointT>::ConstPtr cloud = pcl::graph::point_cloud (graph);

  for (VertexId vertex = 0; vertex < boost::num_vertices (graph); ++vertex)
  {
    std::vector<int> neighbors (1, vertex);
    neighbors.reserve (256);

    AdjacencyIterator vi1, ve1, vi2, ve2;
    for (boost::tie (vi1, ve1) = boost::adjacent_vertices (vertex, graph); vi1 != ve1; ++vi1)
    {
      neighbors.push_back (*vi1);
      if (!neighborhood_1ring)
        for (boost::tie (vi2, ve2) = boost::adjacent_vertices (*vi1, graph); vi2 != ve2; ++vi2)
          neighbors.push_back (*vi2);
    }

    Eigen::Vector4f normal;
    float curvature;
    pcl::computePointNormal (*cloud, neighbors, normal, curvature);
    normal[3] = 0;
    normal.normalize ();

    // Re-orient normals. If the graph already has normals, then make sure that
    // the new normals are codirectional with them. Otherwise make sure that they
    // point towards origin.
    if (pcl_isfinite (graph[vertex].data_n[0]) &&
        pcl_isfinite (graph[vertex].data_n[1]) &&
        pcl_isfinite (graph[vertex].data_n[2]) &&
        pcl_isfinite (graph[vertex].data_n[3]) &&
        graph[vertex].getNormalVector4fMap ().any ())
    {
      if (graph[vertex].getNormalVector4fMap ().dot (normal) < 0)
        normal *= -1;
    }
    else
    {
      pcl::flipNormalTowardsViewpoint (graph[vertex], 0.0f, 0.0f, 0.0f, normal);
    }
    graph[vertex].getNormalVector4fMap () = normal;
    graph[vertex].curvature = std::isnan (curvature) ? 0.0f : curvature;
  }
}

template <typename Graph> void
pcl::graph::computeSignedCurvatures (Graph& graph)
{
  BOOST_CONCEPT_ASSERT ((pcl::graph::PointCloudGraphConcept<Graph>));

  typedef typename point_cloud_graph_traits<Graph>::point_type PointT;
  typedef typename Graph::edge_iterator EdgeIterator;
  typedef typename Graph::vertex_descriptor VertexId;

  EdgeIterator ei, ee;
  std::vector<float> convexities (boost::num_vertices (graph), 0.0f);
  for (boost::tie (ei, ee) = boost::edges (graph); ei != ee; ++ei)
  {
    VertexId v1 = boost::source (*ei, graph);
    VertexId v2 = boost::target (*ei, graph);
    const PointT& p1 = graph[v1];
    const PointT& p2 = graph[v2];
    const Eigen::Vector3f&  d = p2.getVector3fMap () - p1.getVector3fMap ();
    const Eigen::Vector3f& n1 = p1.getNormalVector3fMap ();
    const Eigen::Vector3f& n2 = p2.getNormalVector3fMap ();
    float c = (((d - d.dot (n1) * n1).dot (n2) > 0 ? 1.0 : -1.0) * (n1 - n2).squaredNorm ());
    convexities[v1] += c;
    convexities[v2] += c;
  }

  for (VertexId vertex = 0; vertex < boost::num_vertices (graph); ++vertex)
    graph[vertex].curvature = copysign (graph[vertex].curvature, convexities[vertex]);
}

template <typename Graph> size_t
pcl::graph::createSubgraphsFromConnectedComponents (Graph& graph,
                                                    std::vector<boost::reference_wrapper<Graph> >& subgraphs)
{
  typedef typename Graph::vertex_descriptor VertexId;
  typedef typename boost::reference_wrapper<Graph> GraphRef;

  subgraphs.clear ();
  std::vector<int> component (boost::num_vertices (graph));
  size_t num_components = boost::connected_components (graph, &component[0]);
  for (size_t i = 0; i < num_components; ++i)
    subgraphs.push_back (GraphRef (graph.create_subgraph ()));
  for (VertexId v = 0; v < boost::num_vertices (graph); ++v)
    boost::add_vertex (graph.local_to_global (v), subgraphs.at (component[v]).get ());
  return num_components;
}

template <typename Graph, typename ColorMap> size_t
pcl::graph::createSubgraphsFromColorMap (Graph& graph,
                                         ColorMap color_map,
                                         std::vector<boost::reference_wrapper<Graph> >& subgraphs)
{
  typedef typename Graph::vertex_descriptor VertexId;
  typedef typename boost::reference_wrapper<Graph> GraphRef;
  typedef typename boost::property_traits<ColorMap>::value_type Color;
  typedef std::map<Color, GraphRef> SubgraphMap;
  typedef typename SubgraphMap::iterator SubgraphMapIterator;

  SubgraphMap subgraph_map;
  for (VertexId v = 0; v < boost::num_vertices (graph); ++v)
  {
    SubgraphMapIterator itr = subgraph_map.find (color_map[v]);
    if (itr == subgraph_map.end ())
    {
      GraphRef subgraph (graph.create_subgraph ());
      boost::add_vertex (graph.local_to_global (v), subgraph.get ());
      subgraph_map.insert (std::make_pair (color_map[v], subgraph));
    }
    else
    {
      boost::add_vertex (graph.local_to_global (v), itr->second.get ());
    }
  }

  subgraphs.clear ();
  for (SubgraphMapIterator itr = subgraph_map.begin (); itr != subgraph_map.end (); ++itr)
    subgraphs.push_back (itr->second);

  return subgraphs.size ();
}

template <typename Graph> void
pcl::graph::createSubgraphsFromIndices (Graph& graph,
                                        const pcl::PointIndices& indices,
                                        std::vector<boost::reference_wrapper<Graph> >& subgraphs)
{
  typedef typename Graph::vertex_descriptor VertexId;
  typedef typename boost::reference_wrapper<Graph> GraphRef;

  std::set<int> index_set;

  subgraphs.clear ();
  subgraphs.push_back (GraphRef (graph.create_subgraph ()));
  subgraphs.push_back (GraphRef (graph.create_subgraph ()));
  Graph& first = subgraphs.at (0).get ();
  Graph& second = subgraphs.at (1).get ();

  for (size_t i = 0; i < indices.indices.size (); ++i)
  {
    index_set.insert (indices.indices[i]);
    boost::add_vertex (indices.indices[i], first);
  }

  for (VertexId v = 0; v < boost::num_vertices (graph); ++v)
    if (!index_set.count (v))
      boost::add_vertex (v, second);
}

template <typename Graph> void
pcl::graph::createSubgraphsFromIndices (Graph& graph,
                                        const std::vector<pcl::PointIndices>& indices,
                                        std::vector<boost::reference_wrapper<Graph> >& subgraphs)
{
  typedef typename Graph::vertex_descriptor VertexId;
  typedef typename boost::reference_wrapper<Graph> GraphRef;

  std::set<int> index_set;

  subgraphs.clear ();
  for (size_t i = 0; i < indices.size (); ++i)
  {
    subgraphs.push_back (GraphRef (graph.create_subgraph ()));
    Graph& s = subgraphs.back ().get ();
    for (size_t j = 0; j < indices[i].indices.size (); ++j)
    {
      index_set.insert (indices[i].indices[j]);
      boost::add_vertex (indices[i].indices[j], s);
    }
  }

  subgraphs.push_back (GraphRef (graph.create_subgraph ()));
  Graph& s = subgraphs.back ().get ();
  for (VertexId v = 0; v < boost::num_vertices (graph); ++v)
    if (!index_set.count (v))
      boost::add_vertex (v, s);
}

template <typename Graph> void
pcl::graph::smoothen (Graph& graph, float spatial_sigma, float influence_sigma)
{
  BOOST_CONCEPT_ASSERT ((pcl::graph::PointCloudGraphConcept<Graph>));

  typedef typename Graph::edge_iterator EdgeIterator;
  typedef typename Graph::vertex_descriptor VertexId;

  std::vector<float> K (boost::num_vertices (graph), 0);
  Eigen::MatrixXf P (boost::num_vertices (graph), 3);
  P.setZero ();

  EdgeIterator ei, ee;
  for (boost::tie (ei, ee) = boost::edges (graph); ei != ee; ++ei)
  {
    const VertexId& src = boost::source (*ei, graph);
    const VertexId& tgt = boost::target (*ei, graph);
    const Eigen::Vector3f& p = graph[src].getVector3fMap ();
    const Eigen::Vector3f& q = graph[tgt].getVector3fMap ();
    const Eigen::Vector3f& np = graph[src].getNormalVector3fMap ();
    const Eigen::Vector3f& nq = graph[tgt].getNormalVector3fMap ();
    Eigen::Vector3f x = p - q;
    float d1 = nq.dot (x);
    float d2 = np.dot (-x);
    float ws = std::exp (- std::pow (x.norm (), 2) / (2 * std::pow (spatial_sigma, 2)));
    float wi1 = std::exp (- std::pow (d1, 2) / (2 * std::pow (influence_sigma, 2)));
    float wi2 = std::exp (- std::pow (d2, 2) / (2 * std::pow (influence_sigma, 2)));
    float w1 = ws * wi1;
    float w2 = ws * wi2;
    K[src] += w1;
    K[tgt] += w2;
    P.row (src) += nq * d1 * w1;
    P.row (tgt) += np * d2 * w2;
  }

  for (VertexId v = 0; v < boost::num_vertices (graph); ++v)
    if (K[v] > 0.01)
      graph[v].getVector3fMap () -= P.row (v) / K[v];
}

#endif /* PCL_GRAPH_IMPL_COMMON_HPP */

