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

#ifndef PCL_GRAPH_NEAREST_NEIGHBORS_GRAPH_BUILDER_H
#define PCL_GRAPH_NEAREST_NEIGHBORS_GRAPH_BUILDER_H

#include <pcl/search/search.h>

#include "graph/graph_builder.h"

namespace pcl
{

  namespace graph
  {

    /** This class builds a point cloud graph representing an input dataset by
      * using nearest neighbor search.
      *
      * The points from the input cloud become vertices and the edges are
      * established between each point and its neighbors (as found by the search
      * object provided with setSearchMethod()). The user may choose to use the
      * default search method, which will be either KdTree or OrganizedNeighbor
      * depending on whether the input point cloud is organized or not.
      *
      * The data contained in the points of the input cloud will be copied
      * inside the vertices of the newly created graph. Note that the points in
      * the input cloud and the output graph may have different types, in which
      * case only the intersecting fields will be copied over.
      *
      * For additional information see documentation for \ref GraphBuilder.
      *
      * \author Sergey Alexandrov
      * \ingroup graph */
    template <typename PointT, typename GraphT>
    class PCL_EXPORTS NearestNeighborsGraphBuilder : public GraphBuilder<PointT, GraphT>
    {

        using PCLBase<PointT>::initCompute;
        using PCLBase<PointT>::deinitCompute;
        using PCLBase<PointT>::indices_;
        using PCLBase<PointT>::input_;
        using PCLBase<PointT>::use_indices_;
        using PCLBase<PointT>::fake_indices_;
        using GraphBuilder<PointT, GraphT>::point_to_vertex_map_;

      private:

        /// Search types that can be used to find nearest neighbors.
        enum SearchType { KNN, RADIUS };

      public:

        using typename GraphBuilder<PointT, GraphT>::PointInT;
        using typename GraphBuilder<PointT, GraphT>::PointOutT;
        using typename GraphBuilder<PointT, GraphT>::VertexId;

        typedef pcl::search::Search<PointOutT> Search;
        typedef typename Search::Ptr SearchPtr;

        /** Constructor.
          *
          * By default kNN search with 10 neighbors will be used. Use functions
          * setNumberOfNeighbors() or setRadius() to change search type and/or
          * adjust its parameters. */
        NearestNeighborsGraphBuilder ()
        : num_neighbors_ (10)
        , search_type_ (KNN)
        {
        }

        virtual void
        compute (GraphT& graph);

        /** Set search method that will be used for finding nearest neighbors
          * when building a graph. */
        inline void
        setSearchMethod (const SearchPtr& search)
        {
          search_ = search;
        }

        /** Get the search method used for finding nearest neighbors when
          * building a graph. */
        inline SearchPtr
        getSearchMethod () const
        {
          return (search_);
        }

        /** Use k-nearest neighbors search to establish edge set of a graph. */
        inline void
        useNearestKSearch ()
        {
          search_type_ = KNN;
        }

        /** Use radius search to establish edge set of a graph. */
        inline void
        useRadiusSearch ()
        {
          search_type_ = RADIUS;
        }

        /** Set the (maximum) number of neighbors to find when building a
          * graph. */
        inline void
        setNumberOfNeighbors (size_t num_neighbors)
        {
          num_neighbors_ = num_neighbors;
        }

        /** Set the sphere radius when building a graph with radius search
          * type. */
        inline void
        setRadius (double radius)
        {
          radius_ = radius;
        }

      private:

        /// The search method that will be used for finding K nearest neighbors
        /// when building a graph.
        SearchPtr search_;

        /// The number of neighbors to find when building a graph.
        size_t num_neighbors_;

        /// Sphere radius for radius search.
        double radius_;

        /// Search type (knn or radius).
        SearchType search_type_;

    };

  }

}

#include "graph/impl/nearest_neighbors_graph_builder.hpp"

#endif /* PCL_GRAPH_NEAREST_NEIGHBORS_GRAPH_BUILDER_H */

