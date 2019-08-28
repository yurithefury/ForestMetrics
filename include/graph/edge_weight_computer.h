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

#ifndef PCL_GRAPH_EDGE_WEIGHT_COMPUTER_H
#define PCL_GRAPH_EDGE_WEIGHT_COMPUTER_H

#include <boost/function.hpp>

#include "graph/point_cloud_graph.h"
#include "graph/point_cloud_graph_concept.h"

#include "graph/edge_weight_computer_terms.h"

namespace pcl
{

  namespace graph
  {

    /** This class computes edge weights for a given point cloud graph.
      *
      * The compute() function iterates over graph edges and calculates their
      * weights based on the data contained in their end vertices. The general
      * form of the weighting function used in calculation is fixed, however the
      * user is provided with a means to customize it.
      *
      * For a pair of vertices \f$v_i\f$ and \f$v_j\f$ connected by an edge, the
      * weight is computed as a product of \f$k\f$ independent *terms*:
      *
      * \f[
      *     w_{ij} =
      *       Term_1\left(v_i,v_j\right) \cdot
      *       \dots                      \cdot
      *       Term_k\left(v_i,v_j\right)
      * \f]
      *
      * Each term has the following form:
      *
      * \f[
      *     Term\left(v_i,v_j\right) =
      *       \phi\left(\frac{d\left(v_i,v_j\right)}
      *                      {\bar{d}\left(v_i,v_j\right)}, \sigma\right)
      * \f]
      *
      * \f$\phi\left(\cdot,\cdot\right)\f$ is a *balancing function*. Its first
      * argument is a value calculated from the vertex data, and the second
      * argument controls the *influence* of the term. An example of a balancing
      * function is Gaussian:
      *
      * \f[
      *     \phi\left(x,\sigma\right) =
      *       \exp{\left\{-\frac{x}{\sigma}\right\}}
      * \f]
      *
      * The value that is fed to the balancing function is calculated based on
      * the data in the vertices; it consists of a *distance* between vertices
      * \f$d\left(v_i,v_j\right)\f$ (in numerator) and a *normalization* term
      * \f$\bar{d}\left(v_i,v_j\right)\f$ (in denominator). The former is
      * called "distance" because typically it is indeed a distance function,
      * such as Euclidean distance between 3D points, or angular distance
      * between point normals. The normalization term is needed to account for
      * the cases when the distance has a significant global variation over the
      * graph.
      *
      *
      * Predefined terms
      * ----------------
      *
      * A few commonly used terms are predefined; they are presented below using
      * the following notation. Given a point cloud graph vertex \f$v_i\f$,
      * \f$p_i\f$ denotes the 3D coordinates of the associated point, \f$n_i\f$
      * denotes the normal orientation of the associated point, \f${rgb}_i\f$ is a
      * vector consisting of R, G, and B components of the color of the
      * associated point, and \f$c_i\f$ denotes the curvature of the associated
      * point.
      *
      * - \ref terms::XYZ "XYZ" term (Euclidean distance between points)
      *
      *   \f$d_{xyz}(v_i,v_j) = ||p_i-p_j||^2\f$
      *
      * - \ref terms::Normal "Normal" term (angular distance between normals)
      *
      *   \f$d_{normal}(v_i,v_j) = \frac{||n_i-n_j||^2}{2}\f$
      *
      * - \ref terms::Curvature "Curvature" term (product of point curvatures)
      *
      *   \f$d_{curvature}(v_i,v_j) = c_i \cdot c_j\f$
      *
      * - \ref terms::RGB "RGB" term (Euclidean distance in RGB space)
      *
      *   \f$d_{xyz}(v_i,v_j) = ||rgb_i-rgb_j||^2\f$
      *
      *
      * Normalization
      * -------------
      *
      * Three types of normalization (defined by the \ref NormalizationType enum)
      * are available:
      *
      * - \ref NORMALIZATION_NONE "No normalization"
      *
      *   No normalization, the term is left as is.
      *
      * - \ref NORMALIZATION_GLOBAL "Global normalization"
      *
      *   Normalize term by the average value of the corresponding distance over
      *   all edges in the graph.
      *
      * - \ref NORMALIZATION_LOCAL "Local normalization"
      *
      *   Normalize term by the average value of the corresponding distance over
      *   all edges incident to the end points of the edge.
      *
      *
      * Small weight policies
      * ---------------------
      *
      * After the edge weights were computed, there is an optional step where
      * the weights or the edge set of the graph may be modified to ensure that
      * there are no edges with weights that are "too small". This is useful
      * e.g. if the further processing involves solving linear systems based on
      * adjacency matrix of the graph. Leaving smallish weights might lead to
      * numerical problems in that case.
      *
      * The threshold is controlled by the setSmallWeightThreshold() function.
      * The policies are defined by the \ref SmallWeightPolicy enum and may be
      * set using setSmallWeightPolicy() function.
      *
      *
      * Usage
      * -----
      *
      * The following code snippet demonstrates a typical usage of the
      * EdgeWeightComputer class:
      *
      * ~~~{.cpp}
      * using namespace pcl::graph;
      *
      * // Typedef a point cloud graph with internal edge weight map
      * typedef point_cloud_graph<pcl::PointXYZRGB,
      *                           boost::vecS,
      *                           boost::undirectedS,
      *                           boost::property<boost::edge_weight_t, float>
      *                           boost::listS> Graph;
      * // Create a graph
      * Graph graph;
      * // Add vertices and edges
      * // ...
      *
      * // Create edge weight computer
      * EdgeWeightComputer<Graph> computer;
      * // Add XYZ term with 1.0 influence
      * computer.addTerm<terms::XYZ> (1.0f);
      * // Add Normal term with 2.0 influence and a discounting multiplier
      * // for convex edges
      * computer.addTerm<terms::Normal> (1.0f, 2.0f);
      * // Add RGB term with 1.0 influence and local normalization
      * computer.addTerm<terms::RGB> (1.0f, EdgeWeightComputer<Graph>::NORMALIZATION_LOCAL);
      * // Compute edge weights;
      * computer.compute (graph);
      * ~~~
      *
      * \author Sergey Alexandrov
      * \ingroup graph */
    template <typename GraphT>
    class PCL_EXPORTS EdgeWeightComputer
    {

        BOOST_CONCEPT_ASSERT ((pcl::graph::PointCloudGraphConcept<GraphT>));

      public:

        typedef typename pcl::graph::point_cloud_graph_traits<GraphT>::point_type PointT;
        typedef boost::function<float (float, float)> TermBalancingFunction;
        typedef boost::shared_ptr<EdgeWeightComputer> Ptr;

        /** Different normalization types that could be applied to a term. */
        enum NormalizationType
        {
          /// No normalization
          NORMALIZATION_NONE,
          /// Global normalization
          NORMALIZATION_GLOBAL,
          /// Local normalization
          NORMALIZATION_LOCAL
        };

        /** Policy which controls what happens to edges with small weights
          * (i.e. below user-specified threshold). */
        enum SmallWeightPolicy
        {
          /// Do nothing, leave the weights and edges as is.
          SMALL_WEIGHT_IGNORE,
          /// Coerce the weight to the threshold. This guarantees that there
          /// are no edges in the graph with weight below threshold, and at the
          /// same time does not modify the edge set of the graph.
          SMALL_WEIGHT_COERCE_TO_THRESHOLD,
          /// Remove edges with weights below threshold. This guarantees that
          /// there are no edges in the graph with weight below threshold, but
          /// may modify the edge set of the graph.
          SMALL_WEIGHT_REMOVE_EDGE,
        };

        /** Construct a weight computer with default settings.
          *
          * By default the small weight threshold is set to zero, and the edges
          * with small weights are ignored. The default term balancing function
          * is Gaussian. */
        EdgeWeightComputer ()
        : policy_ (SMALL_WEIGHT_IGNORE)
        , threshold_ (0.0f)
        , balancing_function_ (&EdgeWeightComputer<GraphT>::gaussian)
        {
        }

        /** Compute weights for the edges in a given graph.
          *
          * This function iterates over graph edges and calculates their
          * weights based on the data contained in their end vertices. See
          * class documentation for more information.
          *
          * This version stores computed weights in *external* property map.
          *
          * \param[in]      graph a point cloud graph
          * \param[in]      weights an external edge weight property map */
        template <typename EdgeWeightMap> void
        compute (GraphT& graph, EdgeWeightMap weights);

        /** Compute weights for the edges in a given graph.
          *
          * This function iterates over graph edges and calculates their
          * weights based on the data contained in their end vertices. See
          * class documentation for more information.
          *
          * This version stores computed weights in *internal* property map.
          *
          * \param[in]      graph a point cloud graph */
        inline void
        compute (GraphT& graph)
        {
          compute (graph, boost::get (boost::edge_weight, graph));
        }

        /** Add a term to the edge weighting function.
          *
          * \param[in]      influence \f$\sigma\f$ that will be passed to the
          *                 balancing function
          * \param[in]      convex_influence_multiplier an influence multiplier
          *                 that will be applied if the edge is convex
          *                 (default: 1.0, i.e. no difference between concave
          *                 and convex edges).
          * \param[in]      normalization normalization type for the term
          *                 (default: NORMALIZATION_NONE, i.e. no
          *                 normalization). */
        template <typename TermT> void
        addTerm (float influence,
                 float convex_influence_multiplier = 1.0,
                 NormalizationType normalization = NORMALIZATION_NONE)
        {
          addTermImpl<TermT> (influence, convex_influence_multiplier, normalization,
                              typename boost::mpl::apply<typename TermT::is_compatible, PointT>::type ());
        }

        /** Add a term to the edge weighting function.
          *
          * This is an overloaded function provided for convenience. See the
          * documentation for addTerm(). */
        template <typename TermT> void
        addTerm (float influence,
                 NormalizationType normalization)
        {
          addTermImpl<TermT> (influence, 1.0, normalization,
                              typename boost::mpl::apply<typename TermT::is_compatible, PointT>::type ());
        }

        /** Set the policy for edges with small (below threshold) weights. */
        inline void
        setSmallWeightPolicy (SmallWeightPolicy policy)
        {
          policy_ = policy;
        }

        /** Set the threshold for edge weights. */
        inline void
        setSmallWeightThreshold (float threshold)
        {
          threshold_ = threshold;
        }

        /** Set the function used to balance the contributions of the terms. */
        inline void
        setTermBalancingFunction (TermBalancingFunction func)
        {
          balancing_function_ = func;
        }

      private:

        /** Gaussian, used as a balancing function by default. */
        static float
        gaussian (float val, float influence)
        {
          return (influence > 0.0 ? std::exp (-val / influence) : 1.0);
        };

        /** Internal helper structure used to represent a term in the weighting
          * function, implemented by the edge weight computer. */
        struct Term
        {

          typedef boost::function<float (const PointT&, const PointT&)> ComputeFunction;

          ComputeFunction compute_;
          float influence_;
          float convex_influence_multiplier_;

          Term (ComputeFunction f, float i, float c)
          : compute_ (f)
          , influence_ (i)
          , convex_influence_multiplier_ (c)
          {
          }

          inline float
          getInfluence (bool is_convex = false) const
          {
            return (influence_ * (is_convex ? convex_influence_multiplier_ : 1.0));
          }

        };

        /** Internal helper structure used to represent a globally normalized
          * term in the weighting function, implemented by the edge weight
          * computer. */
        struct GloballyNormalizedTerm : Term
        {

          GloballyNormalizedTerm (typename Term::ComputeFunction f, float i, float c)
          : Term (f, i, c)
          {
          }

          void
          init (size_t num_edges)
          {
            edge_weights_.resize (num_edges, 0.0f);
            total_weight_ = 0.0f;
          }

          float
          round1 (const PointT& p1, const PointT& p2, size_t edge_id)
          {
            float weight = this->compute_ (p1, p2);
            edge_weights_[edge_id] = weight;
            total_weight_ += weight;
            return (weight);
          }

          void
          extract ()
          {
            average_ = total_weight_ / edge_weights_.size ();
          }

          float
          round2 (size_t edge_id)
          {
            return (edge_weights_[edge_id] / average_);
          }

          std::vector<float> edge_weights_;
          float total_weight_;
          float average_;

        };

        /** Internal helper structure used to represent a locally normalized
          * term in the weighting function, implemented by the edge weight
          * computer. */
        struct LocallyNormalizedTerm : Term
        {

          LocallyNormalizedTerm (typename Term::ComputeFunction f, float i, float c)
          : Term (f, i, c)
          {
          }

          void
          init (size_t num_edges, size_t num_vertices)
          {
            edge_weights_.resize (num_edges, 0.0f);
            vertex_sums_.resize (num_vertices, 0.0f);
            vertex_degrees_.resize (num_vertices, 0);
          }

          float
          round1 (const PointT& p1,
                  const PointT& p2,
                  size_t vertex1_id,
                  size_t vertex2_id,
                  size_t edge_id)
          {
            float weight = this->compute_ (p1, p2);
            edge_weights_[edge_id] = weight;
            vertex_sums_[vertex1_id] += weight;
            vertex_sums_[vertex2_id] += weight;
            ++vertex_degrees_[vertex1_id];
            ++vertex_degrees_[vertex2_id];
            return (weight);
          }

          void
          extract ()
          {
            for (size_t i = 0; i < vertex_sums_.size (); ++i)
              if (vertex_degrees_[i])
                vertex_sums_[i] /= vertex_degrees_[i];
          }

          float
          round2 (size_t vertex1_id,
                  size_t vertex2_id,
                  size_t edge_id) const
          {
            float n = (vertex_sums_[vertex1_id] + vertex_sums_[vertex2_id]) / 2.0;
            float weight = (n > 0.0f && this->getInfluence () > 0.0f)
                         ? edge_weights_[edge_id] / n
                         : 0.0f;
            return (weight);
          }

          std::vector<float> edge_weights_;
          std::vector<float> vertex_sums_;
          std::vector<size_t> vertex_degrees_;

        };

        template <typename TermT> void
        addTermImpl (float influence,
                     float convex_influence_multiplier,
                     NormalizationType normalization,
                     boost::mpl::bool_<true>)
        {
          switch (normalization)
          {
            case NORMALIZATION_NONE:
              {
                terms_.push_back (Term (TermT::template compute<PointT>, influence, convex_influence_multiplier));
                break;
              }
            case NORMALIZATION_GLOBAL:
              {
                g_terms_.push_back (GloballyNormalizedTerm (TermT::template compute<PointT>, influence, convex_influence_multiplier));
                break;
              }
            case NORMALIZATION_LOCAL:
              {
                l_terms_.push_back (LocallyNormalizedTerm (TermT::template compute<PointT>, influence, convex_influence_multiplier));
                break;
              }
          }
        }

        /** No-op implementation of addTerm(), instantiated for terms that are
          * not compatible with the graph point type. */
        template <typename TermT> void
        addTermImpl (float influence,
                     float convex_influence_multiplier,
                     NormalizationType normalization,
                     boost::mpl::bool_<false>)
        {
        }

        std::vector<Term> terms_;
        std::vector<GloballyNormalizedTerm> g_terms_;
        std::vector<LocallyNormalizedTerm> l_terms_;

        SmallWeightPolicy policy_;
        float threshold_;
        TermBalancingFunction balancing_function_;

    };

  }

}

#include "graph/impl/edge_weight_computer.hpp"

#endif /* PCL_GRAPH_EDGE_WEIGHT_COMPUTER_H */

