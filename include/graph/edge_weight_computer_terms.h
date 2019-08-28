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

#ifndef PCL_GRAPH_EDGE_WEIGHT_COMPUTER_TERMS_H
#define PCL_GRAPH_EDGE_WEIGHT_COMPUTER_TERMS_H

#include <pcl/point_types.h>

namespace pcl
{

  namespace graph
  {

    namespace terms
    {

      /** Squared Euclidean distance between points.
        *
        * \f[
        *     d_{xyz}(v_i,v_j) = ||p_i-p_j||^2
        * \f]
        *
        * Requires that the point type has *x*, *y*, and *z* fields.
        *
        * \ingroup graph
        * \author Sergey Alexandrov
        */
      struct XYZ
      {
        typedef pcl::traits::has_xyz<boost::mpl::_1> is_compatible;

        template <typename PointT> float
        static compute (const PointT& p1, const PointT& p2)
        {
          return (p2.getVector3fMap () - p1.getVector3fMap ()).squaredNorm ();
        }
      };

      /** Angular distance between normals.
        *
        * \f[
        *     d_{normal}(v_i,v_j) = \frac{||n_i-n_j||^2}{2}
        * \f]
        *
        * Requires that the point type has *normal_x*, *normal_y*, and
        * *normal_z* fields.
        *
        * \ingroup graph
        * \author Sergey Alexandrov
        */
      struct Normal
      {
        typedef pcl::traits::has_normal<boost::mpl::_1> is_compatible;

        template <typename PointT> float
        static compute (const PointT& p1, const PointT& p2)
        {
          return (0.5 * (p1.getNormalVector3fMap () - p2.getNormalVector3fMap ()).squaredNorm ());
        }
      };

      /** Product of curvatures.
        *
        * \f[
        *     d_{curvature}(v_i,v_j) = c_i \cdot c_j
        * \f]
        *
        * Requires that the point type has *curvature* field.
        *
        * \ingroup graph
        * \author Sergey Alexandrov
        */
      struct Curvature
      {
        typedef pcl::traits::has_curvature<boost::mpl::_1> is_compatible;

        template <typename PointT> float
        static compute (const PointT& p1, const PointT& p2)
        {
          return (std::fabs (p1.curvature) * std::fabs (p2.curvature));
        }
      };

      /** Squared Euclidean distance in RGB space.
        *
        * \f[
        *     d_{xyz}(v_i,v_j) = ||rgb_i-rgb_j||^2
        * \f]
        *
        * Requires that the point type has *rgb* or *rgba* field.
        *
        * \ingroup graph
        * \author Sergey Alexandrov
        */
      struct RGB
      {
        typedef pcl::traits::has_color<boost::mpl::_1> is_compatible;

        template <typename PointT> float
        static compute (const PointT& p1, const PointT& p2)
        {
          return ((p1.getBGRVector3cMap ().template cast<float> () -
                   p2.getBGRVector3cMap ().template cast<float> ()).norm () / 255.0f);
        }
      };

      struct Verticality
      {
        typedef pcl::traits::has_xyz<boost::mpl::_1> is_compatible;

        template <typename PointT> float
        static compute (const PointT& p1, const PointT& p2)
        {
          return (1.0f - std::abs((p2.getVector3fMap () - p1.getVector3fMap ()).normalized()[2]));
        }
      };

    }

  }

}

#endif /* PCL_GRAPH_EDGE_WEIGHT_COMPUTER_TERMS_H */

