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

#ifndef PCL_GRAPH_VOXEL_GRID_GRAPH_BUILDER_H
#define PCL_GRAPH_VOXEL_GRID_GRAPH_BUILDER_H

#include "graph/graph_builder.h"

namespace pcl
{

  namespace graph
  {

    /** This class builds a BGL graph representing an input dataset by using
      * octree::OctreePointCloud.
      *
      * For additional information see documentation for \ref GraphBuilder.
      *
      * \author Sergey Alexandrov
      * \ingroup graph */
    template <typename PointT, typename GraphT>
    class PCL_EXPORTS VoxelGridGraphBuilder : public GraphBuilder<PointT, GraphT>
    {

        using PCLBase<PointT>::initCompute;
        using PCLBase<PointT>::deinitCompute;
        using PCLBase<PointT>::indices_;
        using PCLBase<PointT>::input_;
        using GraphBuilder<PointT, GraphT>::point_to_vertex_map_;

      public:

        using typename GraphBuilder<PointT, GraphT>::PointInT;
        using typename GraphBuilder<PointT, GraphT>::PointOutT;
        using typename GraphBuilder<PointT, GraphT>::VertexId;

        /** Constructor.
          *
          * \param[in] voxel_resolution resolution of the voxel grid */
        VoxelGridGraphBuilder (float voxel_resolution)
        : voxel_resolution_ (voxel_resolution)
        {
        }

        virtual void
        compute (GraphT& graph);

        inline void
        setVoxelResolution (float resolution)
        {
          voxel_resolution_ = resolution;
        }

        inline float
        getVoxelResolution () const
        {
          return (voxel_resolution_);
        }

      private:

        /// Resolution of the voxel grid.
        float voxel_resolution_;

    };

  }

}

#include "graph/impl/voxel_grid_graph_builder.hpp"

#endif /* PCL_GRAPH_VOXEL_GRID_GRAPH_BUILDER_H */

