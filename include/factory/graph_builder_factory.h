#ifndef FACTORY_GRAPH_BUILDER_FACTORY_H
#define FACTORY_GRAPH_BUILDER_FACTORY_H

#include "factory.h"
#include "graph/graph_builder.h"
#include "graph/nearest_neighbors_graph_builder.h"
#include "graph/voxel_grid_graph_builder.h"

namespace factory
{

template <typename PointT, class GraphT>
class GraphBuilderFactory : public Factory
{

public:

  typedef pcl::graph::GraphBuilder<PointT, GraphT> GraphBuilderT;

  GraphBuilderFactory ()
  : Factory ("Graph Builder")
  , builder_ ("builder type", "--builder", { { "vg",  "VOXEL GRID"              }
                                           , { "nnk", "NEAREST NEIGHBORS KNN"   }
                                           , { "nnr", "NEAREST NEIGHBORS RADIUS"} })
  , voxel_resolution_ ("voxel resolution", "-v", 0.006f)
  , number_of_neighbors_ ("number of neighbors", "--nn", 14)
  , radius_ ("sphere radius", "--radius", 0.006f)
  , no_transform_ ("no transform", "-nt")
  {
    add (&builder_);
    add (&voxel_resolution_);
    add (&number_of_neighbors_);
    add (&radius_);
    add (&no_transform_);
  }

  typename GraphBuilderT::Ptr
  instantiate (int argc, char** argv)
  {
    parse (argc, argv);
    typename GraphBuilderT::Ptr gb;
    if (builder_.value == "vg")
    {
      gb.reset (new pcl::graph::VoxelGridGraphBuilder<PointT, GraphT> (voxel_resolution_));
    }
    else if (builder_.value == "nnk")
    {
      auto nngb = new pcl::graph::NearestNeighborsGraphBuilder<PointT, GraphT>;
      nngb->setNumberOfNeighbors (number_of_neighbors_);
      nngb->useNearestKSearch ();
      gb.reset (nngb);
    }
    else
    {
      auto nngb = new pcl::graph::NearestNeighborsGraphBuilder<PointT, GraphT>;
      nngb->setNumberOfNeighbors (number_of_neighbors_);
      nngb->setRadius (radius_);
      nngb->useRadiusSearch ();
      gb.reset (nngb);
    }
    return gb;
  }

private:

  EnumOption builder_;
  NumericOption<float> voxel_resolution_;
  NumericOption<int> number_of_neighbors_;
  NumericOption<float> radius_;
  BoolOption no_transform_;

};

}

#endif /* FACTORY_GRAPH_BUILDER_FACTORY_H */

