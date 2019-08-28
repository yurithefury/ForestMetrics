#ifndef FACTORY_GRAPH_FACTORY_H
#define FACTORY_GRAPH_FACTORY_H

#include <memory>

#include "factory.h"
#include "graph_builder_factory.h"
#include "edge_weight_computer_factory.h"

#include "graph/common.h"

#include "measure_runtime.h"

namespace factory
{

template <typename Point, typename Graph>
class GraphFactory : public Factory
{

public:

  typedef boost::shared_ptr<Graph> GraphPtr;
  typedef boost::reference_wrapper<Graph> GraphRef;
  typedef std::vector<GraphRef> GraphRefVector;
  typedef typename pcl::PointCloud<Point>::ConstPtr PointCloudConstPtr;
  typedef typename pcl::graph::GraphBuilder<Point, Graph>::Ptr GraphBuilderPtr;

  GraphFactory ()
  : Factory ("Graph")
  , component_ ("connected component", "--component", -1)
  , smoothing_spatial_ ("smoothing spatial", "--smoothing-spatial", 0.012)
  , smoothing_influence_ ("smoothing influence", "--smoothing-influence", 0.0012)
  {
    add (&component_);
    add (&smoothing_spatial_);
    add (&smoothing_influence_);
  }

  virtual const std::string
  getUsage ()
  {
    std::stringstream usage;
    usage << Factory::getUsage () << std::endl;
    usage << gb_factory_.getUsage () << std::endl;
    usage << wc_factory_.getUsage ();
    return usage.str ();
  }

  virtual void
  printValues ()
  {
    Factory::printValues ();
    gb_factory_.printValues ();
    wc_factory_.printValues ();
    if (produced_graph_)
    {
      pcl::console::print_info ("Total graph size: %zu / %zu\n",
                                boost::num_vertices (*produced_graph_),
                                boost::num_edges (*produced_graph_));
    }
  }

  GraphRef
  instantiate (const PointCloudConstPtr& cloud, int argc, char** argv)
  {
    parse (argc, argv);
    gb_ = gb_factory_.instantiate (argc, argv);
    auto wc = wc_factory_.instantiate (argc, argv);
    // Build graph
    produced_graph_.reset (new Graph);
    gb_->setInputCloud (cloud);
    MEASURE_RUNTIME ("Building graph... ",
                     gb_->compute (*produced_graph_));
    MEASURE_RUNTIME ("Computing normals... ",
                     pcl::graph::computeNormalsAndCurvatures (*produced_graph_));
    MEASURE_RUNTIME ("Smoothening graph... ",
                     pcl::graph::smoothen (*produced_graph_, smoothing_spatial_, smoothing_influence_));
    MEASURE_RUNTIME ("Re-computing normals... ",
                     pcl::graph::computeNormalsAndCurvatures (*produced_graph_));
    MEASURE_RUNTIME ("Computing curvature signs... ",
                     pcl::graph::computeSignedCurvatures (*produced_graph_));
    MEASURE_RUNTIME ("Computing edge weights... ",
                     wc->compute (*produced_graph_));
    MEASURE_RUNTIME ("Computing connected components... ",
                     pcl::graph::createSubgraphsFromConnectedComponents (*produced_graph_, components_));
    if (component_ != -1 && component_ < static_cast<int> (components_.size ()))
      return components_[component_];
    return GraphRef (*produced_graph_);
  }

  GraphPtr
  getProducedGraph ()
  {
    return produced_graph_;
  }

  GraphRefVector&
  getProducedGraphComponents ()
  {
    return components_;
  }

  GraphBuilderPtr
  getGraphBuilder ()
  {
    return gb_;
  }

private:

  NumericOption<int> component_;
  NumericOption<float> smoothing_spatial_;
  NumericOption<float> smoothing_influence_;

  EdgeWeightComputerFactory<Graph> wc_factory_;
  GraphBuilderFactory<Point, Graph> gb_factory_;

  GraphBuilderPtr gb_;

  GraphPtr produced_graph_;
  GraphRefVector components_;

};

}

#endif /* FACTORY_GRAPH_FACTORY_H */

