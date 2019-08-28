#ifndef FACTORY_EDGE_WEIGHT_COMPUTER_FACTORY_H
#define FACTORY_EDGE_WEIGHT_COMPUTER_FACTORY_H

#include "factory.h"
#include "graph/edge_weight_computer.h"

namespace factory
{

template <typename GraphT>
class EdgeWeightComputerFactory : public Factory
{

public:

  typedef pcl::graph::EdgeWeightComputer<GraphT> EdgeWeightComputer;

  EdgeWeightComputerFactory ()
  : Factory ("Edge Weight Computer")
  , xyz_influence_ ("xyz influence", "-z", 3.0f)
  , normal_influence_ ("normal influence", "-n", 0.01f)
  , curvature_influence_ ("curvature influence", "-b", 0.0001f)
  , color_influence_ ("color influence", "-c", 3.0f)
  , verticality_influence_ ("verticality influence", "-V", 1.0f)
  , weight_threshold_ ("weight threshold", "--weight-threshold", 0.00001f)
  , convex_multiplier_ ("convex multiplier", "--convex-multiplier", 0.01f)
  {
    add (&xyz_influence_);
    add (&normal_influence_);
    add (&curvature_influence_);
    add (&color_influence_);
    add (&verticality_influence_);
    add (&weight_threshold_);
    add (&convex_multiplier_);
  }

  typename EdgeWeightComputer::Ptr
  instantiate (int argc, char** argv)
  {
    using namespace pcl::graph::terms;
    parse (argc, argv);
    typename EdgeWeightComputer::Ptr ewc (new EdgeWeightComputer);
    ewc->template addTerm<XYZ> (xyz_influence_, EdgeWeightComputer::NORMALIZATION_LOCAL);
    ewc->template addTerm<Normal> (normal_influence_, convex_multiplier_);
    ewc->template addTerm<Curvature> (curvature_influence_, convex_multiplier_);
    ewc->template addTerm<RGB> (color_influence_, EdgeWeightComputer::NORMALIZATION_GLOBAL);
    ewc->template addTerm<Verticality> (verticality_influence_);
    ewc->setSmallWeightThreshold (weight_threshold_);
    ewc->setSmallWeightPolicy (EdgeWeightComputer::SMALL_WEIGHT_COERCE_TO_THRESHOLD);
    return ewc;
  }

private:

  NumericOption<float> xyz_influence_;
  NumericOption<float> normal_influence_;
  NumericOption<float> curvature_influence_;
  NumericOption<float> color_influence_;
  NumericOption<float> verticality_influence_;
  NumericOption<float> weight_threshold_;
  NumericOption<float> convex_multiplier_;

};

}

#endif /* FACTORY_EDGE_WEIGHT_COMPUTER_FACTORY_H */

