#include <vtkLine.h>
#include <vtkPolyLine.h>
#include <vtkPolyData.h>
#include <vtkCellData.h>
#include <vtkCellArray.h>
#include <vtkSmartPointer.h>

#include <pcl/common/colors.h>

#include "tviewer/tviewer.h"
#include "tviewer/tviewer_widget/tviewer_widget.h"

#include "config.h"
#include "status_bar.h"
#include "graph_building_form.h"
#include "ui_graph_building_form.h"

#include "graph/common.h"
#include "graph/edge_weight_computer.h"
#include "graph/voxel_grid_graph_builder.h"
#include "graph/nearest_neighbors_graph_builder.h"

using namespace tviewer;

GraphBuildingForm::GraphBuildingForm (QWidget* parent)
: QWidget (parent)
, ui_ (new Ui::GraphBuildingForm)
, graph_ (new Graph)
, vertices_ (new PointCloud)
, edges_ (vtkSmartPointer<vtkPolyData>::New ())
{
  ui_->setupUi (this);
}

GraphBuildingForm::~GraphBuildingForm ()
{
  delete ui_;
}

void
GraphBuildingForm::enter ()
{
  viewer_->add
  ( CreatePointCloudObject<Point> ("vertices", "v")
  . description                   ("Graph vertices")
  . data                          (vertices_)
  . pointSize                     (2)
  , true
  );

  viewer_->add
  ( CreatePolyDataObject ("edges", "e")
  . description          ("Graph edges")
  . data                 (edges_)
  );

  if (input_cloud_.changed () || input_indices_.changed ())
    execute ();
}

void
GraphBuildingForm::leave ()
{
  viewer_->hide ("vertices");
  viewer_->hide ("edges");
}

void
GraphBuildingForm::execute ()
{
  buildGraph();
  computeEdgeWeights ();

  output_graph_ = graph_;

  pcl::copyPointCloud (*pcl::graph::point_cloud (*graph_), *vertices_);
  boost::get (boost::vertex_color, *graph_);
  for (size_t i = 0; i < vertices_->size (); ++i)
  {
    uint32_t label = boost::get (boost::vertex_color, *graph_, i);
    vertices_->at (i).rgba = pcl::GlasbeyLUT::at (label % pcl::GlasbeyLUT::size ()).rgba;
  }

  vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
  vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();
  colors->SetNumberOfComponents (3);
  unsigned char c[3];
  boost::graph_traits<Graph>::edge_iterator s, e;
  int id = 0;
  for (boost::tie (s, e) = boost::edges (*graph_); s != e; ++s)
  {
    vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New ();
    points->InsertNextPoint ((*graph_)[boost::source (*s, *graph_)].getVector3fMap ().data ());
    points->InsertNextPoint ((*graph_)[boost::target (*s, *graph_)].getVector3fMap ().data ());
    line->GetPointIds ()->SetId (0, id++);
    line->GetPointIds ()->SetId (1, id++);
    cells->InsertNextCell (line);
    getRGBFromColor (getColor (boost::get (boost::edge_weight_t (), *graph_, *s)), c);
    colors->InsertNextTupleValue (c);
  }
  edges_->SetPoints (points);
  edges_->SetLines (cells);
  edges_->GetCellData ()->SetScalars (colors);

  viewer_->update ("vertices");
  viewer_->update ("edges");
}

void
GraphBuildingForm::onGraphBuilderEditingFinished ()
{
  if (ui_->checkbox_auto_update->checkState ())
    execute ();
}

void
GraphBuildingForm::onEdgeWeightsEditingFinished ()
{
  if (ui_->checkbox_auto_update->checkState ())
    execute ();
}

void
GraphBuildingForm::onUpdateButtonClicked ()
{
  execute ();
}

void
GraphBuildingForm::buildGraph ()
{
  pcl::graph::GraphBuilder<Point, Graph>::Ptr gb;

  switch (ui_->tabs_graph_builder->currentIndex ())
  {
    case 0: // Voxel Grid
      {
        double r = ui_->spinbox_voxel_resolution->value ();
        using Builder = pcl::graph::VoxelGridGraphBuilder<Point, Graph>;
        Builder* graph_builder = new Builder (r);
        gb.reset (graph_builder);
        break;
      }
    case 1: // KNN
      {
        int n = ui_->spinbox_nearest_neighbors->value ();
        using Builder = pcl::graph::NearestNeighborsGraphBuilder<Point, Graph>;
        Builder* graph_builder = new Builder;
        graph_builder->setNumberOfNeighbors (n);
        graph_builder->useNearestKSearch ();
        gb.reset (graph_builder);
        break;
      }
    case 2: // Radius
      {
        int n = ui_->spinbox_max_neighbors->value ();
        double r = ui_->spinbox_radius->value ();
        using Builder = pcl::graph::NearestNeighborsGraphBuilder<Point, Graph>;
        Builder* graph_builder = new Builder;
        graph_builder->setNumberOfNeighbors (n);
        graph_builder->setRadius (r);
        graph_builder->useRadiusSearch ();
        gb.reset (graph_builder);
        break;
      }
  }

  gb->setInputCloud (input_cloud_);
  if (input_indices_)
    gb->setIndices (static_cast<pcl::IndicesPtr> (input_indices_));

  status_bar_->showMessage ("Building graph...");
  gb->compute (*graph_);
  status_bar_->showMessage ("Built a graph with %i vertices and %i edges",
                            boost::num_vertices (*graph_),
                            boost::num_edges (*graph_));

  pcl::graph::computeNormalsAndCurvatures (*graph_);
  pcl::graph::computeSignedCurvatures (*graph_);
}

void
GraphBuildingForm::computeEdgeWeights ()
{
  using namespace pcl::graph;
  typedef EdgeWeightComputer<Graph> EWC;
  EWC computer;
  if (ui_->checkbox_xyz->checkState ())
  {
    float influence = ui_->spinbox_xyz_influence->value ();
    float multiplier = ui_->checkbox_xyz_only_concave->checkState () ? 0.0 : 1.0;
    computer.addTerm<terms::XYZ> (influence, multiplier, EWC::NORMALIZATION_LOCAL);
  }
  if (ui_->checkbox_normal->checkState ())
  {
    float influence = ui_->spinbox_normal_influence->value ();
    float multiplier = ui_->checkbox_normal_only_concave->checkState () ? 0.0 : 1.0;
    computer.addTerm<terms::Normal> (influence, multiplier);
  }
  if (ui_->checkbox_curvature->checkState ())
  {
    float influence = ui_->spinbox_curvature_influence->value ();
    float multiplier = ui_->checkbox_curvature_only_concave->checkState () ? 0.0 : 1.0;
    computer.addTerm<terms::Curvature> (influence, multiplier);
  }
  if (ui_->checkbox_rgb->checkState ())
  {
    float influence = ui_->spinbox_rgb_influence->value ();
    float multiplier = ui_->checkbox_rgb_only_concave->checkState () ? 0.0 : 1.0;
    computer.addTerm<terms::RGB> (influence, multiplier, EWC::NORMALIZATION_GLOBAL);
  }
  if (ui_->checkbox_verticality->checkState ())
  {
    float influence = ui_->spinbox_verticality_influence->value ();
    float multiplier = ui_->checkbox_verticality_only_concave->checkState () ? 0.0 : 1.0;
    computer.addTerm<terms::Verticality> (influence, multiplier);
  }

  auto num_edges_before = boost::num_edges (*graph_);
  computer.setSmallWeightThreshold (ui_->spinbox_small_weight_threshold->value ());
  computer.setSmallWeightPolicy (static_cast<EWC::SmallWeightPolicy> (ui_->combo_small_weight_policy->currentIndex ()));
  computer.compute (*graph_);

  auto num_edges_removed = num_edges_before - boost::num_edges (*graph_);
  if (num_edges_removed > 0)
    status_bar_->showMessage ("Removed %i edges with weights below threshold", num_edges_removed);
}

void
GraphBuildingForm::loadConfig (Config& config)
{
  config.get ("GraphBuilder.Type", ui_->tabs_graph_builder, 0);
  config.get ("GraphBuilder.VoxelGrid.Resolution", ui_->spinbox_voxel_resolution, 0.005);
  config.get ("GraphBuilder.KNN.NearestNeighbors", ui_->spinbox_nearest_neighbors, 15);
  config.get ("GraphBuilder.Radius.Radius", ui_->spinbox_radius, 0.01);
  config.get ("GraphBuilder.Radius.MaxNeighbors", ui_->spinbox_max_neighbors, 10);
  config.get ("EdgeWeights.XYZ.Enabled", ui_->checkbox_xyz, 2);
  config.get ("EdgeWeights.XYZ.Influence", ui_->spinbox_xyz_influence, 3.0);
  config.get ("EdgeWeights.XYZ.OnlyConcave", ui_->checkbox_xyz_only_concave, 0);
  config.get ("EdgeWeights.Normal.Enabled", ui_->checkbox_normal, 2);
  config.get ("EdgeWeights.Normal.Influence", ui_->spinbox_normal_influence, 0.01);
  config.get ("EdgeWeights.Normal.OnlyConcave", ui_->checkbox_normal_only_concave, 2);
  config.get ("EdgeWeights.Curvature.Enabled", ui_->checkbox_curvature, 2);
  config.get ("EdgeWeights.Curvature.Influence", ui_->spinbox_curvature_influence, 0.0001);
  config.get ("EdgeWeights.Curvature.OnlyConcave", ui_->checkbox_curvature_only_concave, 2);
  config.get ("EdgeWeights.RGB.Enabled", ui_->checkbox_rgb, 2);
  config.get ("EdgeWeights.RGB.Influence", ui_->spinbox_rgb_influence, 3.0);
  config.get ("EdgeWeights.RGB.OnlyConcave", ui_->checkbox_rgb_only_concave, 0);
  config.get ("EdgeWeights.Verticality.Enabled", ui_->checkbox_verticality, 2);
  config.get ("EdgeWeights.Verticality.Influence", ui_->spinbox_verticality_influence, 3.0);
  config.get ("EdgeWeights.Verticality.OnlyConcave", ui_->checkbox_verticality_only_concave, 0);
  config.get ("EdgeWeights.SmallWeight.Threshold", ui_->spinbox_small_weight_threshold, 1e-5);
  config.get ("EdgeWeights.SmallWeight.Policy", ui_->combo_small_weight_policy, 1);
}

void
GraphBuildingForm::saveConfig (Config& config) const
{
  config.put ("GraphBuilder.Type", ui_->tabs_graph_builder);
  config.put ("GraphBuilder.VoxelGrid.Resolution", ui_->spinbox_voxel_resolution);
  config.put ("GraphBuilder.KNN.NearestNeighbors", ui_->spinbox_nearest_neighbors);
  config.put ("GraphBuilder.Radius.Radius", ui_->spinbox_radius);
  config.put ("GraphBuilder.Radius.MaxNeighbors", ui_->spinbox_max_neighbors);
  config.put ("EdgeWeights.XYZ.Enabled", ui_->checkbox_xyz);
  config.put ("EdgeWeights.XYZ.Influence", ui_->spinbox_xyz_influence);
  config.put ("EdgeWeights.XYZ.OnlyConcave", ui_->checkbox_rgb_only_concave);
  config.put ("EdgeWeights.Normal.Enabled", ui_->checkbox_normal);
  config.put ("EdgeWeights.Normal.Influence", ui_->spinbox_normal_influence);
  config.put ("EdgeWeights.Normal.OnlyConcave", ui_->checkbox_normal_only_concave);
  config.put ("EdgeWeights.Curvature.Enabled", ui_->checkbox_curvature);
  config.put ("EdgeWeights.Curvature.Influence", ui_->spinbox_curvature_influence);
  config.put ("EdgeWeights.Curvature.OnlyConcave", ui_->checkbox_curvature_only_concave);
  config.put ("EdgeWeights.RGB.Enabled", ui_->checkbox_rgb);
  config.put ("EdgeWeights.RGB.Influence", ui_->spinbox_rgb_influence);
  config.put ("EdgeWeights.RGB.OnlyConcave", ui_->checkbox_rgb_only_concave);
  config.put ("EdgeWeights.Verticality.Enabled", ui_->checkbox_verticality);
  config.put ("EdgeWeights.Verticality.Influence", ui_->spinbox_verticality_influence);
  config.put ("EdgeWeights.Verticality.OnlyConcave", ui_->checkbox_verticality_only_concave);
  config.put ("EdgeWeights.SmallWeight.Threshold", ui_->spinbox_small_weight_threshold);
  config.put ("EdgeWeights.SmallWeight.Policy", ui_->combo_small_weight_policy);
}

