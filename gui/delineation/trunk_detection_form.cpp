#include <pcl/common/common.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>

#include "tviewer/tviewer.h"
#include "tviewer/tviewer_widget/tviewer_widget.h"

#include "kde.h"
#include "config.h"
#include "mesh_grid.h"
#include "status_bar.h"
#include "tree_top_detector.h"
#include "trunk_detection_form.h"
#include "ui_trunk_detection_form.h"

using namespace tviewer;

TrunkDetectionForm::TrunkDetectionForm (QWidget* parent)
: QWidget (parent)
, ui_ (new Ui::TrunkDetectionForm)
, input_ (new PointCloud)
, tree_seeds_ (new PointCloudLabel)
, cluster_list_ (new ClusterListModel)
{
  ui_->setupUi (this);
  ui_->list_clusters->setModel (cluster_list_.get ());

  connect (ui_->list_clusters->selectionModel (),
           SIGNAL (selectionChanged (QItemSelection, QItemSelection)),
           cluster_list_.get (),
           SLOT (currentChanged (QItemSelection, QItemSelection)));
  connect (cluster_list_.get (),
           SIGNAL (selectionChanged ()),
           this,
           SLOT (onClusterSelectionChanged ()));
  connect (ui_->minmax_height,
           SIGNAL (valueChanged (double, double)),
           this,
           SLOT (onHeightFilterChanged ()));
}

TrunkDetectionForm::~TrunkDetectionForm ()
{
  delete ui_;
}

void
TrunkDetectionForm::enter ()
{
  if (input_cloud_.changed () || input_indices_.changed ())
  {
    if (input_indices_)
      pcl::copyPointCloud<Point> (input_cloud_, input_indices_, *input_);
    else
      pcl::copyPointCloud<Point> (input_cloud_, *input_);
    Point min;
    Point max;
    pcl::getMinMax3D (*input_, min, max);
    ui_->minmax_height->setRange (min.z, max.z);
    onHeightFilterChanged ();
  }

  viewer_->add
  ( CreatePointCloudObject<Point> ("points", "p")
  . description                   ("Input point cloud")
  . pointSize                     (2)
  . color                         (0xFFF000)
  . onUpdate                      ([&]
    {
      PointCloud::Ptr cloud (new PointCloud);
      pcl::copyPointCloud (*input_, *filtered_indices_, *cloud);
      return cloud;
    })
  , true
  , true
  );

  viewer_->add
  ( CreatePointCloudObject<pcl::PointXYZRGBA> ("trees", "t")
  . description                               ("Detected trees")
  . pointSize                                 (8)
  . onUpdate                                  ([&]
    {
      return cluster_list_->getPointCloudForVisualization ();
    })
  );
}

void
TrunkDetectionForm::leave ()
{
  viewer_->remove ("trees");
  viewer_->remove ("points");
  viewer_->remove ("kde");
}

void
TrunkDetectionForm::onHeightFilterChanged ()
{
  auto z = ui_->minmax_height->value ();
  filtered_indices_.reset (new std::vector<int>);
  for (size_t i = 0; i < input_->size (); ++i)
  {
    auto& point = input_->at (i);
    if (point.z >= z.first && point.z <= z.second)
      filtered_indices_->push_back (i);
  }
  viewer_->update ("points");
}

void
TrunkDetectionForm::onButtonDetectClicked ()
{
  switch (ui_->tabs_detection->currentIndex ())
  {
    case 0: // Bottom-up
      {
        performBottomUpTreeDetection ();
        break;
      }
    case 1: // Top-down
      {
        performTopDownTreeDetection ();
        break;
      }
  }
  cluster_list_->setClusterList (input_, tree_cluster_indices_);
  updateTreeSeeds ();
  status_bar_->showMessage ("Detected %d trees", tree_cluster_indices_.size ());
}

void
TrunkDetectionForm::onButtonFitClicked ()
{
  ArrowsPtr a (new Arrows);
  tree_line_coefficients_.resize (tree_cluster_indices_.size ());

  for (size_t i = 0; i < tree_cluster_indices_.size (); ++i)
  {
    const auto& tree_indices = tree_cluster_indices_[i];
    if (tree_indices.indices.size () == 1)
    {
      // Special case: single point, make a vertical line through it
      auto pt = input_->at (tree_indices.indices[0]);
      tree_line_coefficients_[i].resize (6);
      tree_line_coefficients_[i] << pt.x, pt.y, pt.z, 0, 0, 1;
    }
    else
    {
      typedef pcl::SampleConsensusModelParallelLine<pcl::PointXYZRGB> Model;
      Model::Ptr model (new Model (input_, tree_indices.indices));
      model->setAxis (Eigen::Vector3f (0, 0, 1));
      model->setEpsAngle (M_PI * ui_->spinbox_angular_threshold->value () / 180.0);
      pcl::RandomSampleConsensus<pcl::PointXYZRGB> sac (model);
      sac.setDistanceThreshold (ui_->spinbox_distance_threshold->value ());
      auto result = sac.computeModel ();
      std::vector<int> inliers;
      sac.getInliers (inliers);
      if (!result || inliers.size () == 0)
        continue;
      sac.refineModel ();
      sac.getModelCoefficients (tree_line_coefficients_[i]);
    }
    Eigen::Vector3f point = tree_line_coefficients_[i].topRows (3);
    Eigen::Vector3f direction = tree_line_coefficients_[i].bottomRows (3);
    Eigen::Vector3f pt_ground = point - direction * (point[2] / direction[2]);
    Eigen::Vector3f pt_top = point + direction * (ui_->minmax_height->range ().second - point[2]) / direction[2];
    a->push_back({pt_ground, pt_top, pcl::GlasbeyLUT::at (i % pcl::GlasbeyLUT::size ()).rgba});
  }

  ui_->button_enrich->setEnabled (true);

  viewer_->add
  ( CreateArrowArrayObject ("lines", "l")
  . description          ("Fitted lines")
  . data                 (a)
  , true
  );
}

void
TrunkDetectionForm::onButtonEnrichClicked ()
{
  typedef pcl::SampleConsensusModelLine<pcl::PointXYZRGB> Model;
  Model::Ptr model (new Model (input_));
  for (size_t i = 0; i < tree_line_coefficients_.size (); ++i)
  {
    if (tree_line_coefficients_[i].size () == 0)
      continue;
    model->selectWithinDistance (tree_line_coefficients_[i], ui_->spinbox_distance_threshold->value (), tree_cluster_indices_[i].indices);
  }
  cluster_list_->setClusterList (input_, tree_cluster_indices_);
  updateTreeSeeds ();
}

void
TrunkDetectionForm::onButtonUseAsSeedsClicked ()
{
  output_seeds_ = tree_seeds_;
}

void
TrunkDetectionForm::onClusterSelectionChanged ()
{
  viewer_->update ("trees");
}

void
TrunkDetectionForm::loadConfig (Config& config)
{
  config.get ("BottomUp.HeightFilter", ui_->minmax_height);
  config.get ("BottomUp.EuclideanClustering.DistanceThreshold.Horizontal", ui_->spinbox_threshold_horizontal, 0.6);
  config.get ("BottomUp.EuclideanClustering.DistanceThreshold.Vertical", ui_->spinbox_threshold_vertical, 0.4);
  config.get ("BottomUp.EuclideanClustering.ClusterTolerance", ui_->spinbox_cluster_tolerance, 8.0);
  config.get ("BottomUp.EuclideanClustering.MinimumClusterSize", ui_->spinbox_min_cluster_size, 5);
  config.get ("LineFitting.AngularThreshold", ui_->spinbox_angular_threshold, 5);
  config.get ("LineFitting.DistanceThreshold", ui_->spinbox_distance_threshold, 0.2);
}

void
TrunkDetectionForm::saveConfig (Config& config) const
{
  config.put ("BottomUp.HeightFilter", ui_->minmax_height);
  config.put ("BottomUp.EuclideanClustering.DistanceThreshold.Horizontal", ui_->spinbox_threshold_horizontal);
  config.put ("BottomUp.EuclideanClustering.DistanceThreshold.Vertical", ui_->spinbox_threshold_vertical);
  config.put ("BottomUp.EuclideanClustering.ClusterTolerance", ui_->spinbox_cluster_tolerance);
  config.put ("BottomUp.EuclideanClustering.MinimumClusterSize", ui_->spinbox_min_cluster_size);
  config.put ("LineFitting.AngularThreshold", ui_->spinbox_angular_threshold);
  config.put ("LineFitting.DistanceThreshold", ui_->spinbox_distance_threshold);
}

void
TrunkDetectionForm::updateTreeSeeds ()
{
  tree_seeds_.reset (new PointCloudLabel);
  uint32_t label = 0;
  for (const auto& indices : tree_cluster_indices_)
  {
    for (const auto& index : indices.indices)
    {
      PointLabel p;
      pcl::copyPoint (input_->at (index), p);
      p.label = label;
      tree_seeds_->push_back (p);
    }
    ++label;
  }
  viewer_->update ("trees");
  viewer_->show ("trees");
}

void
TrunkDetectionForm::performBottomUpTreeDetection ()
{
  pcl::ConditionalEuclideanClustering<Point> cec (true);
  cec.setInputCloud (input_);
  if (filtered_indices_)
    cec.setIndices (filtered_indices_);
  cec.setConditionFunction (boost::bind (&TrunkDetectionForm::euclideanClusteringCondition, this, _1, _2, _3));
  cec.setClusterTolerance (ui_->spinbox_cluster_tolerance->value ());
  cec.setMinClusterSize (ui_->spinbox_min_cluster_size->value ());
  cec.segment (tree_cluster_indices_);
}

void
TrunkDetectionForm::performTopDownTreeDetection ()
{
  TreeTopDetector ttd;
  ttd.setInputCloud (input_);
  ttd.setRadius (ui_->spinbox_radius->value ());
  ttd.detect (tree_cluster_indices_);

  Point min, max;
  pcl::getMinMax3D (*input_, min, max);

  MeshGrid mesh (min.x, max.x, min.y, max.y,  0.05);
  auto p = mesh.createPolyData (ttd.getDensity (), -5);

  viewer_->add
  ( CreatePolyDataObject ("kde", "k")
  . description          ("Kernel density estimate")
  . data                 (p)
  . representation       (pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE)
  , true
  );
}

bool
TrunkDetectionForm::euclideanClusteringCondition (const Point& a, const Point& b, float dist)
{
  float h_diff_sqr = std::pow(ui_->spinbox_threshold_horizontal->value (), 2);
  float v_diff = ui_->spinbox_threshold_vertical->value ();
  if ((std::pow (a.x - b.x, 2) + std::pow (a.y - b.y, 2) < h_diff_sqr) &&
      (std::fabs (a.z - b.z) > v_diff))
    return (true);
  else
    return (false);
}

