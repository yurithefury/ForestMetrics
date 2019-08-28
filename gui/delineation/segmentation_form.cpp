#include <QMessageBox>

#include <pcl/common/colors.h>

#include "tviewer/tviewer.h"
#include "tviewer/tviewer_widget/tviewer_widget.h"

#include "status_bar.h"
#include "segmentation_form.h"
#include "ui_segmentation_form.h"

#include "random_walker_segmentation.h"

using namespace tviewer;

SegmentationForm::SegmentationForm (QWidget* parent)
: QWidget (parent)
, ui_ (new Ui::SegmentationForm)
, seeds_ (new PointCloudLabel)
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
}

SegmentationForm::~SegmentationForm ()
{
  delete ui_;
}

void
SegmentationForm::enter ()
{
  if (input_seeds_.changed () && input_seeds_)
    pcl::copyPointCloud<PointLabel> (input_seeds_, *seeds_);

  viewer_->add
  ( CreatePointCloudObject<PointLabel> ("seeds", "s")
  . description                        ("Seeds")
  . pointSize                          (7)
  . data                               (seeds_)
  , true
  );

  viewer_->add
  ( CreatePointCloudObject<pcl::PointXYZRGBA> ("trees", "t")
  . description                               ("Segmented trees")
  . pointSize                                 (2)
  . onUpdate                                  ([&]
    {
      return cluster_list_->getPointCloudForVisualization ();
    })
  );

  connect (viewer_,
           SIGNAL (pointPicked (QString, int)),
           this,
           SLOT (onPointPicked (QString, int)));
}

void
SegmentationForm::leave ()
{
  viewer_->remove ("trees");
  viewer_->hide ("seeds");
  viewer_->disconnect (this);
}

void
SegmentationForm::onButtonSegmentClicked ()
{
  if (!input_graph_)
  {
    QMessageBox::warning (this, "Missing graph", "A graph is required to perform segmentation.\n"
                                                 "Visit \"Graph building\" form to create it.");
    return;
  }
  if (!input_seeds_)
  {
    QMessageBox::warning (this, "Missing seeds", "Seeds are required to perform segmentation.\n"
                                                 "Load existing seeds using \"File\" menu, or visit \"Trunk detection\" form to create new seeds from the data.");
    return;
  }

  status_bar_->showMessage ("Segmenting graph...");

  rws_.setInputGraph (input_graph_);
  rws_.setSeeds (input_seeds_);

  rws_.setClustersWithNoSeedsLabeling (ui_->checkbox_unique_labels->checkState () ? rws_.DIFFERENT_GENERATED_LABELS : rws_.SAME_GENERATED_LABEL);
  std::vector<pcl::PointIndices> clusters;
  rws_.segment (clusters);

  PointCloud::Ptr vertices (new PointCloud);
  pcl::copyPointCloud (*pcl::graph::point_cloud (*rws_.getGraph ()), *vertices);
  cluster_list_->setClusterList (vertices, clusters);

  PointCloudLabel::Ptr labeled_vertices (new PointCloudLabel);
  pcl::copyPointCloud (*pcl::graph::point_cloud (*rws_.getGraph ()), *labeled_vertices);
  for (size_t i = 0; i < labeled_vertices->size (); ++i)
    labeled_vertices->at (i).label = boost::get (boost::vertex_color, *rws_.getGraph(), i);
  output_segmentation_ = labeled_vertices;

  viewer_->update ("trees");
  viewer_->show ("trees");

  status_bar_->showMessage ("Segmented graph into %i clusters", clusters.size () - 1);
}

void
SegmentationForm::onClusterSelectionChanged ()
{
  viewer_->update ("trees");
}

void
SegmentationForm::onPointPicked (QString name, int index)
{
  if (name == "trees")
    ui_->list_clusters->setCurrentIndex (cluster_list_->getIndexFromPointIndex (index));
}

void
SegmentationForm::loadConfig (Config& config)
{
  config.get ("UniqueLabels", ui_->checkbox_unique_labels, 2);
}

void
SegmentationForm::saveConfig (Config& config) const
{
  config.put ("UniqueLabels", ui_->checkbox_unique_labels);
}

