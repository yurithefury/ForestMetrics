#include "tviewer/tviewer.h"
#include "tviewer/tviewer_widget/tviewer_widget.h"

#include "seed_selection.h"
#include "seed_selection_form.h"
#include "ui_seed_selection_form.h"

using namespace tviewer;

SeedSelectionForm::SeedSelectionForm (QWidget* parent)
: QWidget (parent)
, ui_ (new Ui::SeedSelectionForm)
, seed_selection_ (new SeedSelection)
, vertices_ (new PointCloud)
, seeds_ (new PointCloudLabel)
{
  ui_->setupUi (this);
  ui_->list_labels->setModel (seed_selection_.get ());
}

SeedSelectionForm::~SeedSelectionForm ()
{
  delete ui_;
}

void
SeedSelectionForm::enter ()
{
  if (input_graph_.changed () && input_graph_)
  {
    GraphPtr graph = input_graph_;
    pcl::copyPointCloud (*pcl::graph::point_cloud (*graph), *vertices_);
    for (size_t i = 0; i < vertices_->size (); ++i)
      vertices_->at (i).rgba = 0xFFF000;
  }
  if (input_seeds_.changed () || filter_predicate_.changed ())
  {
    if (filter_predicate_)
    {
      std::function<bool (const Eigen::Vector3f&)> predicate = filter_predicate_;
      PointCloudLabel in = input_seeds_;
      std::vector<int> indices;
      for (size_t i = 0; i < in.size (); ++i)
      {
        if (predicate (in.at (i).getVector3fMap ()))
          continue;
        indices.push_back (i);
      }
      pcl::copyPointCloud<PointLabel> (in, indices, *seeds_);
    }
    else
    {
      pcl::copyPointCloud<PointLabel> (input_seeds_, *seeds_);
    }
    seed_selection_->setSeeds (*seeds_);
    output_seeds_ = seeds_;
  }

  viewer_->add
  ( CreatePointCloudObject<Point> ("points", "p")
  . description                   ("Input point cloud")
  . data                          (vertices_)
  . pointSize                     (2)
  , true
  );

  viewer_->add
  ( CreatePointCloudObject<Point> ("seeds", "s")
  . description                   ("Seeds")
  . pointSize                     (7)
  . onUpdate                      ([&]
    {
      return seed_selection_->getPointCloudForVisualization ();
    })
  , true
  , true
  );
}

void
SeedSelectionForm::leave ()
{
  viewer_->remove ("seeds");
}

void
SeedSelectionForm::onButtonNewLabelClicked ()
{
}

