#include <boost/make_shared.hpp>

#include <pcl/common/common.h>

#include "tviewer/tviewer.h"
#include "tviewer/tviewer_widget/tviewer_widget.h"

#include "types.h"
#include "preprocessing_form.h"
#include "ui_preprocessing_form.h"

using namespace tviewer;

PreprocessingForm::PreprocessingForm (QWidget* parent)
: QWidget (parent)
, ui_ (new Ui::PreprocessingForm)
, display_ (new PointCloud)
{
  ui_->setupUi (this);
  connect (ui_->minmax_x,
           SIGNAL (valueChanged (double, double)),
           this,
           SLOT (onPassthroughFilterChanged ()));
  connect (ui_->minmax_y,
           SIGNAL (valueChanged (double, double)),
           this,
           SLOT (onPassthroughFilterChanged ()));
}

PreprocessingForm::~PreprocessingForm ()
{
  delete ui_;
}

void
PreprocessingForm::enter ()
{
  if (input_cloud_.changed ())
  {
    pcl::copyPointCloud<Point> (input_cloud_, *display_);
    Point min;
    Point max;
    pcl::getMinMax3D (*display_, min, max);
    ui_->minmax_x->setRange (min.x, max.x);
    ui_->minmax_y->setRange (min.y, max.y);
    onPassthroughFilterChanged ();
  }

  viewer_->add
  ( CreatePointCloudObject<Point> ("points", "p")
  . description                   ("Input point cloud")
  . data                          (display_)
  . pointSize                     (2)
  , true
  );
}

void
PreprocessingForm::onPassthroughFilterChanged ()
{
  uint32_t IN = 0xFFF000;
  uint32_t OUT = 0x1F1F1F;
  auto x = ui_->minmax_x->value ();
  auto y = ui_->minmax_y->value ();
  for (auto& point : *display_)
  {
    if (point.x < x.first || point.x > x.second ||
        point.y < y.first || point.y > y.second)
      point.rgba = OUT;
    else
      point.rgba = IN;
  }
  viewer_->update ("points");
}

void
PreprocessingForm::onButtonApplyClicked ()
{
  boost::shared_ptr<std::vector<int>> indices (new std::vector<int>);
  auto x = ui_->minmax_x->value ();
  auto y = ui_->minmax_y->value ();
  auto predicate = boost::make_shared<std::function<bool (const Eigen::Vector3f&)>>
    ([=] (const Eigen::Vector3f& p)
    { return p[0] < x.first || p[0] > x.second || p[1] < y.first || p[1] > y.second; });
  for (size_t i = 0; i < display_->size (); ++i)
  {
    if (predicate->operator() (display_->at (i).getVector3fMap ()))
      continue;
    indices->push_back (i);
  }
  output_indices_ = indices;
  output_predicate_ = predicate;
}

void
PreprocessingForm::onButtonResetClicked ()
{
  auto x = ui_->minmax_x->range ();
  ui_->minmax_x->setValue (x.first, x.second);
  auto y = ui_->minmax_y->range ();
  ui_->minmax_y->setValue (y.first, y.second);
  output_indices_ = boost::none;
  output_predicate_ = boost::none;
}

void
PreprocessingForm::loadConfig (Config& config)
{
  config.get ("Passthrough.X", ui_->minmax_x);
  config.get ("Passthrough.Y", ui_->minmax_y);
}

void
PreprocessingForm::saveConfig (Config& config) const
{
  config.put ("Passthrough.X", ui_->minmax_x);
  config.put ("Passthrough.Y", ui_->minmax_y);
}

