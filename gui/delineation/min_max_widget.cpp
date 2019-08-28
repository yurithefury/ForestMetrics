#include <iostream>

#include "min_max_widget.h"
#include "ui_min_max_widget.h"

MinMaxWidget::MinMaxWidget (QWidget* parent)
: QWidget (parent)
, ui_ (new Ui::MinMaxWidget)
, updating_widgets_ (false)
{
  ui_->setupUi (this);
  slider_range_ = ui_->slider_min->maximum () - ui_->slider_min->minimum ();
  ratio_ = (range_max_ - range_min_) / slider_range_;
  updateWidgets ();
}

MinMaxWidget::~MinMaxWidget ()
{
  delete ui_;
}

void
MinMaxWidget::setRange (double min, double max)
{
  Q_ASSERT (min <= max);
  if (min != range_min_ || max != range_max_)
  {
    range_min_ = min;
    range_max_ = max;
    ratio_ = (range_max_ - range_min_) / slider_range_;
    if (min_ < range_min_ || min_ > range_max_)
      min_ = range_min_;
    if (max_ > range_max_ || max_ < range_min_)
      max_ = range_max_;
    valueChanged (min_, max_);
    updateWidgets ();
  }
}

void
MinMaxWidget::setValue (double min, double max)
{
  Q_ASSERT (min <= max);
  if (min < range_min_ || min > range_max_)
    min = range_min_;
  if (max > range_max_ || max < range_min_)
    max = range_max_;
  min_ = min;
  max_ = max;
  valueChanged (min_, max_);
  updateWidgets ();
}

void
MinMaxWidget::setSingleStep (double step)
{
  Q_ASSERT (step >= 0);
  ui_->spinbox_min->setSingleStep (step);
  ui_->spinbox_max->setSingleStep (step);
}

std::pair<double, double>
MinMaxWidget::value () const
{
  return {min_, max_};
}

std::pair<double, double>
MinMaxWidget::range () const
{
  return {range_min_, range_max_};
}

double
MinMaxWidget::singleStep () const
{
  return ui_->spinbox_min->singleStep ();
}

void
MinMaxWidget::onSliderMinValueChanged (int value)
{
  if (!updating_widgets_)
  {
    min_ = toRealValue (value);
    if (min_ > max_)
    {
      max_ = min_;
    }
    valueChanged (min_, max_);
    updateWidgets ();
  }
}

void
MinMaxWidget::onSpinboxMinValueChanged (double value)
{
  if (!updating_widgets_)
  {
    min_ = value;
    if (min_ > max_)
    {
      max_ = min_;

    }
    valueChanged (min_, max_);
    updateWidgets ();
  }
}

void
MinMaxWidget::onSliderMaxValueChanged (int value)
{
  if (!updating_widgets_)
  {
    max_ = toRealValue (value);
    if (max_ < min_)
    {
      min_ = max_;
    }
    valueChanged (min_, max_);
    updateWidgets ();
  }
}

void
MinMaxWidget::onSpinboxMaxValueChanged (double value)
{
  if (!updating_widgets_)
  {
    max_ = value;
    if (max_ < min_)
    {
      min_ = max_;
    }
    valueChanged (min_, max_);
    updateWidgets ();
  }
}

void
MinMaxWidget::updateWidgets ()
{
  updating_widgets_ = true;
  ui_->spinbox_min->setRange (range_min_, range_max_);
  ui_->spinbox_max->setRange (range_min_, range_max_);
  ui_->spinbox_min->setValue (min_);
  ui_->spinbox_max->setValue (max_);
  ui_->slider_min->setValue (toSliderValue (min_));
  ui_->slider_max->setValue (toSliderValue (max_));
  updating_widgets_ = false;
}

double
MinMaxWidget::toRealValue (int value)
{
  return ratio_ * value - ratio_ * ui_->slider_min->minimum () + range_min_;
}

int
MinMaxWidget::toSliderValue (double value)
{
  double v = ui_->slider_min->minimum () + (value - range_min_) / ratio_;
  if (v > ui_->slider_min->maximum ())
    return ui_->slider_min->maximum ();
  if (v < ui_->slider_min->minimum ())
    return ui_->slider_min->minimum ();
  return v;
}

