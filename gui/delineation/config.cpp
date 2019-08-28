#include <boost/property_tree/json_parser.hpp>

#include <QSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <QTabWidget>
#include <QDoubleSpinBox>

#include "config.h"
#include "pipeline.h"
#include "min_max_widget.h"

Config::Config ()
{
}

Config::Config (const std::string& filename)
{
  boost::property_tree::read_json (filename, pt_);
}

void
Config::save (const std::string& filename)
{
  boost::property_tree::write_json (filename, pt_);
}

void
Config::put (const std::string& name, pipeline::Stage* stage)
{
  section_ = name + ".";
  stage->saveConfig (*this);
  section_ = "";
}

void
Config::get (const std::string& name, pipeline::Stage* stage)
{
  section_ = name + ".";
  stage->loadConfig (*this);
  section_ = "";
}

template <> void
Config::put (const std::string& name, QTabWidget* obj)
{
  pt_.put (section_ + name, obj->currentIndex ());
}

template <> void
Config::get (const std::string& name, QTabWidget* obj, int dflt)
{
  obj->setCurrentIndex (pt_.get (section_ + name, dflt));
}

template <> void
Config::put (const std::string& name, QComboBox* obj)
{
  pt_.put (section_ + name, obj->currentIndex ());
}

template <> void
Config::get (const std::string& name, QComboBox* obj, int dflt)
{
  obj->setCurrentIndex (pt_.get (section_ + name, dflt));
}

template <> void
Config::put (const std::string& name, QDoubleSpinBox* obj)
{
  pt_.put (section_ + name, obj->value ());
}

template <> void
Config::get (const std::string& name, QDoubleSpinBox* obj, double dflt)
{
  obj->setValue (pt_.get (section_ + name, dflt));
}

template <> void
Config::put (const std::string& name, QSpinBox* obj)
{
  pt_.put (section_ + name, obj->value ());
}

template <> void
Config::get (const std::string& name, QSpinBox* obj, int dflt)
{
  obj->setValue (pt_.get (section_ + name, dflt));
}

template <> void
Config::put (const std::string& name, QCheckBox* obj)
{
  pt_.put (section_ + name, obj->checkState ());
}

template <> void
Config::get (const std::string& name, QCheckBox* obj, int dflt)
{
  obj->setCheckState (Qt::CheckState (pt_.get (section_ + name, dflt)));
}

template <> void
Config::put (const std::string& name, MinMaxWidget* obj)
{
  pt_.put (section_ + name + ".Min", obj->value ().first);
  pt_.put (section_ + name + ".Max", obj->value ().second);
  pt_.put (section_ + name + ".RangeMin", obj->range ().first);
  pt_.put (section_ + name + ".RangeMax", obj->range ().second);
}

template <> void
Config::get (const std::string& name, MinMaxWidget* obj)
{
  obj->setRange (pt_.get (section_ + name + ".Min", 0.000),
                 pt_.get (section_ + name + ".Max", 0.000));
  obj->setValue (pt_.get (section_ + name + ".RangeMin", 0.000),
                 pt_.get (section_ + name + ".RangeMax", 0.000));
}

