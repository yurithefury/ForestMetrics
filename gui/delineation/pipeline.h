#ifndef GUI_DELINEATION_PIPELINE
#define GUI_DELINEATION_PIPELINE

#include <map>
#include <vector>
#include <iostream>

#include <boost/signals2.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <boost/property_tree/ptree.hpp>

#include "config.h"

class StatusBar;
class TViewerWidget;

namespace pipeline
{

  class PipelineException : public std::runtime_error
  {

    public:

      PipelineException (const std::string& msg)
      : std::runtime_error (msg.c_str ())
      {
      }

  };

  template <typename T> class Input;
  template <typename T> class Output;
  template <typename T> class Hub;

  class HubBase
  {

    public:

      HubBase (const std::string& name)
      : name_ (name)
      {
      }

      virtual const std::type_info&
      getType () const = 0;

      bool
      hasSameType (const std::type_info& type) const
      {
        return getType () == type;
      }

      bool
      hasSameType (const HubBase& hub) const
      {
        return getType () == hub.getType ();
      }

    protected:

      std::string name_;

  };

  class Stage
  {

    public:

      virtual ~Stage () {};

      virtual void
      enter ()
      {
      }

      virtual void
      leave ()
      {
      }

      virtual void
      loadConfig (Config& config)
      {
      }

      virtual void
      saveConfig (Config& config) const
      {
      }

    protected:

      std::shared_ptr<StatusBar> status_bar_;
      TViewerWidget* viewer_;

    private:

      friend class Pipeline;

  };


  class Pipeline : public boost::noncopyable
  {

    public:

      using DataChangedSignal = boost::signals2::signal<void (const std::string&)>;
      using DataChangedCallback = DataChangedSignal::slot_function_type;

      static Pipeline&
      getInstance ()
      {
        static Pipeline instance;
        return instance;
      }

      template <typename T> Hub<T>&
      getHub (const std::string& name)
      {
        if (!hubs_.count (name))
          hubs_[name].reset (new Hub<T> (name, data_changed_signal_));
        if (!hubs_[name]->hasSameType (typeid (T)))
          throw PipelineException ("hub type mismatch");
        return *dynamic_cast<Hub<T>*> (hubs_[name].get ());
      }

      Stage*
      createStage (const std::string& name, Stage* stage)
      {
        if (stages_.count (name) != 0)
          throw PipelineException ("failed to create stage: stage already exists");
        stages_.insert ({name, {stage, stages_.size ()}});
        return stages_[name].first;
      }

      void
      registerDataChangedCallback (DataChangedCallback callback)
      {
        data_changed_signal_.connect (callback);
      }

      void
      setStatusBar (std::shared_ptr<StatusBar> status_bar)
      {
        for (auto& stage : stages_)
          stage.second.first->status_bar_ = status_bar;
      }

      void
      setViewer (TViewerWidget* viewer)
      {
        for (auto& stage : stages_)
          stage.second.first->viewer_ = viewer;
      }

      void
      activate (int index)
      {
        for (auto& stage : stages_)
        {
          if (stage.second.second == index)
          {
            if (stages_.count (current_stage_))
              stages_[current_stage_].first->leave ();
            stage.second.first->enter ();
            current_stage_ = stage.first;
            return;
          }
        }
      }

      int
      activate (const std::string& name)
      {
        if (!stages_.count (name))
          throw PipelineException ("failed to activate stage: stage does not exist");
        if (stages_.count (current_stage_))
          stages_[current_stage_].first->leave ();
        stages_[name].first->enter ();
        current_stage_ = name;
        return stages_[name].second;
      }

      void
      saveConfigs (Config& config)
      {
        for (const auto& stage : stages_)
          config.put (stage.first, stage.second.first);
      }

      void
      loadConfigs (Config& config)
      {
        for (auto& stage : stages_)
          config.get (stage.first, stage.second.first);
      }

    private:

      Pipeline ()
      {
      }

      std::map<std::string, std::unique_ptr<HubBase>> hubs_;
      std::map<std::string, std::pair<Stage*, int>> stages_;

      std::string current_stage_ = "";

      DataChangedSignal data_changed_signal_;

  };

  template <typename T>
  class Hub : public HubBase, boost::noncopyable
  {

    public:

      Hub (const std::string& name, const Pipeline::DataChangedSignal& data_changed_signal)
      : HubBase (name)
      , data_changed_signal_ (data_changed_signal)
      {
      }

      void
      connect (Input<T>* input)
      {
        out_links_.push_back (input);
      }

      void
      connect (Output<T>* output)
      {
        in_links_.push_back (output);
      }

      void
      put (boost::shared_ptr<T> data);

      boost::shared_ptr<T>
      get () const
      {
        return data_;
      }

      virtual const std::type_info&
      getType () const override
      {
        return typeid (T);
      }

    private:

      std::vector<Output<T>*> in_links_;
      std::vector<Input<T>*> out_links_;

      boost::shared_ptr<T> data_;

      const Pipeline::DataChangedSignal& data_changed_signal_;

  };

  template <typename T>
  class Input
  {

    public:

      Input (const char* hub_name)
      : hub_ (Pipeline::getInstance ().getHub<T> (hub_name))
      {
        hub_.connect (this);
      }

      bool
      changed () const
      {
        return data_changed_;
      }

      void
      setChanged ()
      {
        data_changed_ = true;
      }

      operator bool ()
      {
        data_changed_ = false;
        return static_cast<bool> (hub_.get ());
      }

      operator T& ()
      {
        data_changed_ = false;
        return *hub_.get ();
      }

      operator boost::shared_ptr<T> ()
      {
        data_changed_ = false;
        return hub_.get ();
      }

      operator boost::shared_ptr<const T> ()
      {
        data_changed_ = false;
        return hub_.get ();
      }

    private:

      Hub<T>& hub_;
      bool data_changed_ = false;

  };

  /** Have to define it after Input, but before Output. */
  template <typename T> void
  Hub<T>::put (boost::shared_ptr<T> data)
  {
    data_ = data;
    for (const auto& out : out_links_)
      out->setChanged ();
    data_changed_signal_ (name_);
  }

  template <typename T>
  class Output
  {

    public:

      Output (const char* hub_name)
      : hub_ (Pipeline::getInstance ().getHub<T> (hub_name))
      {
        hub_.connect (this);
      }

      void operator= (const boost::shared_ptr<T>& data)
      {
        hub_.put (data);
      }

      void operator= (boost::none_t)
      {
        hub_.put (boost::shared_ptr<T> ());
      }

    private:

      Hub<T>& hub_;

  };

}

#endif /* GUI_DELINEATION_PIPELINE */

