#ifndef GUI_DELINEATION_STATUS_BAR_H
#define GUI_DELINEATION_STATUS_BAR_H

#include <boost/format.hpp>

#include <QStatusBar>

/** Wrapper around Qt Status Bar with easy formatting support. */
class StatusBar
{

  public:

    using Ptr = std::shared_ptr<StatusBar>;

    StatusBar (QStatusBar* status_bar)
    : status_bar_ (status_bar)
    {
    }

    void
    showMessage (boost::format& message)
    {
      status_bar_->showMessage (message.str ().c_str ());
    }

    template<typename TValue, typename... TArgs> void
    showMessage (boost::format& message, TValue arg, TArgs... args)
    {
      message % arg;
      showMessage (message, args...);
    }

    template<typename... TArgs> void
    showMessage (const std::string& fmt, TArgs... args)
    {
      boost::format message (fmt);
      showMessage (message, args...);
    }

  private:

    QStatusBar* status_bar_;

};


#endif /* GUI_DELINEATION_STATUS_BAR_H */

