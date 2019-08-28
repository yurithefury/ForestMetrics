#ifndef GUI_DELINEATION_CONFIG_H
#define GUI_DELINEATION_CONFIG_H

#include <string>

#include <boost/property_tree/ptree.hpp>

namespace pipeline
{
  class Stage;
}

class Config
{

  public:

    /// Constructor that creates an empty config.
    Config ();

    /// Constructor that loads existing config.
    Config (const std::string& filename);

    void
    save (const std::string& filename);

    void
    put (const std::string& name, pipeline::Stage* obj);

    void
    get (const std::string& name, pipeline::Stage* obj);

    template <typename T> void
    put (const std::string& name, T* obj);

    template <typename T> void
    get (const std::string& name, T* obj);

    template <typename T, typename M> void
    get (const std::string& name, T* obj, M dflt);

  private:

    boost::property_tree::ptree pt_;
    std::string section_;

};

#endif /* GUI_DELINEATION_CONFIG_H */

