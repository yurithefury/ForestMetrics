#ifndef FACTORY_FACTORY_H
#define FACTORY_FACTORY_H

#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/join.hpp>

namespace factory
{

namespace printer
{

void
startSection (const std::string& name = "")
{
  using namespace pcl::console;
  change_text_color (stdout, TT_RESET, TT_MAGENTA);
  std::cout << (boost::format ("%=44s\n") % name);
  reset_text_color (stdout);
  std::cout << std::setfill ('-') << std::setw (44) << "" << std::endl;
}

void
endSection ()
{
  std::cout << std::setfill ('-') << std::setw (44) << "" << std::endl;
}

template <typename T> void
printValue (const std::string& name, const T& value)
{
  pcl::console::print_info ("%21s : ", name.c_str ());
  pcl::console::print_value ("%s\n", boost::lexical_cast<std::string> (value).c_str ());
}

}

struct Option
{

  typedef std::pair<std::string, std::string> ValueInfo;

  Option (const std::string& desc, const std::string& k)
  : description (desc)
  , key (k)
  {
  }

  virtual void
  parse (int argc, char** argv) = 0;

  virtual std::vector<ValueInfo>
  getValueInfo () = 0;

  virtual std::string
  getDescription () { return description; }

  virtual std::string
  getKey () { return key; }

  std::string description;
  std::string key;

};

template <typename T>
struct NumericOption : Option
{

  typedef T value_t;

  NumericOption (const std::string& desc, const std::string& key, value_t default_value) : Option (desc, key), value (default_value) { }

  operator value_t () { return value; }

  virtual void parse (int argc, char** argv) { pcl::console::parse (argc, argv, key.c_str (), value); }

  virtual std::vector<ValueInfo> getValueInfo () { return { std::make_pair (description, boost::lexical_cast<std::string> (value)) }; }

  value_t value;

};

struct BoolOption : Option
{

  typedef bool value_t;

  BoolOption (const std::string& desc, const std::string& key) : Option (desc, key), value (false) { }

  operator value_t () { return value; }

  virtual void parse (int argc, char** argv) { value = pcl::console::find_switch (argc, argv, key.c_str ()); }

  virtual std::vector<ValueInfo> getValueInfo () { return { std::make_pair (description, value ? "ON" : "OFF") }; }

  value_t value;

};

struct EnumOption : Option
{

  typedef std::string value_t;
  typedef std::vector<std::pair<std::string, std::string>> list_t;

  EnumOption (const std::string& desc, const std::string& key, const list_t& vars)
  : Option (desc, key)
  , variants (vars)
  , value (vars[0].first)
  {
  }

  operator value_t ()
  {
    return value;
  }

  virtual void
  parse (int argc, char** argv)
  {
    pcl::console::parse (argc, argv, key.c_str (), value);
    for (const auto& p : variants)
      if (p.first == value)
        return;
    pcl::console::print_warn ("Invalid value for %s option: \"%s\". Defaulting to \"%s\"\n", key.c_str (), value.c_str (), variants[0].first.c_str ());
  }

  virtual std::vector<ValueInfo>
  getValueInfo ()
  {
    for (const auto& p : variants)
      if (p.first == value)
        return { std::make_pair (description, p.second) };
    return { std::make_pair (description, "UNKNOWN") };
  }

  virtual std::string
  getDescription ()
  {
    std::vector<std::string> names;
    for (const auto& p : variants)
      names.push_back (p.first);
    return description + " [" + boost::algorithm::join (names, "|") + "]";
  }

  list_t variants;
  value_t value;

};


class Factory
{

public:

  Factory (const std::string& name) : name_ (name) { };

  virtual const std::string
  getUsage ()
  {
    std::stringstream usage;
    usage << name_ << " options: ";
    boost::format fmt ("\n  %s <%s>");
    for (const auto& option : options_)
      usage << boost::str (fmt % option->getKey () % option->getDescription ());
    return usage.str ();
  }

  virtual void
  printValues ()
  {
    printer::startSection (name_);
    for (const auto& option : options_)
      for (const auto& value : option->getValueInfo ())
        printer::printValue (value.first, value.second);
    printer::endSection ();
  }

protected:

  inline void
  add (Option* option)
  {
    options_.push_back (option);
  }

  inline void
  parse (int argc, char** argv)
  {
    for (const auto& option : options_)
      option->parse (argc, argv);
  }

private:

  std::vector<Option*> options_;
  std::string name_;

};

}

#endif /* FACTORY_FACTORY_H */

