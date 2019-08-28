#ifndef MEASURE_RUNTIME_H
#define MEASURE_RUNTIME_H

#include <pcl/console/time.h>
#include <pcl/console/print.h>

#define MEASURE_RUNTIME(description, function) \
        { \
          pcl::console::TicToc tt; \
          pcl::console::print_info ((description)); \
          tt.tic (); \
          (function); \
          pcl::console::print_value (" [%g ms]\n", tt.toc ()); \
        }

#define MEASURE_RUNTIME_MULTILINE(description, function) \
        { \
          pcl::console::TicToc tt; \
          pcl::console::print_info ((description " >>>\n")); \
          tt.tic (); \
          (function); \
          pcl::console::print_info (">>> done with %s ", description); \
          pcl::console::print_value ("[%g ms]\n", tt.toc ()); \
        }

#endif /* MEASURE_RUNTIME_H */

