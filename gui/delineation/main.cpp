#include <QApplication>

#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>

#include "main_window.h"

int
main (int argc, char** argv)
{
  if (argc < 2 || pcl::console::find_switch (argc, argv, "--help"))
  {
    pcl::console::print_error ("Usage: %s <data.pcd> [seeds.pcd]"
                               , argv[0]);
    return (1);
  }

  QApplication app (argc, argv);
  MainWindow w (argv[1]);

  if (argc > 2)
    w.loadSeeds (argv[2]);

  w.show ();
  return app.exec ();
}

