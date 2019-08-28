#include <boost/make_shared.hpp>

#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "typedefs.h"

#include "io.h"
#include "tviewer/tviewer.h"
#include "measure_runtime.h"
#include "graph_visualizer.h"
#include "factory/graph_factory.h"


int main (int argc, char ** argv)
{
  factory::GraphFactory<PointT, Graph> g_factory;

  if (argc < 2 || pcl::console::find_switch (argc, argv, "--help"))
  {
    pcl::console::print_error ("Usage: %s <pcd-file>\n"
                               "%s\n"
                               , argv[0]
                               , g_factory.getUsage ().c_str ());
    return (1);
  }

  typename PointCloudT::Ptr cloud (new PointCloudT);
  typename NormalCloudT::Ptr normals (new NormalCloudT);

  if (!load<PointT> (argv[1], cloud, normals))
    return (1);


  /*********************************************************************
   *                         Pre-compute graph                         *
   *********************************************************************/


  auto g = g_factory.instantiate (cloud, argc, argv);
  auto& graph = g.get ();

  g_factory.printValues ();

  pcl::console::print_info ("Working with component of size: %zu / %zu\n",
                            boost::num_vertices (graph),
                            boost::num_edges (graph));


  /*********************************************************************
   *                          Visualize graph                          *
   *********************************************************************/


  using namespace tviewer;
  auto viewer = create (argc, argv);

  typedef GraphVisualizer<Graph> GraphVisualizer;
  GraphVisualizer gv (graph);

  viewer->add
  ( CreatePointCloudObject<pcl::PointXYZRGBA> ("vertices", "v")
  . description                               ("Graph vertices")
  . pointSize                                 (6)
  . visibility                                (0.95)
  . data                                      (gv.getVerticesCloudColorsNatural ())
  );

  viewer->add
  ( CreatePointCloudObject<pcl::PointXYZRGBA> ("curvature", "C")
  . description                               ("Vertex curvature")
  . pointSize                                 (6)
  . visibility                                (0.95)
  . data                                      (gv.getVerticesCloudColorsCurvature ())
  );

  viewer->add
  ( CreateNormalCloudObject ("normals", "n")
  . description             ("Vertex normals")
  . level                   (1)
  . scale                   (0.01)
  . data                    (gv.getVerticesNormalsCloud ())
  );

  viewer->add
  ( CreatePolyDataObject ("edges", "a")
  . description          ("Adjacency edges")
  . data                 (gv.getEdgesPolyData ())
  );

  viewer->show ("edges");
  viewer->run ();

  return (0);
}

