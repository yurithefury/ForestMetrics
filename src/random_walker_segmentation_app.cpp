#include <boost/make_shared.hpp>
#include <boost/format.hpp>

#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>

#include "tviewer/tviewer.h"
#include "io.h"
#include "graph_visualizer.h"
#include "seed_utils.h"
#include "label_utils.h"
#include "measure_runtime.h"

#include "factory/edge_weight_computer_factory.h"
#include "factory/graph_builder_factory.h"

#include "graph/common.h"
#include "graph/edge_weight_computer.h"

#include "random_walker_segmentation.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZRGBNormal PointWithNormalT;
typedef pcl::Normal NormalT;

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointWithNormalT> PointWithNormalCloudT;
typedef pcl::PointCloud<NormalT> NormalCloudT;

typedef typename pcl::segmentation::RandomWalkerSegmentation<pcl::PointXYZRGB>::Graph Graph;
typedef typename pcl::segmentation::RandomWalkerSegmentation<pcl::PointXYZRGB>::GraphPtr GraphPtr;

int main (int argc, char ** argv)
{
  factory::EdgeWeightComputerFactory<Graph> wc_factory;
  factory::GraphBuilderFactory<PointWithNormalT, Graph> gb_factory;

  if (argc < 2 || pcl::console::find_switch (argc, argv, "--help"))
  {
    pcl::console::print_error ("Usage: %s <pcd-file>\n"
                               "--save-seeds <pcd-file>\n"
                               "--load-seeds <pcd-file>\n"
                               "--save <pcd-file>\n"
                               "--save-clusters\n"
                               "--potential\n"
                               "--fixed-colors\n"
                               "--unique-labels\n"
                               "--smoothing\n"
                               "--tv-no-gui\n"
                               "%s\n"
                               "%s\n"
                               , argv[0]
                               , wc_factory.getUsage ().c_str ()
                               , gb_factory.getUsage ().c_str ());
    return (1);
  }

  typename PointCloudT::Ptr cloud (new PointCloudT);
  typename NormalCloudT::Ptr normals (new NormalCloudT);

  if (!load<PointT> (argv[1], cloud, normals))
    return (1);

  bool input_has_color = hasColor (argv[1]);

  bool option_save_segmentation = pcl::console::find_switch (argc, argv, "--save");
  bool option_save_seeds = pcl::console::find_switch (argc, argv, "--save-seeds");
  bool option_load_seeds = pcl::console::find_switch (argc, argv, "--load-seeds");

  bool mode_potential = pcl::console::find_switch (argc, argv, "--potential");
  bool option_fixed_colors = pcl::console::find_switch (argc, argv, "--fixed-colors");
  bool option_unique_labels = pcl::console::find_switch (argc, argv, "--unique-labels");

  std::string segmentation_save_filename;
  if (option_save_segmentation)
    pcl::console::parse (argc, argv, "--save", segmentation_save_filename);

  std::string seeds_save_filename;
  if (option_save_seeds)
    pcl::console::parse (argc, argv, "--save-seeds", seeds_save_filename);

  std::string seeds_load_filename;
  if (option_load_seeds)
    pcl::console::parse (argc, argv, "--load-seeds", seeds_load_filename);

  bool option_save_clusters = pcl::console::find_switch (argc, argv, "--save-clusters");
  bool mode_smoothing = pcl::console::find_switch (argc, argv, "--smoothing");
  bool mode_no_gui = pcl::console::find_switch (argc, argv, "--tv-no-gui");

  if (mode_no_gui && !option_load_seeds)
  {
    pcl::console::print_error ("No GUI mode can only be used with --load-seeds option.\n");
    return (2);
  }

  auto wc = wc_factory.instantiate (argc, argv);
  auto gb = gb_factory.instantiate (argc, argv);

  wc_factory.printValues ();
  gb_factory.printValues ();


  /*********************************************************************
   *                        Setup visualization                        *
   *********************************************************************/


  using namespace tviewer;
  auto viewer = create (argc, argv);

  viewer->add
  ( CreatePointCloudObject<PointT> ("input", "i")
  . description                    ("Input point cloud")
  . data                           (cloud)
  . pointSize                      (4)
  );


  /*********************************************************************
   *                         Pre-compute graph                         *
   *********************************************************************/


  GraphPtr g (new Graph);
  auto& graph = *g;

  typename PointWithNormalCloudT::Ptr cloud_with_normals (new PointWithNormalCloudT);
  if (normals->size ())
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  else
    pcl::copyPointCloud (*cloud, *cloud_with_normals);
  gb->setInputCloud (cloud_with_normals);

  MEASURE_RUNTIME ("Building graph... ", gb->compute (graph));
  MEASURE_RUNTIME ("Computing normals... ", pcl::graph::computeNormalsAndCurvatures (graph));
  if (mode_smoothing)
  {
    MEASURE_RUNTIME ("Smoothening graph... ", pcl::graph::smoothen (graph, 0.012, 0.0012));
    MEASURE_RUNTIME ("Computing normals... ", pcl::graph::computeNormalsAndCurvatures (graph));
  }
  MEASURE_RUNTIME ("Computing curvature signs... ", pcl::graph::computeSignedCurvatures (graph));
  MEASURE_RUNTIME ("Computing edge weights... ", wc->compute (graph));

  pcl::console::print_info ("Built a graph with %zu vertices and %zu edges\n",
                            boost::num_vertices (graph),
                            boost::num_edges (graph));


  /*********************************************************************
   *                          Visualize graph                          *
   *********************************************************************/


  typedef GraphVisualizer<Graph> GraphVisualizer;
  GraphVisualizer gv (graph);

  viewer->add
  ( CreatePointCloudObject<pcl::PointXYZRGBA> ("vertices", "v")
  . description                               ("Graph vertices")
  . pointSize                                 (6)
  . data                                      (input_has_color
                                              ? gv.getVerticesCloudColorsNatural ()
                                              : gv.getVerticesCloudColorsFromMapRandom ())
  );

  viewer->add
  ( CreatePointCloudObject<pcl::PointXYZRGBA> ("curvature", "C")
  . description                               ("Vertex curvature")
  . pointSize                                 (6)
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

  viewer->show ("vertices");


  /*********************************************************************
   *                          Seed selection                           *
   *********************************************************************/


  typename pcl::PointCloud<pcl::PointXYZL>::Ptr seeds_cloud (new pcl::PointCloud<pcl::PointXYZL>);
  std::vector<pcl::PointIndices> seeds_indices;

  if (option_load_seeds)
  {
    pcl::io::loadPCDFile<pcl::PointXYZL> (seeds_load_filename, *seeds_cloud);
  }
  else
  {
    viewer->waitPointsSelected (*seeds_cloud, seeds_indices);
    if (option_save_seeds)
      pcl::io::savePCDFile (seeds_save_filename, *seeds_cloud);
  }

  viewer->add
  ( CreatePointCloudObject<pcl::PointXYZRGBA> ("seeds", "S")
  . description                               ("Random walker seeds")
  . pointSize                                 (14)
  . visibility                                (0.65)
  . color                                     (0xFF0000)
  . data                                      (seeds::createColoredCloudFromSeeds (*seeds_cloud))
  );


  /*********************************************************************
   *                         Run segmentation                          *
   *********************************************************************/


  pcl::segmentation::RandomWalkerSegmentation<pcl::PointXYZRGBA> rws (mode_potential);
  rws.setInputGraph (g);
  rws.setSeeds (seeds_cloud);

  if (mode_potential || !option_unique_labels)
    rws.setClustersWithNoSeedsLabeling (rws.SAME_FIXED_LABEL, 0xFFFFFF);
  else
    rws.setClustersWithNoSeedsLabeling (rws.DIFFERENT_GENERATED_LABELS);

  std::vector<pcl::PointIndices> clusters;

  rws.segment (clusters);

  viewer->add
  ( CreatePointCloudWithColorShufflingObject ("clusters", "c")
  . description                              ("Object clusters")
  . pointSize                                (3)
  . data                                     (option_fixed_colors
                                              ? gv.getVerticesCloudColorsFromMapFixed ()
                                              : gv.getVerticesCloudColorsFromMapRandom ())
  );

  viewer->update ();
  viewer->hide ("vertices");

  if (mode_potential)
  {
    size_t index = 0;
    Eigen::VectorXf potential = rws.getPotentials ().col (0);

    viewer->add
    ( CreatePointCloudObject<pcl::PointXYZRGBA> ("potential", "p")
    . description                               ("Random walker potentials")
    . pointSize                                 (3)
    . onUpdate                                  ([&]{ return gv.getVerticesCloudColorsFromVector (potential); })
    );

    viewer->update ("potential");
    viewer->show ("potential");

    while (viewer->waitPointSelected (index))
    {
      uint32_t label = boost::get (boost::vertex_color, graph, index);
      if (label == 0xFFFFFF)
      {
        pcl::console::print_warn ("Selected point has no label and therefore no potentials\n");
      }
      else
      {
        pcl::console::print_info ("Potential for vertex %zu (label %zu)\n", index, label);
        potential = rws.getPotentials ().col (rws.getLabelClusterMap ().at (label));
        viewer->update ("potential");
      }
    }
  }
  else
  {
    viewer->show ("clusters");
    viewer->run ();
  }

  if (option_save_clusters)
  {
    boost::format fmt ("cluster%i.pcd");
    for (size_t i = 0; i < clusters.size () - 1; ++i)
    {
      if (clusters[i].indices.size ())
      {
        PointCloudT cluster;
        pcl::copyPointCloud (*cloud, clusters[i], cluster);
        pcl::io::savePCDFile (boost::str (fmt % i), cluster);
      }
    }
  }

  if (option_save_segmentation)
  {
    pcl::PointCloud<pcl::PointXYZL>::Ptr labeled;
    labeled = labels::createLabeledCloudFromColorMap (*cloud,
                                                      gb->getPointToVertexMap (),
                                                      boost::get (boost::vertex_color, graph));
    pcl::io::savePCDFile (segmentation_save_filename, *labeled);
  }

  return (0);
}

