#ifndef IO_HPP
#define IO_HPP

#include <iostream>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>

#include "graph/point_cloud_graph.h"
#include "as_range.h"

template <typename Graph> bool
saveGraph (const std::string& filename,
           const Graph& graph)
{
  pcl::console::print_highlight ("Saving to file \"%s\"... ", filename.c_str ());
  if (pcl::io::savePCDFile (filename, *pcl::graph::point_cloud (graph)))
  {
    pcl::console::print_error ("error!\n");
    return false;
  }

  std::ofstream file (filename, std::fstream::app);
  if (!file.is_open ())
  {
    pcl::console::print_error ("error!\n");
    return false;
  }

  file << "# Edges\n";
  auto weights = boost::get (boost::edge_weight, graph);
  for (const auto& edge : as_range (boost::edges (graph)))
  {
    auto src = boost::source (edge, graph);
    auto tgt = boost::target (edge, graph);
    auto wgh = weights[edge];
    file << src << " " << tgt << " " << wgh << "\n";
  }
  file.close();

  pcl::console::print_info ("%zu vertices, %zu edges.\n",
                            boost::num_vertices (graph),
                            boost::num_edges (graph));

  return true;
}

template <typename Graph> bool
loadGraph (const std::string& filename,
           Graph& graph)
{
  typedef typename pcl::graph::point_cloud_graph_traits<Graph>::point_type PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;

  typename PointCloudT::Ptr cloud (new PointCloudT);

  pcl::console::print_highlight ("Loading from file \"%s\"... ", filename.c_str ());
  if (pcl::io::loadPCDFile<PointT> (filename, *cloud))
  {
    pcl::console::print_error ("error!\n");
    return false;
  }

  std::ifstream stream (filename);
  if (!stream)
  {
    pcl::console::print_error ("error!\n");
    return false;
  }

  graph = Graph (cloud);

  std::string line;
  bool edges_section = false;
  while (getline (stream, line))
  {
    if (!edges_section)
    {
      if (line == "# Edges")
        edges_section = true;
    }
    else
    {
      if (!line.size () || line[0] == '#')
        continue;
      int src;
      int tgt;
      float wgh;
      std::stringstream (line) >> src >> tgt >> wgh;
      boost::add_edge (src, tgt, wgh, graph);
    }
  }

  pcl::console::print_info ("%zu vertices, %zu edges.\n",
                            boost::num_vertices (graph),
                            boost::num_edges (graph));

  stream.close ();
  return true;
}

#endif /* IO_HPP */

