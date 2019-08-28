#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>

#include "graph/point_cloud_graph.h"

// Based on geom_utils.h from object discovery source code.

template <typename PointT> void
meshToPointsAndNormals (const pcl::PolygonMesh::ConstPtr& mesh,
                        typename pcl::PointCloud<PointT>::Ptr& cloud,
                        pcl::PointCloud<pcl::Normal>::Ptr& normals)
{
  // Unfold mesh, which is stored as ROS message
  pcl::fromPCLPointCloud2 (mesh->cloud, *cloud);
  std::vector<int> counts (cloud->points.size (), 0);
  normals->points.resize (cloud->points.size ());

  for (size_t i = 0; i < mesh->polygons.size (); i++)
  {
    const pcl::Vertices& vv = mesh->polygons[i];

    // Get the 3 points
    int i1 = vv.vertices[0];
    int i2 = vv.vertices[1];
    int i3 = vv.vertices[2];
    PointT& p1 = cloud->points[i1];
    PointT& p2 = cloud->points[i2];
    PointT& p3 = cloud->points[i3];

    // Convert to eigen points
    Eigen::Vector3d pe1 (p1.x, p1.y, p1.z);
    Eigen::Vector3d pe2 (p2.x, p2.y, p2.z);
    Eigen::Vector3d pe3 (p3.x, p3.y, p3.z);

    // Find normal
    Eigen::Vector3d normal = (pe2 - pe1).cross (pe3 - pe1);
    normal = normal / normal.norm ();
    pcl::Normal pnormal (normal[0], normal[1], normal[2]);

    // Smoothly blend with the old normal estimate at this point.
    // Basically each face votes for the normal of the verteces around it.
    float v;
    pcl::Normal a;
    a = normals->points[i1];
    v = 1.0 / (counts[i1] + 1.0);
    normals->points[i1] = pcl::Normal (v * pnormal.normal_x + (1.0 - v) * a.normal_x,
                                       v * pnormal.normal_y + (1.0 - v) * a.normal_y,
                                       v * pnormal.normal_z + (1.0 - v) * a.normal_z);
    a = normals->points[i2];
    v = 1.0 / (counts[i2] + 1.0);
    normals->points[i2] = pcl::Normal (v * pnormal.normal_x + (1.0 - v) * a.normal_x,
                                       v * pnormal.normal_y + (1.0 - v) * a.normal_y,
                                       v * pnormal.normal_z + (1.0 - v) * a.normal_z);
    a = normals->points[i3];
    v= 1.0 / (counts[i3] + 1.0);
    normals->points[i3] = pcl::Normal (v * pnormal.normal_x + (1.0 - v) * a.normal_x,
                                       v * pnormal.normal_y + (1.0 - v) * a.normal_y,
                                       v * pnormal.normal_z + (1.0 - v) * a.normal_z);
    counts[i1]++;
    counts[i2]++;
    counts[i3]++;
  }
}

template <typename Graph> void
mesh2graph (const std::string& filename, Graph& graph)
{
  typedef typename boost::graph_traits<Graph>::vertex_descriptor VertexId;
  typedef typename boost::graph_traits<Graph>::edge_iterator EdgeIterator;

  vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New ();
  vtkSmartPointer<vtkPLYReader> ply_reader = vtkSmartPointer<vtkPLYReader>::New ();
  ply_reader->SetFileName (filename.c_str ());
  ply_reader->Update ();
  poly_data = ply_reader->GetOutput ();

  size_t num_points = poly_data->GetNumberOfPoints ();

  // First get the xyz information
  graph = Graph (num_points);
  for (VertexId i = 0; i < num_points; i++)
  {
    double point_xyz[3];
    poly_data->GetPoint (i, &point_xyz[0]);
    graph[i].getVector3fMap () = Eigen::Vector3f (point_xyz[0], point_xyz[1], point_xyz[2]);
  }

  // Then the color information, if any
  vtkUnsignedCharArray* poly_colors = NULL;
  if (poly_data->GetPointData() != NULL)
    poly_colors = vtkUnsignedCharArray::SafeDownCast (poly_data->GetPointData ()->GetScalars ("RGB"));
  if (poly_colors && (poly_colors->GetNumberOfComponents () == 3))
  {
    for (VertexId i = 0; i < num_points; i++)
    {
      uint8_t color[3];
      poly_colors->GetTupleValue (i, color);
      graph[i].getBGRVector3cMap () << color[2], color[1], color[0];
    }
  }

  // Now handle the polygons
  vtkIdType* cell_points;
  vtkIdType num_cell_points;
  vtkCellArray* mesh_polygons = poly_data->GetPolys ();
  mesh_polygons->InitTraversal ();
  int id_poly = 0;
  while (mesh_polygons->GetNextCell (num_cell_points, cell_points))
  {
    VertexId i1 = cell_points[0];
    VertexId i2 = cell_points[1];
    VertexId i3 = cell_points[2];
    const Eigen::Vector3f& p1 = graph[i1].getVector3fMap ();
    const Eigen::Vector3f& p2 = graph[i2].getVector3fMap ();
    const Eigen::Vector3f& p3 = graph[i3].getVector3fMap ();

    Eigen::Vector3f normal = (p2 - p1).cross (p3 - p1);
    normal.normalize ();

    graph[i1].getNormalVector3fMap () += normal;
    graph[i2].getNormalVector3fMap () += normal;
    graph[i3].getNormalVector3fMap () += normal;

    if (!boost::edge (i1, i2, graph).second)
      boost::add_edge (i1, i2, graph);
    if (!boost::edge (i2, i3, graph).second)
      boost::add_edge (i2, i3, graph);
    if (!boost::edge (i3, i1, graph).second)
      boost::add_edge (i3, i1, graph);
    ++id_poly;
  }

  for (const auto& vertex : as_range (boost::vertices (graph)))
    graph[vertex].getNormalVector3fMap ().normalize ();

  EdgeIterator ei, ee;
  for (boost::tie (ei, ee) = boost::edges (graph); ei != ee; ++ei)
  {
    const VertexId& i1 = boost::source (*ei, graph);
    const VertexId& i2 = boost::target (*ei, graph);
    const Eigen::Vector3f& p = graph[i1].getVector3fMap ();
    const Eigen::Vector3f& q = graph[i2].getVector3fMap ();
    const Eigen::Vector3f& np = graph[i1].getNormalVector3fMap ();
    const Eigen::Vector3f& nq = graph[i2].getNormalVector3fMap ();
    Eigen::Vector3f x = p - q;
    float d1 = nq.dot (x);
    float d2 = np.dot (-x);
    graph[i1].curvature += d1;
    graph[i2].curvature += d2;
  }

  for (const auto& vertex : as_range (boost::vertices (graph)))
    graph[vertex].curvature /= boost::out_degree (vertex, graph) * 0.001;
}

#endif /* CONVERSIONS_H */

