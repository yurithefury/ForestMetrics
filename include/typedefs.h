#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include <boost/ref.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "graph/point_cloud_graph.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointXYZRGBNormal PointWithNormalT;
typedef pcl::Normal NormalT;

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointWithNormalT> PointCloudWithNormalT;
typedef pcl::PointCloud<NormalT> NormalCloudT;
typedef typename PointCloudT::Ptr PointCloudTPtr;
typedef typename PointCloudWithNormalT::Ptr PointCloudWithNormalTPtr;
typedef typename NormalCloudT::Ptr NormalCloudTPtr;

#ifndef WC_SMALL_WEIGHT_POLICY
  #define WC_SMALL_WEIGHT_POLICY policy::coerce
#endif

namespace graph_with_normals
{

typedef
  boost::subgraph<
    pcl::graph::point_cloud_graph<
      PointWithNormalT
    , boost::vecS
    , boost::undirectedS
    , boost::property<boost::vertex_color_t, uint32_t>
    , boost::property<boost::edge_weight_t, float
    , boost::property<boost::edge_index_t, int>>
    >
  > Graph;

}

namespace graph_without_normals
{

typedef
  boost::subgraph<
    pcl::graph::point_cloud_graph<
      PointT
    , boost::vecS
    , boost::undirectedS
    , boost::property<boost::vertex_color_t, uint32_t>
    , boost::property<boost::edge_weight_t, float
    , boost::property<boost::edge_index_t, int>>
    >
  > Graph;

}

#ifndef GRAPH_WITHOUT_NORMALS
  typedef graph_with_normals::Graph Graph;
#else
  typedef graph_without_normals::Graph Graph;
#endif

typedef boost::shared_ptr<Graph> GraphPtr;
typedef boost::reference_wrapper<Graph> GraphRef;
typedef std::vector<GraphRef> GraphRefVector;
typedef boost::graph_traits<Graph>::vertex_descriptor VertexId;
typedef boost::graph_traits<Graph>::edge_descriptor EdgeId;
typedef boost::graph_traits<Graph>::edge_iterator EdgeIterator;

#endif /* TYPEDEFS_H */

