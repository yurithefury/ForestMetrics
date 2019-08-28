#ifndef IO_H
#define IO_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Sparse>

template <typename PointT> bool
load (const std::string& filename,
      typename pcl::PointCloud<PointT>::Ptr cloud,
      pcl::PointCloud<pcl::Normal>::Ptr normals = pcl::PointCloud<pcl::Normal>::Ptr ());

bool
hasColor (const std::string& filename);

template <typename Graph> bool
saveGraph (const std::string& filename,
           const Graph& graph);

template <typename Graph> bool
loadGraph (const std::string& filename,
           Graph& graph);

void
saveSparseMatrix (const std::string& filename,
                  const Eigen::SparseMatrix<float>& M);

void
loadSparseMatrix (const std::string& filename,
                  Eigen::SparseMatrix<float>& M);

#include "impl/io.hpp"

#endif /* IO_H */

