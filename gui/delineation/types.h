#ifndef GUI_DELINEATION_TYPES_H
#define GUI_DELINEATION_TYPES_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "random_walker_segmentation.h"

using Point = pcl::PointXYZRGB;
using PointLabel = pcl::PointXYZL;
using PointWithNormal = pcl::PointXYZRGBNormal;
using PointCloud = pcl::PointCloud<Point>;
using PointCloudLabel = pcl::PointCloud<PointLabel>;
using PointCloudWithNormal = pcl::PointCloud<PointWithNormal>;

using RandomWalkerSegmentation = pcl::segmentation::RandomWalkerSegmentation<Point>;
using Graph = RandomWalkerSegmentation::Graph;
using GraphPtr = RandomWalkerSegmentation::GraphPtr;
using GraphConstPtr = RandomWalkerSegmentation::GraphConstPtr;

#endif /* GUI_DELINEATION_TYPES_H */

