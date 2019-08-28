#pragma once

#include <Eigen/Core>

#include "types.h"

class TreeTopDetector
{

  public:

    TreeTopDetector ();

    void
    setInputCloud (typename PointCloud::ConstPtr cloud);

    void
    setRadius (float radius);

    virtual ~TreeTopDetector() { }

    void
    detect (std::vector<pcl::PointIndices>& tree_top_indices);

    const std::vector<float>&
    getDensity () const;

  private:

    PointCloud::ConstPtr input_;
    float cell_size_;
    std::vector<float> density_;

    struct Candidate
    {
      int cell_x, cell_y;
      int index;
      float z;
    };

    std::queue<Candidate> candidates_;

    float radius_;
    int cell_radius_;

};
