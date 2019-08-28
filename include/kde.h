#ifndef KDE_H
#define KDE_H

#include <memory>

#include <flann/flann.hpp>

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

class KDE
{

  public:

    KDE (std::vector<float>&& dataset);

    void
    setBandwidth (float bandwidth);

    float
    evaluateAt (float x, float y) const;

    std::vector<float>
    evaluateAt (std::vector<float>& query) const;

  private:

    using KdTree = flann::Index<flann::L2<float>>;

    std::unique_ptr<KdTree> kdtree_;
    std::vector<float> dataset_;
    std::function<float (float)> kernel_;
    float bandwidth_;
    float radius_;

};

#endif /* KDE_H */

