#ifndef MESH_GRID_H
#define MESH_GRID_H

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

class MeshGrid
{

  public:

    MeshGrid (float min_x, float max_x, float min_y, float max_y, float cell_size);

    MeshGrid (float min_x, float max_x, float min_y, float max_y, float cell_size_x, float cell_size_y);

    std::vector<float>
    getCellCenters ();

    vtkSmartPointer<vtkPolyData>
    createPolyData (float z = 0.0);

    vtkSmartPointer<vtkPolyData>
    createPolyData (std::vector<float> colors, float z = 0.0);

  private:

    float min_x_;
    float max_x_;
    float min_y_;
    float max_y_;
    float cell_size_x_;
    float cell_size_y_;
    size_t num_cells_x_;
    size_t num_cells_y_;

};

#endif /* MESH_GRID_H */

