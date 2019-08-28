#include <vtkCellData.h>
#include <vtkPlaneSource.h>

#include "tviewer/color.h"

#include "scaler.h"
#include "mesh_grid.h"

MeshGrid::MeshGrid (float min_x, float max_x, float min_y, float max_y, float cell_size)
: MeshGrid (min_x, max_x, min_y, max_y, cell_size, cell_size)
{
}

MeshGrid::MeshGrid (float min_x, float max_x, float min_y, float max_y, float cell_size_x, float cell_size_y)
: min_x_ (min_x)
, max_x_ (max_x)
, min_y_ (min_y)
, max_y_ (max_y)
, cell_size_x_ (cell_size_x)
, cell_size_y_ (cell_size_y)
, num_cells_x_ (ceil ((max_x_ - min_x_) / cell_size_x_))
, num_cells_y_ (ceil ((max_y_ - min_y_) / cell_size_y_))
{
  assert (max_x_ > min_x_);
  assert (max_y_ > min_y_);
  assert (cell_size_x_ > 0);
  assert (cell_size_y_ > 0);
}

std::vector<float>
MeshGrid::getCellCenters ()
{
  std::vector<float> centers;
  centers.reserve (num_cells_x_ * num_cells_y_ * 2);
  for (size_t i = 0; i < num_cells_y_; ++i)
  {
    float y = min_y_ + i * cell_size_y_ + cell_size_y_ / 2;
    for (size_t j = 0; j < num_cells_x_; ++j)
    {
      float x = min_x_ + j * cell_size_x_ + cell_size_x_ / 2;
      centers.push_back (x);
      centers.push_back (y);
    }
  }
  return centers;
}

vtkSmartPointer<vtkPolyData>
MeshGrid::createPolyData (float z)
{
  auto plane = vtkSmartPointer<vtkPlaneSource>::New ();
  plane->SetOrigin (min_x_, min_y_, z);
  plane->SetPoint1 (max_x_, min_y_, z);
  plane->SetPoint2 (min_x_, max_y_, z);
  plane->SetResolution (num_cells_x_, num_cells_y_);
  plane->Update ();
  return plane->GetOutput ();
}

vtkSmartPointer<vtkPolyData>
MeshGrid::createPolyData (std::vector<float> colors, float z)
{
  auto poly_data = createPolyData (z);
  assert (poly_data->GetNumberOfCells () == colors.size ());
  auto scale = getScaler (colors);
  vtkSmartPointer<vtkUnsignedCharArray> cell_data = vtkSmartPointer<vtkUnsignedCharArray>::New ();
  cell_data->SetNumberOfComponents (3);
  cell_data->SetNumberOfTuples (colors.size ());
  for (size_t i = 0; i < colors.size (); ++i)
  {
    unsigned char c[3];
    auto v = scale (colors[i]);
    tviewer::getRGBFromColor (tviewer::getColor (v), c);
    float rgb[3];
    rgb[0] = c[0];
    rgb[1] = c[1];
    rgb[2] = c[2];
    cell_data->InsertTuple (i, rgb);
  }
  poly_data->GetCellData ()->SetScalars (cell_data);
  return poly_data;
}

