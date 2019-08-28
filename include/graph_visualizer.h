#ifndef GRAPH_VISUALIZER_H
#define GRAPH_VISUALIZER_H

#include <vtkLine.h>
#include <vtkPolyLine.h>
#include <vtkPolyData.h>
#include <vtkCellData.h>
#include <vtkCellArray.h>
#include <vtkSmartPointer.h>

#include "scaler.h"
#include "as_range.h"
#include "tviewer/color.h"
#include "tviewer/visualization_objects/arrow_array_object.h"

#include "graph/point_cloud_graph.h"

template <typename GraphT>
class GraphVisualizer
{

  public:

    typedef typename pcl::PointCloud<pcl::PointXYZRGBA> PointCloudT;
    typedef typename pcl::PointCloud<pcl::PointNormal> NormalCloudT;

    typedef typename boost::graph_traits<GraphT>::vertex_iterator VertexIterator;
    typedef typename boost::graph_traits<GraphT>::edge_iterator EdgeIterator;

    GraphVisualizer (const GraphT& g)
    : graph_ (g)
    {
    }

    tviewer::ArrowsPtr
    getEdgesArrowArray ()
    {
      tviewer::ArrowsPtr arrows (new tviewer::Arrows);
      EdgeIterator s, e;
      auto scale = getRangeScalingForEdges ();
      for (boost::tie (s, e) = boost::edges (graph_); s != e; ++s)
      {
        auto u = boost::source (*s, graph_);
        auto v = boost::target (*s, graph_);
        auto w = boost::get (boost::edge_weight, graph_, *s);
        arrows->push_back ({ graph_[u].getVector3fMap (),
                             graph_[v].getVector3fMap (),
                             tviewer::getColor (scale (w))});
      }
      return arrows;
    }

    vtkSmartPointer<vtkPolyData>
    getEdgesPolyData ()
    {
      vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
      vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New ();
      vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
      vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();
      colors->SetNumberOfComponents (3);
      unsigned char c[3];
      EdgeIterator s, e;
      auto scale = getRangeScalingForEdges ();
      int id = 0;
      for (boost::tie (s, e) = boost::edges (graph_); s != e; ++s)
      {
        auto u = boost::source (*s, graph_);
        auto v = boost::target (*s, graph_);
        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New ();
        points->InsertNextPoint (graph_[u].getVector3fMap ().data ());
        points->InsertNextPoint (graph_[v].getVector3fMap ().data ());
        line->GetPointIds ()->SetId (0, id++);
        line->GetPointIds ()->SetId (1, id++);
        cells->InsertNextCell (line);
        tviewer::getRGBFromColor (tviewer::getColor (scale (boost::get (boost::edge_weight_t (), graph_, *s))), c);
        colors->InsertNextTupleValue (c);
      }
      polydata->SetPoints (points);
      polydata->SetLines (cells);
      polydata->GetCellData ()->SetScalars (colors);
      return polydata;
    }

    typename NormalCloudT::Ptr
    getVerticesNormalsCloud ()
    {
      NormalCloudT::Ptr cloud (new NormalCloudT);
      copyPointCloud (*pcl::graph::point_cloud (graph_), *pcl::graph::indices (graph_), *cloud);
      return cloud;
    }

    typename PointCloudT::Ptr
    getVerticesCloudColorsNatural ()
    {
      return constructVerticesCloud (MODE_NATURAL);
    }

    typename PointCloudT::Ptr
    getVerticesCloudColorsCurvature ()
    {
      return constructVerticesCloud (MODE_CURVATURE);
    }

    typename PointCloudT::Ptr
    getVerticesCloudColorsDegree ()
    {
      Eigen::VectorXf degrees = Eigen::VectorXf::Zero (boost::num_vertices (graph_));
      for (const auto& edge : as_range (boost::edges (graph_)))
      {
        auto src = boost::source (edge, graph_);
        auto tgt = boost::target (edge, graph_);
        auto degree = boost::get (boost::edge_weight_t (), graph_, edge);
        degrees (src) += degree;
        degrees (tgt) += degree;
      }
      return constructVerticesCloud (degrees);
    }

    typename PointCloudT::Ptr
    getVerticesCloudColorsFromMapRandom ()
    {
      return constructVerticesCloud (boost::get (boost::vertex_color, graph_), MODE_RANDOM);
    }

    template <typename VertexColorMap> typename PointCloudT::Ptr
    getVerticesCloudColorsFromMapRandom (VertexColorMap map)
    {
      return constructVerticesCloud (map, MODE_RANDOM);
    }

    typename PointCloudT::Ptr
    getVerticesCloudColorsFromMapFixed ()
    {
      return constructVerticesCloud (boost::get (boost::vertex_color, graph_), MODE_FIXED);
    }

    template <typename VertexColorMap> typename PointCloudT::Ptr
    getVerticesCloudColorsFromMapFixed (VertexColorMap map)
    {
      return constructVerticesCloud (map, MODE_FIXED);
    }

    typename PointCloudT::Ptr
    getVerticesCloudColorsFromMapPersistent ()
    {
      return constructVerticesCloud (boost::get (boost::vertex_color, graph_), MODE_PERSISTENT);
    }

    template <typename VertexColorMap> typename PointCloudT::Ptr
    getVerticesCloudColorsFromMapPersistent (VertexColorMap map)
    {
      return constructVerticesCloud (map, MODE_PERSISTENT);
    }

    typename PointCloudT::Ptr
    getVerticesCloudColorsFromVector (const Eigen::VectorXf& colors)
    {
      return constructVerticesCloud (colors);
    }

  private:

    enum ColorMode

    {
      MODE_NATURAL,
      MODE_CURVATURE,
      MODE_RANDOM,
      MODE_FIXED,
      MODE_PERSISTENT,
    };

    typename PointCloudT::Ptr
    constructVerticesCloud (ColorMode mode)
    {
      PointCloudT::Ptr cloud (new PointCloudT);
      copyPointCloud (*pcl::graph::point_cloud (graph_), *pcl::graph::indices (graph_), *cloud);
      if (mode == MODE_CURVATURE)
        for (const auto& s : as_range (boost::vertices (graph_)))
          // The curvature produced by PCAG is signed.
          // Empirically the absolute value of the curvature is below 0.25,
          // so it is safe (i.e. we end up in 0...1 region) to add 0.5.
          cloud->at (s).rgba = tviewer::getColor (graph_[s].curvature + 0.5);
      return cloud;
    }

    typename PointCloudT::Ptr
    constructVerticesCloud (const Eigen::VectorXf& colors)
    {
      assert (static_cast<int> (boost::num_vertices (graph_)) == colors.size ());
      PointCloudT::Ptr cloud (new PointCloudT);
      copyPointCloud (*pcl::graph::point_cloud (graph_), *pcl::graph::indices (graph_), *cloud);
      auto scale = getScaler (colors);
      for (const auto& s : as_range (boost::vertices (graph_)))
      {
        cloud->at (s).rgba = tviewer::getColor (scale (colors[s]));
      }
      return cloud;
    }

    template <typename VertexColorMap> typename PointCloudT::Ptr
    constructVerticesCloud (VertexColorMap colors, ColorMode mode)
    {
      PointCloudT::Ptr cloud (new PointCloudT);
      copyPointCloud (*pcl::graph::point_cloud (graph_), *pcl::graph::indices (graph_), *cloud);
      std::map<uint32_t, tviewer::Color> colormap;
      const std::vector<tviewer::Color> COLORS = { 0x9E9E9E, 0x29CC00, 0x008FCC, 0xA300CC, 0xCC3D00, 0xFFDD00, 0x63E6E6, 0xA5E663, 0x9E2B2B };
      size_t c = 0;
      for (const auto& s : as_range (boost::vertices (graph_)))
      {
        auto color_id = colors[s];
        switch (mode)
        {
          case MODE_RANDOM:
            {
              if (!colormap.count (color_id))
                colormap[color_id] = tviewer::generateRandomColor ();
              cloud->at (s).rgba = colormap[color_id];
              break;
            }
          case MODE_FIXED:
            {
              if (!colormap.count (color_id))
                colormap[color_id] = COLORS[c++];
              cloud->at (s).rgba = colormap[color_id];
              break;
            }
          case MODE_PERSISTENT:
            {
              if (!colormap_.count (color_id))
                colormap_[color_id] = tviewer::generateRandomColor ();
              cloud->at (s).rgba = colormap_[color_id];
              break;
            }
          default:
            break;
        }
      }
      return cloud;
    }

    std::function<float (float)> getRangeScalingForEdges ()
    {
      float max = -std::numeric_limits<float>::infinity();
      float min = std::numeric_limits<float>::infinity();
      for (const auto& s : as_range (boost::edges (graph_)))
      {
        auto v = boost::get (boost::edge_weight_t (), graph_, s);
        //v = std::log (v);
        if (v > max) max = v;
        if (v < min) min = v;
      }
      if (max != min)
        //return [min, max] (float v) { return (std::log (v) - min) / (max - min); };
        return [min, max] (float v) { return (v - min) / (max - min); };
      else
        return [] (float) { return 1.0; };
    }

    const GraphT& graph_;
    std::map<uint32_t, tviewer::Color> colormap_;

};

#endif /* GRAPH_VISUALIZER_H */

