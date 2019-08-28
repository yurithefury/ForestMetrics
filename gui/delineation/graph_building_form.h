#ifndef GUI_DELINEATION_GRAPH_BUILDING_FORM_H
#define GUI_DELINEATION_GRAPH_BUILDING_FORM_H

#include <QWidget>

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#ifndef Q_MOC_RUN
#include "types.h"
#include "pipeline.h"
#endif

namespace Ui
{
  class GraphBuildingForm;
}

class GraphBuildingForm : public QWidget, public pipeline::Stage
{

    Q_OBJECT

  public:

    explicit GraphBuildingForm (QWidget* parent = 0);

    ~GraphBuildingForm ();

    virtual void
    loadConfig (Config& config) override;

    virtual void
    saveConfig (Config& config) const override;

    virtual void
    enter () override;

    virtual void
    leave () override;

    void
    execute ();

  public Q_SLOTS:

    void
    onGraphBuilderEditingFinished ();

    void
    onEdgeWeightsEditingFinished ();

    void
    onUpdateButtonClicked ();

  private:

    void
    buildGraph ();

    void
    computeEdgeWeights ();

    pipeline::Input<PointCloud> input_cloud_ = "LoadedCloud";
    pipeline::Input<std::vector<int>> input_indices_ = "FilteredIndices";
    pipeline::Output<Graph> output_graph_ = "Graph";

    Ui::GraphBuildingForm* ui_;

    GraphPtr graph_;

    PointCloud::Ptr vertices_;

    vtkSmartPointer<vtkPolyData> edges_;

};

#endif // GUI_DELINEATION_GRAPH_BUILDING_FORM_H
