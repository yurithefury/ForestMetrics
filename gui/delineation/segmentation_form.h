#ifndef GUI_DELINEATION_SEGMENTATION_FORM_H
#define GUI_DELINEATION_SEGMENTATION_FORM_H

#include <QWidget>

#ifndef Q_MOC_RUN
#include "types.h"
#include "pipeline.h"
#endif

#include "cluster_list_model.h"

namespace Ui
{
  class SegmentationForm;
}

class SegmentationForm : public QWidget, public pipeline::Stage
{

    Q_OBJECT

  public:

    explicit SegmentationForm (QWidget* parent = 0);

    ~SegmentationForm ();

    virtual void
    enter () override;

    virtual void
    leave () override;

    virtual void
    loadConfig (Config& config) override;

    virtual void
    saveConfig (Config& config) const override;

  public Q_SLOTS:

    void
    onButtonSegmentClicked ();

    void
    onClusterSelectionChanged ();

    void
    onPointPicked (QString name, int index);

  private:

    pipeline::Input<Graph> input_graph_ = "Graph";
    pipeline::Input<PointCloudLabel> input_seeds_ = "Seeds";
    pipeline::Output<PointCloudLabel> output_segmentation_ = "Segmentation";

    Ui::SegmentationForm *ui_;

    RandomWalkerSegmentation rws_;

    PointCloudLabel::Ptr seeds_;

    ClusterListModel::Ptr cluster_list_;

};

#endif // GUI_DELINEATION_SEGMENTATION_FORM_H
