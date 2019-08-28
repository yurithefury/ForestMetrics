#ifndef GUI_DELINEATION_TRUNK_DETECTION_FORM_H
#define GUI_DELINEATION_TRUNK_DETECTION_FORM_H

#include <QWidget>

#ifndef Q_MOC_RUN
#include "types.h"
#include "pipeline.h"
#endif

#include "cluster_list_model.h"

namespace Ui
{
  class TrunkDetectionForm;
}

class TrunkDetectionForm : public QWidget, public pipeline::Stage
{

    Q_OBJECT

  public:

    explicit TrunkDetectionForm (QWidget *parent = 0);

    ~TrunkDetectionForm ();

    virtual void
    loadConfig (Config& config) override;

    virtual void
    saveConfig (Config& config) const override;

    virtual void
    enter () override;

    virtual void
    leave () override;

  public Q_SLOTS:

    void
    onHeightFilterChanged ();

    void
    onButtonDetectClicked ();

    void
    onButtonFitClicked ();

    void
    onButtonEnrichClicked ();

    void
    onButtonUseAsSeedsClicked ();

    void
    onClusterSelectionChanged ();

  private:

    bool
    euclideanClusteringCondition (const Point& a, const Point& b, float dist);

    void
    updateTreeSeeds ();

    void
    performBottomUpTreeDetection ();

    void
    performTopDownTreeDetection ();

    pipeline::Input<PointCloud> input_cloud_ = "LoadedCloud";
    pipeline::Input<std::vector<int>> input_indices_ = "FilteredIndices";
    pipeline::Output<PointCloudLabel> output_seeds_ = "Seeds";

    Ui::TrunkDetectionForm *ui_;

    PointCloud::Ptr input_;
    PointCloudLabel::Ptr tree_seeds_;
    std::vector<pcl::PointIndices> tree_cluster_indices_;
    std::vector<Eigen::VectorXf> tree_line_coefficients_;

    pcl::IndicesPtr filtered_indices_;

    ClusterListModel::Ptr cluster_list_;

};

#endif // GUI_DELINEATION_TRUNK_DETECTION_FORM_H
