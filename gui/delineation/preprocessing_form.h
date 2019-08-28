#ifndef GUI_DELINEATION_PREPROCESSING_FORM_H
#define GUI_DELINEATION_PREPROCESSING_FORM_H

#include <QWidget>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#ifndef Q_MOC_RUN
#include "types.h"
#include "pipeline.h"
#endif

namespace Ui
{
  class PreprocessingForm;
}

class PreprocessingForm : public QWidget, public pipeline::Stage
{

    Q_OBJECT

  public:

    explicit PreprocessingForm (QWidget* parent = 0);

    virtual ~PreprocessingForm ();

    virtual void
    enter () override;

    virtual void
    loadConfig (Config& config) override;

    virtual void
    saveConfig (Config& config) const override;

  public Q_SLOTS:

    void
    onPassthroughFilterChanged ();

    void
    onButtonApplyClicked ();

    void
    onButtonResetClicked ();

  private:

    pipeline::Input<PointCloud> input_cloud_ = "LoadedCloud";
    pipeline::Output<std::vector<int>> output_indices_ = "FilteredIndices";
    pipeline::Output<std::function<bool (const Eigen::Vector3f&)>> output_predicate_ = "FilterPredicate";

    Ui::PreprocessingForm* ui_;

    PointCloud::Ptr display_;

};

#endif // GUI_DELINEATION_PREPROCESSING_FORM_H
