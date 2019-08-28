#ifndef GUI_DELINEATION_SEED_SELECTION_FORM_H
#define GUI_DELINEATION_SEED_SELECTION_FORM_H

#include <QWidget>

#ifndef Q_MOC_RUN
#include "types.h"
#include "pipeline.h"
#endif

namespace Ui
{
  class SeedSelectionForm;
}

class SeedSelection;

class SeedSelectionForm : public QWidget, public pipeline::Stage
{

    Q_OBJECT

  public:

    explicit SeedSelectionForm (QWidget* parent = 0);

    ~SeedSelectionForm ();

    virtual void
    enter () override;

    virtual void
    leave () override;

  public Q_SLOTS:

    void
    onButtonNewLabelClicked ();

  private:

    pipeline::Input<Graph> input_graph_ = "Graph";
    pipeline::Input<PointCloudLabel> input_seeds_ = "LoadedSeeds";
    pipeline::Input<std::function<bool (const Eigen::Vector3f&)>> filter_predicate_ = "FilterPredicate";
    pipeline::Output<PointCloudLabel> output_seeds_ = "SelectedSeeds";

    Ui::SeedSelectionForm *ui_;

    std::unique_ptr<SeedSelection> seed_selection_;

    PointCloud::Ptr vertices_;
    PointCloudLabel::Ptr seeds_;

};

#endif // GUI_DELINEATION_SEED_SELECTION_FORM_H
