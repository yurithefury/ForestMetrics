#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <map>

#include <QMainWindow>
#include <QFileDialog>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#ifndef Q_MOC_RUN
#include "types.h"
#include "pipeline.h"
#include "random_walker_segmentation.h"
#endif

namespace Ui
{
  class MainWindow;
}

class PipelineStage;
class Visualizer;
class StatusBar;

class MainWindow : public QMainWindow
{
    Q_OBJECT

  public:

    MainWindow (const std::string& filename, QWidget* parent = 0);

    ~MainWindow ();

  public Q_SLOTS:

    void
    onCurrentToolboxPageChanged (int index);

    void
    onButtonUpdateGraphClicked ();

    void
    buttonNewLabelClicked ();

    void
    buttonDeleteLabelClicked ();

    void
    buttonSegmentClicked ();

    void
    checkboxDisplayStateChanged (int state)
    {
      displayGraphVertices ();
      displayGraphEdges ();
    }

    void
    onActionExitTriggered ()
    {
      this->close ();
    }

    inline void
    onActionLoadSeedsTriggered ()
    {
      loadSeeds (QFileDialog::getOpenFileName ().toStdString ());
    }

    void
    onActionSaveSeedsTriggered ();

    void
    onActionSaveSegmentationTriggered ();

    void
    seedsChanged ();

    void
    onKeyUp ();

    void
    onKeyDown ();

    bool
    loadData (const std::string& filename);

    bool
    loadSeeds (const std::string& filename);

  private:

    void
    displayGraphVertices ();

    void
    displayGraphEdges (uint32_t color = 0);

    void
    displaySeeds ();

    void
    saveConfig ();

    void
    loadConfig ();

    void
    activateStage (const std::string& name);

    void
    onDataChanged (const std::string& hub_name);

    Ui::MainWindow* ui_;

    std::shared_ptr<StatusBar> status_bar_;

    PointCloud::Ptr cloud_;
    GraphPtr graph_;
    std::map<uint32_t, uint32_t> colormap_;

    pipeline::Output<PointCloud> loaded_cloud_ = "LoadedCloud";
    pipeline::Output<PointCloudLabel> loaded_seeds_ = "Seeds";

};

#endif // MAIN_WINDOW_H
