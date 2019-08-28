#include <ctime>

#ifndef Q_MOC_RUN
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#endif

#include <QModelIndex>
#include <QShortcut>

#include <vtkLine.h>
#include <vtkPolyLine.h>
#include <vtkPolyData.h>
#include <vtkCellData.h>
#include <vtkCellArray.h>
#include <vtkSmartPointer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>

#include "tviewer/color.h"

#include "main_window.h"
#include "ui_main_window.h"
#include "status_bar.h"

#include "io.h"

using namespace tviewer;
using namespace pipeline;

MainWindow::MainWindow (const std::string& filename, QWidget* parent)
: QMainWindow (parent)
, ui_ (new Ui::MainWindow)
, graph_ (new Graph)
{
  srand (time (0));

  ui_->setupUi (this);
  ui_->menu_bar->addMenu (ui_->viewer->getMenu ("View"));

  // Setup status bar
  status_bar_.reset (new StatusBar (ui_->status_bar));

  // Initialize pipeline
  Pipeline::getInstance ().createStage ("Preprocessing", ui_->page_preprocessing);
  Pipeline::getInstance ().createStage ("TrunkDetection", ui_->page_trunk_detection);
  Pipeline::getInstance ().createStage ("GraphBuilding", ui_->page_graph_building);
  Pipeline::getInstance ().createStage ("Segmentation", ui_->page_segmentation);

  Pipeline::getInstance ().setViewer (ui_->viewer);
  Pipeline::getInstance ().setStatusBar (status_bar_);

  Pipeline::getInstance ().registerDataChangedCallback (boost::bind (&MainWindow::onDataChanged, this, _1));

  //ui_->list_labels->setModel (seed_selection_.get ());

  //connect (ui_->list_labels->selectionModel (),
           //SIGNAL (selectionChanged (QItemSelection, QItemSelection)),
           //seed_selection_.get (),
           //SLOT (currentChanged (QItemSelection, QItemSelection)));

  //connect (seed_selection_.get (),
           //SIGNAL (seedsChanged ()),
           //this,
           //SLOT (seedsChanged ()));

  QShortcut* shortcut_key_up = new QShortcut (QKeySequence (Qt::Key_Up), this);
  shortcut_key_up->setContext (Qt::ApplicationShortcut);
  connect (shortcut_key_up,
           SIGNAL (activated ()),
           this,
           SLOT (onKeyUp ()));

  QShortcut* shortcut_key_down = new QShortcut (QKeySequence (Qt::Key_Down), this);
  shortcut_key_down->setContext (Qt::ApplicationShortcut);
  connect (shortcut_key_down,
           SIGNAL (activated ()),
           this,
           SLOT (onKeyDown ()));

  //QModelIndex index = seed_selection_->addNewLabel ();
  //ui_->list_labels->selectionModel ()->select (index, QItemSelectionModel::ClearAndSelect);

  loadConfig ();

  loadData (filename);

  onButtonUpdateGraphClicked ();

  activateStage ("Preprocessing");
}

MainWindow::~MainWindow ()
{
  saveConfig ();
  delete ui_;
}

void
MainWindow::onCurrentToolboxPageChanged (int index)
{
  Pipeline::getInstance ().activate (index);
}

void
MainWindow::onActionSaveSeedsTriggered ()
{
  std::string filename = QFileDialog::getSaveFileName (this, tr ("Save tree trunk seeds to"), "seeds.pcd", tr ("PCD file (*.pcd)")).toStdString ();
  auto seeds = Pipeline::getInstance ().getHub<PointCloudLabel> ("Seeds").get ();
  if (seeds && filename != "")
  {
    pcl::io::savePCDFile (filename, *seeds);
    status_bar_->showMessage ("Saved tree trunk seeds");
  }
}

void
MainWindow::onActionSaveSegmentationTriggered ()
{
  std::string filename = QFileDialog::getSaveFileName (this, tr ("Save segmented point cloud to"), "segmentation.pcd", tr ("PCD file (*.pcd)")).toStdString ();
  auto segmentation = Pipeline::getInstance ().getHub<PointCloudLabel> ("Segmentation").get ();
  if (segmentation && filename != "")
  {
    pcl::io::savePCDFile (filename, *segmentation);
    status_bar_->showMessage ("Saved segmented point cloud");
  }
}

void
MainWindow::seedsChanged ()
{
}

void
MainWindow::onKeyUp ()
{
}

void
MainWindow::onKeyDown ()
{
}

void
MainWindow::onButtonUpdateGraphClicked ()
{
}

void
MainWindow::buttonNewLabelClicked ()
{
}

void
MainWindow::buttonDeleteLabelClicked ()
{
}

void
MainWindow::buttonSegmentClicked ()
{
}

bool
MainWindow::loadSeeds (const std::string& filename)
{
  if (boost::filesystem::exists (filename))
  {
    PointCloudLabel::Ptr seeds (new PointCloudLabel);
    pcl::io::loadPCDFile<pcl::PointXYZL> (filename, *seeds);
    loaded_seeds_ = seeds;
    status_bar_->showMessage ("Loaded %s seeds", seeds->size ());
    return true;
  }
  return false;
}

bool
MainWindow::loadData (const std::string& filename)
{
  if (boost::filesystem::exists (filename))
  {
    cloud_.reset (new PointCloud);
    if (pcl::io::loadPCDFile (filename, *cloud_))
      throw std::runtime_error ("failed to load input point cloud");
    if (!hasColor (filename))
      for (auto& point : *cloud_)
        point.rgba = 0xFFF000;
    loaded_cloud_ = cloud_;
    ui_->label_dataset_name_value->setText (QString (boost::filesystem::path (filename).filename ().c_str ()));
    ui_->label_num_points_value->setText (QString::number (cloud_->size ()));
    return true;
  }
  return false;
}


void
MainWindow::displayGraphVertices ()
{
}

void
MainWindow::displayGraphEdges (uint32_t color)
{
}

void
MainWindow::displaySeeds ()
{
}

void
MainWindow::saveConfig ()
{
  Config config;
  Pipeline::getInstance ().saveConfigs (config);
  config.save ("config.json");
}

void
MainWindow::loadConfig ()
{
  try
  {
    Config config ("config.json");
    Pipeline::getInstance ().loadConfigs (config);
  }
  catch (std::runtime_error& e)
  {
    // Cannot load config file, do nothing
    return;
  }
}

void
MainWindow::activateStage (const std::string& name)
{
  int index = Pipeline::getInstance ().activate (name);
  if (ui_->toolbox->currentIndex () != index)
    ui_->toolbox->setCurrentIndex (index);
}

void
MainWindow::onDataChanged (const std::string& hub_name)
{
  // Graph
  if (hub_name == "Graph")
  {
    auto graph = Pipeline::getInstance ().getHub<Graph> ("Graph").get ();
    auto vertices = boost::num_vertices (*graph);
    auto edges = boost::num_edges (*graph);
    auto text = QString ("%1 vertices and %2 edges").arg (vertices).arg (edges);
    ui_->label_graph_value->setText (text);
  }
  else if (hub_name == "FilteredIndices")
  {
    auto indices = Pipeline::getInstance ().getHub<std::vector<int>> ("FilteredIndices").get ();
    if (indices)
      ui_->label_num_points_value->setText (QString::number (indices->size ()));
    else
      ui_->label_num_points_value->setText (QString::number (cloud_->size ()));
  }
  else if (hub_name == "Seeds")
  {
    auto seeds = Pipeline::getInstance ().getHub<PointCloudLabel> ("Seeds").get ();
    std::set<uint32_t> labels;
    for (const auto& p : seeds->points)
      labels.insert (p.label);
    auto text = QString ("%1 seeds (%2 unique labels)").arg (seeds->size ()).arg (labels.size ());
    ui_->label_seeds_value->setText (text);
    ui_->action_save_seeds->setEnabled (true);
  }
  else if (hub_name == "Segmentation")
  {
    ui_->action_save_segmentation->setEnabled (true);
  }
}

