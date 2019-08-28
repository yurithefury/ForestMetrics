#ifndef SEED_SELECTION_H
#define SEED_SELECTION_H

#include <set>

#include <QAbstractListModel>
#include <QItemSelection>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class SeedSelection : public QAbstractListModel
{

    Q_OBJECT

  public:

    typedef boost::shared_ptr<SeedSelection> Ptr;

    typedef pcl::PointCloud<pcl::PointXYZRGB> ColoredPointCloud;
    typedef ColoredPointCloud::Ptr ColoredPointCloudPtr;

    typedef pcl::PointCloud<pcl::PointXYZL> LabeledPointCloud;
    typedef LabeledPointCloud::Ptr LabeledPointCloudPtr;

    SeedSelection (QObject* parent = 0);

    ~SeedSelection ();

    virtual int
    rowCount (const QModelIndex&) const;

    virtual QVariant
    data (const QModelIndex& index, int role) const;

    void
    pickPoint (const pcl::PointXYZ& p);

    QModelIndex
    addNewLabel ();

    void
    deleteLabel ();

    LabeledPointCloudPtr
    getSelectedSeeds ()
    {
      return seeds_cloud_;
    }

    void
    setSeeds (const LabeledPointCloud& seeds);

    ColoredPointCloudPtr
    getPointCloudForVisualization ();

    uint32_t getFirstCurrentLabel () const
    {
      return current_labels_.size () ? *current_labels_.cbegin () : 0;
    }

    uint32_t getLastCurrentLabel () const
    {
      return current_labels_.size () ? *current_labels_.crbegin () : num_labels_;
    }

    const std::set<uint32_t> getCurrentLabels () const
    {
      return current_labels_;
    }

    size_t getNumLabels () const
    {
      return num_labels_;
    }

  public Q_SLOTS:

    void
    currentChanged (const QItemSelection& current, const QItemSelection& previous);

  Q_SIGNALS:

    /** This signal is triggered when the seeds cloud is changed.
      *
      * This includes addition/deletion of labels/seeds, as well as changes of
      * visualization colors when the user selects a label. */
    void
    seedsChanged ();

  private:

    size_t
    countSeedsWithLabel (uint32_t label) const;

    LabeledPointCloudPtr seeds_cloud_;

    size_t num_labels_;

    std::set<uint32_t> current_labels_;

};

#endif /* SEED_SELECTION_H */

