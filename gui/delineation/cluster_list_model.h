#ifndef CLUSTER_LIST_MODEL_H
#define CLUSTER_LIST_MODEL_H

#include <set>

#include <QItemSelection>
#include <QAbstractListModel>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#ifndef Q_MOC_RUN
#include "types.h"
#endif

class ClusterListModel : public QAbstractListModel
{

    Q_OBJECT

  public:

    typedef boost::shared_ptr<ClusterListModel> Ptr;

    ClusterListModel (QObject* parent = 0);

    ~ClusterListModel ();

    virtual int
    rowCount (const QModelIndex&) const override;

    virtual QVariant
    data (const QModelIndex& index, int role) const override;

    QModelIndex
    getIndexFromPointIndex (int index);

    void
    setClusterList (const PointCloud::ConstPtr& cloud,
                    const std::vector<pcl::PointIndices>& cluster_indices);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
    getPointCloudForVisualization ();

  public Q_SLOTS:

    void
    currentChanged (const QItemSelection& current, const QItemSelection& previous);

  Q_SIGNALS:

    //void
    //pointPicked ();

    void
    selectionChanged ();

  private:

    PointCloud::ConstPtr cloud_;

    std::vector<pcl::PointIndices> cluster_indices_;

    std::set<uint32_t> current_selection_;

};

#endif /* CLUSTER_LIST_MODEL_H */

