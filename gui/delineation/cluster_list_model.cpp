#include <QColor>
#include <QBrush>

#include <pcl/common/io.h>
#include <pcl/common/utils.h>
#include <pcl/common/colors.h>
#include <pcl/console/print.h>

#include "cluster_list_model.h"

ClusterListModel::ClusterListModel (QObject* parent)
: QAbstractListModel (parent)
, cloud_ (new PointCloud)
{
}

ClusterListModel::~ClusterListModel ()
{
}

int
ClusterListModel::rowCount (const QModelIndex&) const
{
  return cluster_indices_.size ();
}

QVariant
ClusterListModel::data (const QModelIndex& index, int role) const
{
  if (role == Qt::DisplayRole)
  {
    return QString ("%1").arg (cluster_indices_[index.row ()].indices.size ());
  }
  if (role == Qt::BackgroundRole)
  {
    auto c = pcl::GlasbeyLUT::at (index.row () % pcl::GlasbeyLUT::size ());
    return QBrush (QColor (c.r, c.g, c.b));
  }
  return QVariant ();
}

QModelIndex
ClusterListModel::getIndexFromPointIndex (int index)
{
  for (size_t i = 0; i < cluster_indices_.size (); ++i)
  {
    index -= static_cast<int> (cluster_indices_[i].indices.size ());
    if (index < 0)
      return createIndex (i, 0);
  }
  return QModelIndex ();
}

void
ClusterListModel::setClusterList (const PointCloud::ConstPtr& cloud,
                                  const std::vector<pcl::PointIndices>& cluster_indices)
{
  auto old_size = cluster_indices_.size ();
  auto new_size = cluster_indices.size ();
  if (new_size > old_size)
  {
    beginInsertRows (QModelIndex (), old_size, new_size - 1);
    endInsertRows ();
  }
  else if (new_size < old_size)
  {
    beginRemoveRows (QModelIndex (), new_size, old_size - 1);
    endRemoveRows ();
  }
  cloud_ = cloud;
  cluster_indices_ = cluster_indices;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
ClusterListModel::getPointCloudForVisualization ()
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  for (size_t i = 0; i < cluster_indices_.size (); ++i)
  {
    uint8_t alpha = 0xFF;
    if (current_selection_.size () != 0 && current_selection_.count (i + 1) == 0)
      alpha = 0x3F;
    for (const auto& index : cluster_indices_[i].indices)
    {
      pcl::PointXYZRGBA p;
      pcl::copyPoint (cloud_->at (index), p);
      p.rgba = pcl::GlasbeyLUT::at (i % pcl::GlasbeyLUT::size ()).rgba;
      p.a = alpha;
      cloud->push_back (p);
    }
  }
  return cloud;
}

void
ClusterListModel::currentChanged (const QItemSelection& current, const QItemSelection& previous)
{
  for (size_t i = 0; i < current.indexes ().size (); ++i)
    current_selection_.insert (current.indexes ().at (i).row () + 1);
  for (size_t i = 0; i < previous.indexes ().size (); ++i)
    current_selection_.erase (previous.indexes ().at (i).row () + 1);
  selectionChanged ();
}

