#ifndef CLOUDLIST_H
#define CLOUDLIST_H

#include <vector>
#include <mutex>
#include <memory>
#include <QAbstractListModel>
#include <model/pointcloud.h>
#include "model/export.h"

class QUndoStack;
class QItemSelection;

class MODEL_API CloudList : public QAbstractListModel {
    Q_OBJECT
 public:
    explicit CloudList(QObject *parent = 0);
    ~CloudList();
    Qt::ItemFlags flags(const QModelIndex & index) const;
    int columnCount(const QModelIndex & parent = QModelIndex()) const;
    int rowCount(const QModelIndex & parent = QModelIndex()) const;
    QVariant data(const QModelIndex & index, int role = Qt::DisplayRole) const;
    bool setData(const QModelIndex & index, const QVariant & value, int role);
    boost::shared_ptr<PointCloud> addCloud();
    boost::shared_ptr<PointCloud> addCloud(const char* filename);
    boost::shared_ptr<PointCloud> addCloud(boost::shared_ptr<PointCloud> pc);
    void removeCloud(int idx);
    void reset();
    
 signals:
    void cloudUpdate(boost::shared_ptr<PointCloud> pc);
    void updated(); // Very vague
    void changedSelection(std::vector<int> selection);
    void updatedActive(boost::shared_ptr<PointCloud> pc);
    void deletingCloud(boost::shared_ptr<PointCloud> cloud);

    void progressUpdate(int percentage);
    void startNonDetJob();
    void endNonDetJob();
    void listModified();

 public slots:
    void removeCloud();
    void selectionChanged(const QItemSelection &sel, const QItemSelection &des);
    boost::shared_ptr<PointCloud> loadFile(QString filename);
    bool saveFile(QString filename, std::vector<uint16_t> labels);
    
 private:
    std::mutex * mtx_;

 public:
    std::vector<boost::shared_ptr<PointCloud> > clouds_;
    std::vector<int> selection_;
    boost::shared_ptr<PointCloud> active_;
};

#endif // CLOUDLIST_H
