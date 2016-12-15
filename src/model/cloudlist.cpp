#include "cloudlist.h"
#include <QTextStream>
#include <QUndoStack>
#include <QItemSelection>
#include <QApplication>
#include <QFileInfo>
#include <commands/select.h>

CloudList::CloudList(QObject *parent)
    : QAbstractListModel(parent) {
    mtx_ = new std::mutex();
}

CloudList::~CloudList(){
    delete mtx_;
}

Qt::ItemFlags CloudList::flags(const QModelIndex & index) const {
    int col = index.column();

    if (col == 0) {
        return Qt::ItemIsUserCheckable | Qt::ItemIsEnabled
                | Qt::ItemIsSelectable;
    }

    return Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled;
}

int CloudList::columnCount(const QModelIndex &) const {
    return 2;
}

int CloudList::rowCount(const QModelIndex &) const {
    return clouds_.size();
}

QVariant CloudList::data(const QModelIndex & index, int role) const {
    int row = index.row();
    int col = index.column();

    if (col == 1) {
        switch (role) {
            case Qt::DisplayRole:
            {
                QString re;
                QFileInfo fi(clouds_[row]->filepath());
                QTextStream(&re) << fi.fileName();
                return re;
            }
        }
    } else if (col == 0) {
        switch (role) {
        case Qt::CheckStateRole:
                return clouds_[row]->isVisible() ? Qt::Checked : Qt::Unchecked;
        }
    }

    return QVariant();
}

bool CloudList::setData(const QModelIndex & index, const QVariant & value,
                        int role) {
    int row = index.row();
    int col = index.column();

    if (role == Qt::CheckStateRole && col == 0) {
        clouds_[row]->toggleVisible();
        emit dataChanged(index, index);
        emit updated();
    }
    return true;
}

boost::shared_ptr<PointCloud> CloudList::addCloud() {
    boost::shared_ptr<PointCloud> pc(new PointCloud());
    return addCloud(pc);
}

boost::shared_ptr<PointCloud> CloudList::addCloud(const char* filename) {
    boost::shared_ptr<PointCloud> pc(new PointCloud());
    pc->load_ptx(filename);
    return addCloud(pc);
}

boost::shared_ptr<PointCloud> CloudList::addCloud(boost::shared_ptr<PointCloud> pc) {
    mtx_->lock();
    beginInsertRows(QModelIndex(), clouds_.size(), clouds_.size());
    clouds_.push_back(pc);
    if(active_.get() == nullptr)
        active_ = pc;
    endInsertRows();
    mtx_->unlock();

    //connect(pc.get(), SIGNAL(flagUpdate()), this, SIGNAL(updated()));
    //connect(pc.get(), SIGNAL(labelUpdate()), this, SIGNAL(updated()));
    connect(pc.get(), SIGNAL(transformed()), this, SIGNAL(updated()));

    emit cloudUpdate(pc);
    return pc;
}

void CloudList::removeCloud(){
    int idx = sender()->property("cloud_id").toInt();
    removeCloud(idx);
}

void CloudList::removeCloud(int idx){
    boost::shared_ptr<PointCloud> pc = clouds_[idx];
    if(pc == active_)
        active_.reset();

    pc->deleting_ = true;
    emit deletingCloud(pc);

    //disconnect(pc.get(), SIGNAL(flagUpdate()), this, SIGNAL(updated()));
    //disconnect(pc.get(), SIGNAL(labelUpdate()), this, SIGNAL(updated()));
    disconnect(pc.get(), SIGNAL(transformed()), this, SIGNAL(updated()));


    beginRemoveRows(QModelIndex(), idx, idx);
    clouds_.erase(clouds_.begin()+idx);
    endRemoveRows();
    active_.reset();
    if(clouds_.size() > 0)
        active_ = clouds_.at(0);
    emit listModified();
    emit updated();
}

void CloudList::selectionChanged(const QItemSelection &sel,
                                     const QItemSelection &des) {
    selection_.clear();
    for (QModelIndex s : sel.indexes()) {
        selection_.push_back(s.row());
    }
    emit changedSelection(selection_);

    if(selection_.size() != 0)
        active_ = clouds_[selection_[0]];
        emit updatedActive(active_);
}

boost::shared_ptr<PointCloud> CloudList::loadFile(QString filename){
    if(!QFileInfo(filename).exists())
        return nullptr;

    boost::shared_ptr<PointCloud> pc;
    pc.reset(new PointCloud());
    pc->moveToThread(QApplication::instance()->thread());

    connect(pc.get(), SIGNAL(progress(int)), this, SIGNAL(progressUpdate(int)));
    pc->load_ptx(filename.toLocal8Bit().constData());
    emit progressUpdate(0);
    disconnect(pc.get(), SIGNAL(progress(int)), this, SIGNAL(progressUpdate(int)));

    emit startNonDetJob();
    addCloud(pc);
    emit endNonDetJob();
    emit listModified();
    return pc;
}

bool CloudList::saveFile(QString filename, std::vector<uint16_t> labels) {
    connect(active_.get(), SIGNAL(progress(int)), this, SIGNAL(progressUpdate(int)));
    active_->save_ptx(filename.toLocal8Bit().constData(), labels);
    connect(active_.get(), SIGNAL(progress(int)), this, SIGNAL(progressUpdate(int)));
    emit progressUpdate(0);
    return true;
}

void CloudList::reset(){
    beginResetModel();
    selection_.clear();
    emit changedSelection(selection_);
    clouds_.clear();
    active_ = nullptr;
    endResetModel();
}
