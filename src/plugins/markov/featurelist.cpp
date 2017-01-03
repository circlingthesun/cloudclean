#include "featurelist.h"

#include <QAction>
#include <QCheckBox>
#include <QSlider>
#include <QLabel>
#include <QToolBar>
#include <QVBoxLayout>
#include <QDockWidget>
#include <QApplication>
#include <QMenu>
#include <QMenuBar>
#include <QToolTip>
#include <QStatusBar>
#include <QSpinBox>
#include <QTableView>
#include <QHeaderView>
#include <QTextStream>
#include <QDebug>

FeatureList::FeatureList(QObject *parent) : QAbstractListModel(parent) {

}

FeatureList::~FeatureList(){

}

Qt::ItemFlags FeatureList::flags(const QModelIndex & index) const {
    int col = index.column();

    if (col == 0) {
        return Qt::ItemIsUserCheckable | Qt::ItemIsEnabled
                | Qt::ItemIsSelectable;
    }

    return Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled;
}

int FeatureList::columnCount(const QModelIndex &) const {
    return 2;
}

int FeatureList::rowCount(const QModelIndex &) const {
    return features_.size();
}

QVariant FeatureList::data(const QModelIndex & index, int role) const {
    int row = index.row();
    int col = index.column();

    if (col == 1) {
        switch (role) {
        case Qt::EditRole:
        case Qt::DisplayRole:
        {
            QString re;
            QTextStream(&re) << features_[row].name_;
            return re;
        }
        }
    } else if (col == 0) {
        switch (role) {
        case Qt::CheckStateRole:
                return features_[row].enabled_ ? Qt::Checked : Qt::Unchecked;
        }
    }

    return QVariant();
}

bool FeatureList::setData(const QModelIndex & index, const QVariant & value,
                        int role) {
    int row = index.row();
    int col = index.column();

    if (role == Qt::CheckStateRole && col == 0) {
        features_[row].enabled_ = !features_[row].enabled_;
    }
    return true;
}

void FeatureList::addFeature(Feature contour){
    beginInsertRows(QModelIndex(), features_.size(), features_.size());
    features_.push_back(contour);
    endInsertRows();
}
