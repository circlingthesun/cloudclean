#ifndef FEATURE_LIST_H
#define FEATURE_LIST_H

#include <QDebug>
#include <tuple>
#include <QAbstractListModel>
#include <QModelIndex>

class QAction;

class Feature{
public:
    QString name_;
    int id_ = -1;
    bool enabled_ = true;
    static int next_id_;
    Feature(): name_("Feature"), id_(next_id_++){}
    Feature(QString name, int id = -1, bool enabled = true): name_(name), id_(id), enabled_(enabled){}
};

class FeatureList : public QAbstractListModel {
    Q_OBJECT
 public:
    explicit FeatureList(QObject *parent = 0);
    ~FeatureList();
    Qt::ItemFlags flags(const QModelIndex & index) const;
    int columnCount(const QModelIndex &) const;
    int rowCount(const QModelIndex & parent = QModelIndex()) const;
    QVariant data(const QModelIndex & index, int role = Qt::DisplayRole) const;
    void addFeature(Feature = Feature());
    void remove(int idx);
    bool setData(const QModelIndex & index, const QVariant & value, int role);

    int activeCount(){
        int c = 0;
        for(Feature & f : features_) {
            if(f.enabled_){
                c++;
            }
        }
        return c;
    }

    bool hasFeature(QString name){
        for(Feature & f : features_) {
            if(f.name_ == name && f.enabled_){
                return true;
            }
        }
        return false;
    }

    void selectOnly(std::vector<std::string> feature_names) {
        beginResetModel();
        for(Feature & f : features_) {
            bool found = false;
            for(std::string name : feature_names) {
                if(name == f.name_.toStdString()) {
                    found = true;
                }
            }

            f.enabled_ = found;
        }

        endResetModel();
    }

    std::vector<Feature> features_ = {
        Feature("X", 0),
        Feature("Y", 1),
        Feature("Z", 2),
        Feature("X Normal", 3, false),
        Feature("Y Normal", 4, false),
        Feature("Z Normal", 5, false),
        Feature("Intensity", 6),
        Feature("Eigen 1", 7, false),
        Feature("Eigen 2", 8, false),
        Feature("Eigen 3", 9, false),
        Feature("Curvature 1", 10),
        Feature("Curvature 2", 11),
        Feature("Anisotrophy", 12, false),
        Feature("Planarity", 13, false),
        Feature("Sphericity", 14, false),
        Feature("Linearity", 15, false),
        Feature("Omnivariance", 16, false),
        Feature("Curvature", 17, false),
        Feature("Eigenentrophy", 18, false),
        Feature("Distance from origin", 19, false),
        Feature("Number of neighbours", 20, false),
    };
 signals:


 public slots:


 private:

};

#endif
