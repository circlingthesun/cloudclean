#ifndef GRAPH_CUT_H
#define GRAPH_CUT_H

#include "pluginsystem/iplugin.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <memory>
#include <map>
#include <boost/shared_ptr.hpp>

class QAction;
class QWidget;
class Core;
class CloudList;
class LayerList;
class FlatView;
class GLWidget;
class MainWindow;
class Event;
class QMouseEvent;
class NormalEstimator;
class Picker;
class PointCloud;

enum class Feature: uint {
    Normal,
    Intensity,
    Connectivity
};

enum class TerminationCond: uint {
    SourceDeviation,
    NeighbourDeviation
};

class Flood : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin" FILE "flood.json")
 public:
    QString getName();
    void initialize(Core * core);
    void initialize2(PluginManager * pm);
    void cleanup();
    ~Flood();

    boost::shared_ptr<std::vector<int> > getLayerIndices();

 signals:
   void enabling();

 private slots:
    void flood(int source_idx);
    //void global_flood();
    void global_flood2();

 public slots:
   void enable();
   void disable();

 private:
    Core * core_;
    CloudList * cl_;
    LayerList * ll_;
    GLWidget * glwidget_;
    FlatView * flatview_;
    MainWindow * mw_;

    QAction * enable_;
    QAction * global_flood_;
    QAction * global_flood2_;
    QWidget * settings_;
    bool is_enabled_;
    NormalEstimator * ne_;
    Picker * picker_;

    float threshold_;
    TerminationCond term_cond_;
    Feature feature_;

    int k_;

    std::map<boost::shared_ptr<PointCloud>, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> cache_;
    std::map<boost::shared_ptr<PointCloud>, std::vector<int>> cache2_;
    std::map<boost::shared_ptr<PointCloud>, std::vector<std::vector<int>>> cache3_;
};

#endif  // GRAPH_CUT_H
