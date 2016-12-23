#ifndef OUTLIERFILTER_H
#define OUTLIERFILTER_H

#include "pluginsystem/iplugin.h"
class QAction;
class QWidget;
class Core;
class CloudList;
class LayerList;
class FlatView;
class GLWidget;
class MainWindow;

class OutlierFilter : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.outlierfilter" FILE "outlierfilter.json")
 public:
    QString getName();
    void initialize(Core * core);
    void cleanup();
    ~OutlierFilter();

 signals:
   void enabling();

 private slots:
    void enable();
    void disable();
    void filter();

 private:
    Core * core_;
    CloudList * cl_;
    LayerList * ll_;
    GLWidget * glwidget_;
    FlatView * flatview_;
    MainWindow * mw_;

    QAction * enable_;
    QWidget * settings_;

    float radius_;
    uint min_neigbours_;
    bool is_enabled_;

    bool non_uniform_;
    float cam_distance_;
};

#endif  // OUTLIERFILTER_H
