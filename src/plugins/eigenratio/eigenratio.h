#ifndef EIGENRATIO_H
#define EIGENRATIO_H

#include "pluginsystem/iplugin.h"
class QAction;
class QWidget;
class Core;
class CloudList;
class LayerList;
class FlatView;
class GLWidget;
class MainWindow;

class EigenRatio : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin" FILE "eigenratio.json")
 public:
    QString getName();
    void initialize(Core * core);
    void cleanup();
    ~EigenRatio();

 private slots:
    void myFunc();

 private:
    Core * core_;
    CloudList * cl_;
    LayerList * ll_;
    GLWidget * glwidget_;
    FlatView * flatview_;
    MainWindow * mw_;

    QAction * myaction;
    QWidget * settings_;
};

#endif  // EIGENRATIO_H
