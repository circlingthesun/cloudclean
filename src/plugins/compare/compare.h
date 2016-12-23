#ifndef COMPARE_H
#define COMPARE_H

#include <set>
#include <boost/weak_ptr.hpp>
#include "pluginsystem/iplugin.h"
class QAction;
class QWidget;
class Core;
class CloudList;
class LayerList;
class FlatView;
class GLWidget;
class MainWindow;
class Layer;
class QLineEdit;

class Compare : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.compare" FILE "compare.json")
 public:
    QString getName();
    void initialize(Core * core);
    void cleanup();
    ~Compare();

 signals:
   void enabling();

 private slots:
    void enable();
    void disable();
    void compare();

 private:
    Core * core_;
    CloudList * cl_;
    LayerList * ll_;
    GLWidget * glwidget_;
    FlatView * flatview_;
    MainWindow * mw_;

    QAction * enable_;
    QWidget * settings_;
    bool is_enabled_;

    QLineEdit * precision_;
    QLineEdit * recall_;

    float radius_;
    std::vector<boost::weak_ptr<Layer> >  layers1_;
    std::vector<boost::weak_ptr<Layer> >  layers2_;
};

#endif  // COMPARE_H
