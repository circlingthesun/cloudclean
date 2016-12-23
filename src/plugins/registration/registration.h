#ifndef REGISTRATION_H
#define REGISTRATION_H

#include "pluginsystem/iplugin.h"
class QAction;
class QWidget;
class Core;
class CloudList;
class LayerList;
class FlatView;
class GLWidget;
class MainWindow;

class Registration : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.registration" FILE "registration.json")
 public:
    QString getName();
    void initialize(Core * core);
    void cleanup();
    ~Registration();

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
};

#endif  // REGISTRATION_H
