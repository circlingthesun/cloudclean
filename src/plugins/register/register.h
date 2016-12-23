#ifndef REGISTER_H
#define REGISTER_H

#include "pluginsystem/iplugin.h"
class QAction;
class QWidget;
class QComboBox;
class Core;
class CloudList;
class LayerList;
class FlatView;
class GLWidget;
class MainWindow;

class Register : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.register" FILE "register.json")
 public:
    QString getName();
    void initialize(Core * core);
    void cleanup();
    ~Register();

 signals:
   void enabling();

 private slots:
    void enable();
    void disable();
    void clModified();
    void align();

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

    float radius_;

    QComboBox * stationary_cb_;
    QComboBox * moving_cb_;

    int stationary_idx_;
    int moving_idx_;
};

#endif  // REGISTER_H
