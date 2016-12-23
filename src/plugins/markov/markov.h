#ifndef MARKOV_H
#define MARKOV_H

#include "pluginsystem/iplugin.h"
class QAction;
class QWidget;
class Core;
class CloudList;
class LayerList;
class FlatView;
class GLWidget;
class MainWindow;
class Picker;
class NormalEstimator;

class Markov : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin" FILE "markov.json")
 public:
    QString getName();
    void initialize(Core * core);
    void initialize2(PluginManager * pm);
    void cleanup();

    ~Markov();

 private:
    void graphcut(int idx = 0);
    void randomforest();
    void svm();

 signals:
    void enabling();

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

    Picker * picker_;
    NormalEstimator * ne_;

    QAction * enable_;
    QAction * forrest_action_;
    QAction * svm_action_;
    bool enabled_;

    int fg_idx_;
};

#endif  // MARKOV_H
