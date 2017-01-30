#ifndef AUTOTEST_H
#define AUTOTEST_H

#include "pluginsystem/iplugin.h"
class QAction;
class QWidget;
class QDockWidget;
class Core;
class CloudList;
class LayerList;
class FlatView;
class GLWidget;
class MainWindow;
class Project;
class Markov;
class Accuracy;
class QDoubleSpinBox;

class AutoTest : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.autotest" FILE "autotest.json")
 public:
    QString getName();
    void initialize(Core * core);
    void initialize2(PluginManager *pm);
    std::tuple<float, float, float, double,double,double,double,double,double,double,double,double,double,double> runTest(std::vector<std::__cxx11::string> features, float downsample, float curvature_radius, float pca_radius, float density_radius, int pca_max_nn, int tree_count, int tree_depth);
    void cleanup();
    void knntest();
    ~AutoTest();

 signals:
   void enabling();

 private slots:
    void enable();
    void disable();
    void runtests();

 private:
    Core * core_;
    CloudList * cl_;
    LayerList * ll_;
    GLWidget * glwidget_;
    FlatView * flatview_;
    MainWindow * mw_;

    QAction * enable_;
    QWidget * dock_widget_;
    bool is_enabled_;

    QDoubleSpinBox * radius_spinner_;
    float radius_;

    Project * project_;
    Accuracy * accuracy_;
    Markov * markov_;

    QDockWidget * dock_;

    QString test_path_;
};

#endif  // AUTOTEST_H
