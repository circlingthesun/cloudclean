#ifndef AUTOTEST_H
#define AUTOTEST_H

#include "pluginsystem/iplugin.h"
class QAction;
class QWidget;
class Core;
class CloudList;
class LayerList;
class FlatView;
class GLWidget;
class MainWindow;
class Project;
class FeatureEval;

class AutoTest : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.autotest" FILE "autotest.json")
 public:
    QString getName();
    void initialize(Core * core);
    void initialize2(PluginManager *pm);
    void setPermuteAndRun(std::vector<std::pair<QString, QJsonArray>> & params, QString fname, int idx);
    void cleanup();
    ~AutoTest();

 signals:
   void enabling();

 private slots:
    void enable();
    void disable();
    void runtest();

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

    Project * project_;
    FeatureEval * feature_eval_;

    QString test_path_;
};

#endif  // AUTOTEST_H
