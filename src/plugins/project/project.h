#ifndef project_H
#define project_H

#include "pluginsystem/iplugin.h"
#include "plugins/project/export.h"
class QAction;
class QWidget;
class Core;
class CloudList;
class LayerList;
class FlatView;
class GLWidget;
class MainWindow;

class PROJECT_API Project : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.project" FILE "project.json")
 public:
    QString getName();
    void initialize(Core * core);
    void cleanup();
    ~Project();

    void load(QString filename);

 private slots:
    void save();
    void load();

 private:
    Core * core_;
    CloudList * cl_;
    LayerList * ll_;
    GLWidget * glwidget_;
    FlatView * flatview_;
    MainWindow * mw_;

    QAction * save_action_;
    QAction * load_action_;
};

#endif  // project_H
