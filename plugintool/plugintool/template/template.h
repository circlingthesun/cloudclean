#ifndef ${upper_name}_H
#define ${upper_name}_H

#include "pluginsystem/iplugin.h"
class QAction;
class QWidget;
class Core;
class CloudList;
class LayerList;
class FlatView;
class GLWidget;
class MainWindow;

class ${camel_name} : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.${lower_name}" FILE "${lower_name}.json")
 public:
    QString getName();
    void initialize(Core * core);
    void cleanup();
    ~${camel_name}();

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

#endif  // ${upper_name}_H
