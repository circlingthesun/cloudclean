#ifndef ${upper_name}_H
#define ${upper_name}_H

#include "pluginsystem/iplugin.h"

#include <cstdint>
#include <Eigen/Dense>
#include "glheaders.h"

class QMouseEvent;
class QWheelEvent;
class QKeyEvent;
class QAction;
class Core;
class CloudList;
class LayerList;
class FlatView;
class GLWidget;
class MainWindow;
class QSpinbox;
class QLabel;
class QPushButton;
class QButtonGroup;
class Picker;

class ${camel_name} : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.${lower_name}" FILE "${lower_name}.json")
 public:
    QString getName();
    void initialize(Core * core);
    void cleanup();
    bool eventFilter(QObject *object, QEvent *event);

 signals:
    void enabling();

 public slots:
    void enable();
    void disable();

 private:
    void select2D(int x, int y);
    int select3D(float x, float y);
    bool mouseClickEvent(QMouseEvent * event);
    bool mouseMoveEvent(QMouseEvent * event);
    bool mousePressEvent(QMouseEvent * event);
    bool mouseReleaseEvent(QMouseEvent * event);
    void setSelectMask(uint8_t mask);
    bool is3d();

 private:
    Core * core_;
    CloudList * cl_;
    LayerList * ll_;
    GLWidget * glwidget_;
    FlatView * flatview_;

    MainWindow * mw_;
    QAction * enable_;

    Eigen::Vector2d last_mouse_pos_;
    Eigen::Vector2d mouse_down_pos_;

    Eigen::Vector3f p1;
    Eigen::Vector3f p2;

    bool is_enabled_;
    float radius_;

    Picker * picker_;

    bool depth_adjust_;
    std::vector<QPushButton *> buttons_;
    QButtonGroup * button_group_;
    QWidget * settings_;
    int last_picked_point_;
    float last_rad_;
};

#endif  // ${upper_name}_H
