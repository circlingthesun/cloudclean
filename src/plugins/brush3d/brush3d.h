#ifndef BRUSH_3D_H
#define BRUSH_3D_H

#include "pluginsystem/iplugin.h"

#include <cstdint>
#include <Eigen/Dense>
#include "glheaders.h"

class QMouseEvent;
class QWheelEvent;
class QKeyEvent;
class QAction;
class QGLShaderProgram;
class QGLBuffer;
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

class Brush3D : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin" FILE "brush3d.json")
 public:
    QString getName();
    void initialize(Core * core);
    void cleanup();

    void initializeGL();

    bool eventFilter(QObject *object, QEvent *event);

 signals:
    void enabling();

 public slots:
    void enable();
    void disable();
    void paint(const Eigen::Affine3f& proj, const Eigen::Affine3f& mv);

 private:
    void select2D(int x, int y);
    int select3D(float x, float y);
    int select(int idx, int x, int y);
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
    bool initialized_gl;
    MainWindow * mw_;
    QAction * enable_;

    Eigen::Vector2d last_mouse_pos_;
    Eigen::Vector2d mouse_down_pos_;

    Eigen::Vector3f p1;
    Eigen::Vector3f p2;
    QGLShaderProgram * program_;
    QGLBuffer * line_;

    bool is_enabled_;
    float radius_;

    Picker * picker_;

    bool depth_adjust_;
    bool spacially_aware_;
    std::vector<QPushButton *> buttons_;
    QButtonGroup * button_group_;
    QWidget * settings_;
    int last_picked_point_;
    float last_rad_;
};

#endif  // BRUSH_3D_H
