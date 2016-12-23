#ifndef LASSO_3D_H
#define LASSO_3D_H

#include "pluginsystem/iplugin.h"

#include <Eigen/Dense>

#include "glheaders.h"
#include "utilities/lasso.h"

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

#include <QTime>

class Lasso3D : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin" FILE "lasso3d.json")
 public:
    QString getName();
    void initialize(Core * core);
    void cleanup();

    void initializeGL();

    bool eventFilter(QObject *object, QEvent *event);

 signals:
    void enabling();

 private slots:

 public slots:
    void enable();
    void disable();
    void paint2d();
    void paint(const Eigen::Affine3f& proj, const Eigen::Affine3f& mv);

 private:
    void select(QMouseEvent * event);
    bool is3d();
    bool mouseClickEvent(QMouseEvent * event);
    bool mouseDblClickEvent(QMouseEvent *);
    bool mouseMoveEvent(QMouseEvent * event);
    bool mousePressEvent(QMouseEvent * event);
    bool mouseReleaseEvent(QMouseEvent * event);

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

    bool is_enabled_;

    QWidget * settings_;
    Lasso * lasso_;

    int invocations_;
    int action_count_;
    float seconds_;
    QTime timer_;
    QString log_;
};

#endif  // LASSO_3D_H
