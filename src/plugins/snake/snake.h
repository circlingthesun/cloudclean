#ifndef SNAKE_H
#define SNAKE_H

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
class PointCloud;

class Snake : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin" FILE "snake.json")
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
    void paint();

 private:
    void drawFloats(boost::shared_ptr<const std::vector<float> > out_img, boost::shared_ptr<PointCloud> cloud);
    void select(QMouseEvent * event);
    bool mouseClickEvent(QMouseEvent * event);
    bool mouseDblClickEvent(QMouseEvent * event);
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
    bool done_;

    QWidget * settings_;
    Lasso * lasso_;
    float min_segment_len_;

    boost::shared_ptr<std::vector<float> > img_;

    QWidget * widget_;
    int tab_idx_;

    QLabel * image_container_;
    QImage * image_;
};

#endif  // SNAKE_H
