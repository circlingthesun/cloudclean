#ifndef GLWIDGET_H
#define GLWIDGET_H

#include "glheaders.h"
#include <unordered_map>
#include <memory>
#include <functional>
#include <QGLWidget>
#include <QGLBuffer>
#include <QGLShaderProgram>
#include "gui/camera.h"
#include "gui/cloudgldata.h"
#include "model/cloudlist.h"
#include "model/layerlist.h"
#include "gui/gldata.h"
#include "gui/export.h"

class GUI_API GLWidget : public QGLWidget
{
    Q_OBJECT

public:
    GLWidget(QGLFormat & fmt, CloudList * cl,
             LayerList * ll, QWidget *parent = 0);
    ~GLWidget();

    void setGLD(GLData *gld);
    QSize minimumSizeHint() const;
    QSize sizeHint() const;

    void resetRotationMatrix();
    GLData *gld();

    float pointRenderSize(){
        return point_render_size_;
    }



protected:
    void initializeGL();
    void paintEvent(QPaintEvent *event);
    void resizeGL(int width, int height);

protected:
   void mouseDoubleClickEvent(QMouseEvent * event);
   void mouseMoveEvent(QMouseEvent * event);
   void mousePressEvent(QMouseEvent * event);
   void mouseReleaseEvent(QMouseEvent * event);
   void wheelEvent(QWheelEvent * event);
   void keyPressEvent(QKeyEvent * event);
   bool eventFilter(QObject *object, QEvent *event);

 signals:
   void pluginPaint(const Eigen::Affine3f& proj, const Eigen::Affine3f& mv);
   void rollCorrectionToggle(bool on);

 private slots:
  void contextMenu(const QPoint &pos);

 public:
   Camera camera_;

 private:
    CloudList * cl_;
    LayerList * ll_;
    GLData * gld_;

    QGLShaderProgram program_;
    QGLShaderProgram program_bg_;

    QGLBuffer bg_buff_;

    int uni_sampler_;
    int uni_projection_;
    int uni_modelview_;
    int uni_resolution_;

    float translate_unit_;
    QVector2D mouse_drag_start_;
    float point_render_size_;

    GLuint texture_id_;
    GLuint vao_, vao_bg_;

    bool gl_init_;
    Eigen::Vector2d last_mouse_pos_;
};

#endif

