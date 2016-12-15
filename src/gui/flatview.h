#ifndef FLATVIEW_H
#define FLATVIEW_H

#include "glheaders.h"
#include <QGLWidget>
#include <QImage>
#include <QPainter>
#include <QVector2D>
#include <QGLShaderProgram>
#include "model/cloudlist.h"
#include "model/layerlist.h"
#include "model/pointcloud.h"
#include "gui/gldata.h"
#include <Eigen/Dense>
#include "gui/export.h"

class GUI_API FlatView : public QGLWidget {
    Q_OBJECT
 public:
    FlatView(QGLFormat &fmt, CloudList * cl,
             LayerList * ll, QWidget *parent = 0,
             QGLWidget * sharing = 0);
    ~FlatView();
    void setGLD(GLData *gld);
    const Eigen::Affine2f getCamera();
    const Eigen::Affine2f getNDCCamera();

    int imageToCloudIdx(int x, int y, boost::shared_ptr<PointCloud> & pc);
    QPoint cloudToImageCoord(int idx);

 public slots:
    void setCloud(boost::shared_ptr<PointCloud> new_pc);
    void rotate90() {
        rotate(M_PI/2.0f);
    }

    void rotate(float angle);


 private slots:
    void contextMenu(const QPoint &pos);

 signals:
    void flagUpdate();
    void labelUpdate();
    void pluginPaint();

 protected:
    void initializeGL();
    void paintEvent(QPaintEvent *event);
    void resizeGL(int width, int height);

    void mouseMoveEvent(QMouseEvent * event);
    void mousePressEvent(QMouseEvent * event);
    void mouseReleaseEvent(QMouseEvent * event);
    void wheelEvent(QWheelEvent * event);

 private:
    std::vector<int> cloud_idx_lookup_;
    boost::weak_ptr<PointCloud> pc_;
    CloudList * cl_;
    LayerList * ll_;
    GLData * gld_;

    QGLShaderProgram program_;

    int uni_sampler_;
    int uni_width_;
    int uni_camera_;
    GLuint texture_id_;
    GLuint vao_;


    Eigen::Affine2f transform_;

    float current_scale_;

    float rotation_;
    Eigen::Vector2f aspect_;
    Eigen::Vector2f last_mouse_pos_;
    bool gl_init_;
};

#endif // FLATVIEW_H
