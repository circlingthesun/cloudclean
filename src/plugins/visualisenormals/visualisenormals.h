#ifndef VISUALISE_NORMALS_H
#define VISUALISE_NORMALS_H

#include "pluginsystem/iplugin.h"

#include <Eigen/Dense>

#include "glheaders.h"

class QAction;
class QGLShaderProgram;
class QGLBuffer;
class Core;
class CloudList;
class LayerList;
class GLWidget;
class MainWindow;
class NormalEstimator;

class VisualiseNormals : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin" FILE "visualisenormals.json")
 public:
    QString getName();
    void initialize(Core * core);
    void initialize2(PluginManager * pm);
    void cleanup();

    void initializeGL();

 private:
    void loadGLBuffers();
    void unloadGLBuffers();

 signals:
    void enabling();

 public slots:
    void enable();
    void disable();
    void paint(const Eigen::Affine3f& proj, const Eigen::Affine3f& mv);

 private:
    Core * core_;
    CloudList * cl_;
    GLWidget * glwidget_;

    MainWindow * mw_;
    QAction * enable_;

    QGLShaderProgram * program_;
    std::vector<QGLBuffer *> normal_buffers_;

    bool is_enabled_;
    std::vector<bool> buffers_loaded_;
    bool initialized_gl;

    NormalEstimator * ne_;

    GLuint vao_;

};

#endif  // VISUALISE_NORMALS_H
