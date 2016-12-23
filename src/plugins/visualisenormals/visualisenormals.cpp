#include "plugins/visualisenormals/visualisenormals.h"
#include <QDebug>
#include <QAction>
#include <QMessageBox>
#include <QGLShaderProgram>
#include <QGLBuffer>
#include <QApplication>
#include <QResource>
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "utilities/pointpicker.h"
#include "commands/select.h"
#include "plugins/normalestimation/normalestimation.h"
#include "pluginsystem/pluginmanager.h"
#include "pluginsystem/core.h"

QString VisualiseNormals::getName(){
    return "Normal visualisation";
}

void VisualiseNormals::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    glwidget_ = core_->mw_->glwidget_;
    mw_ = core_->mw_;

    initialized_gl = false;
}

void VisualiseNormals::initialize2(PluginManager * pm) {
    ne_ = pm->findPlugin<NormalEstimator>();
    if (ne_ == nullptr) {
        qDebug() << "Normal estimator plugin needed for normal viz";
        return;
    }

    enable_ = new QAction("Visualise Normals", 0);
    enable_->setCheckable(true);
    enable_->setChecked(false);

    is_enabled_ = false;

    connect(enable_, SIGNAL(triggered()), this, SLOT(enable()));
    connect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));

    mw_->addMenu(enable_, "View");

}

void VisualiseNormals::cleanup(){
    disconnect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));
    disconnect(enable_, SIGNAL(triggered()), this, SLOT(enable()));
    unloadGLBuffers();
    delete program_;
}

void VisualiseNormals::loadGLBuffers() {
    normal_buffers_.resize(cl_->clouds_.size(), nullptr);
    buffers_loaded_.resize(cl_->clouds_.size(), false);

    size_t point_size = 3*sizeof(float);

    for(uint i = 0; i < cl_->clouds_.size(); i++) {
        if(buffers_loaded_[i])
            continue;
        QGLBuffer * buff = new QGLBuffer();
        normal_buffers_[i] = buff;
        buff->create(); CE();
        buff->bind(); CE();
        buff->allocate(point_size*cl_->clouds_[i]->points.size()); CE();

        float * gbuff =
                static_cast<float *>(glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY)); CE();

        pcl::PointCloud<pcl::Normal>::Ptr normals = ne_->getNormals(cl_->clouds_[i]);

        for(uint j = 0; j < cl_->clouds_[i]->points.size(); j++){
            gbuff[3*j] = normals->at(j).data_n[0];
            gbuff[3*j+1] = normals->at(j).data_n[1];
            gbuff[3*j+2] = normals->at(j).data_n[2];
        }

        glUnmapBuffer(GL_ARRAY_BUFFER); CE();

        buff->release(); CE();
        buffers_loaded_[i] = true;
    }

}

void VisualiseNormals::unloadGLBuffers() {
    for(uint i = 0; i < normal_buffers_.size(); i++) {
        if(!buffers_loaded_[i])
            continue;
        delete normal_buffers_[i];
        normal_buffers_[i] = nullptr;
        buffers_loaded_[i] = false;
    }
}

void VisualiseNormals::initializeGL() {
    if(initialized_gl)
        return;

    program_ = new QGLShaderProgram();
    bool succ = program_->addShaderFromSourceFile(
                QGLShader::Vertex, ":/normals.vs.glsl"); CE();
    qWarning() << program_->log();
    if (!succ) qWarning() << "Shader compile log:" << program_->log();
    succ = program_->addShaderFromSourceFile(
                QGLShader::Fragment, ":/normals.fs.glsl"); CE();
    if (!succ) qWarning() << "Shader compile log:" << program_->log();
    succ = program_->addShaderFromSourceFile(
                QGLShader::Geometry, ":/normals.gs.glsl"); CE();
    if (!succ) qWarning() << "Shader compile log:" << program_->log();
    succ = program_->link(); CE();
    if (!succ) {
        qWarning() << "Could not link shader program_:" << program_->log();
        qWarning() << "Exiting...";
        abort();
    }

    glGenVertexArrays(1, &vao_);

    initialized_gl = true;
}

void VisualiseNormals::paint(const Eigen::Affine3f& proj, const Eigen::Affine3f& mv){
    initializeGL();
    loadGLBuffers();

    program_->bind();

    glUniformMatrix4fv(program_->uniformLocation("proj"), 1, GL_FALSE,
                       glwidget_->camera_.projectionMatrix().data()); CE();

    float col[3] = {0, 1, 0};
    glUniform3fv(program_->uniformLocation("line_colour"), 1, col); CE();

    glBindVertexArray(vao_);

    for(uint i = 0; i < cl_->clouds_.size(); i++){
        boost::shared_ptr<PointCloud> pc = cl_->clouds_[i];
        std::unique_ptr<QGLBuffer> & pb =
                glwidget_->gld()->cloudgldata_[pc]->point_buffer_;
        pb->bind();
        glEnableVertexAttribArray(0); CE();
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float)*4, 0); CE();
        pb->release();

        normal_buffers_[i]->bind();
        glEnableVertexAttribArray(1); CE();
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float)*3, 0); CE();
        normal_buffers_[i]->release();

        glUniformMatrix4fv(program_->uniformLocation("mv"), 1, GL_FALSE,
                           (glwidget_->camera_.modelviewMatrix() * pc->modelview()).data());CE();

        glDrawArrays(GL_POINTS, 0, cl_->clouds_[i]->size()); CE();
    }

    glBindVertexArray(0);

    program_->release();
}

void VisualiseNormals::enable() {
    if(is_enabled_){
        disable();
        return;
    }

    bool ready = true;
    for(uint i = 0; i < cl_->clouds_.size(); i++) {
        if(!ne_->normalsAvailible(cl_->clouds_[i]))
              ready = false;
    }

    if(!ready) {
        int ret = QMessageBox::warning(nullptr, tr("Normals"),
                        tr("Normals are not ready. Would you like to wait"),
                        QMessageBox::Yes | QMessageBox::No,
                        QMessageBox::No);

        if(ret == QMessageBox::No){
            enable_->setChecked(false);
            return;
        }
    }

    QTabWidget * tabs = qobject_cast<QTabWidget *>(glwidget_->parent()->parent());
    tabs->setCurrentWidget(glwidget_);
    enable_->setChecked(true);
    emit enabling();
    connect(glwidget_, SIGNAL(pluginPaint(Eigen::Affine3f, Eigen::Affine3f)),
            this, SLOT(paint(Eigen::Affine3f, Eigen::Affine3f)),
            Qt::DirectConnection);
    connect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    is_enabled_ = true;
}

void VisualiseNormals::disable() {
    unloadGLBuffers(); // TODO: Does this work? Might need to set context
    enable_->setChecked(false);
    disconnect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    disconnect(glwidget_, SIGNAL(pluginPaint(Eigen::Affine3f, Eigen::Affine3f)),
            this, SLOT(paint(Eigen::Affine3f, Eigen::Affine3f)));
    is_enabled_ = false;
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin")
