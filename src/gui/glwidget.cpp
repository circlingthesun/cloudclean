#include "glwidget.h"
#include <QResource>
#include <QKeyEvent>
#include <QCoreApplication>
#include <QWindow>
#include <cmath>
#include <cstdlib>
#include <boost/make_shared.hpp>
#include "utilities/pointpicker.h"

using namespace Eigen;

GLWidget::GLWidget(QGLFormat &fmt, CloudList *cl,
                   LayerList *ll, QWidget *parent)
    : QGLWidget(fmt, parent) {
    setFocusPolicy(Qt::StrongFocus);
    translate_unit_ = 0.4;
    point_render_size_ = 4.0f;

    cl_ = cl;
    ll_ = ll;
    setMouseTracking(true);
    setContextMenuPolicy(Qt::CustomContextMenu);
    setAutoFillBackground(false);
    setAutoBufferSwap(false);
    gl_init_ = false;

    connect(&camera_, SIGNAL(updated()), this, SLOT(update()));

}

GLWidget::~GLWidget() {
}

void GLWidget::setGLD(GLData * gld){
    gld_ = gld;
}

QSize GLWidget::minimumSizeHint() const {
    return QSize(500, 500);
}

QSize GLWidget::sizeHint() const {
    return QSize(700, 500);
}

void GLWidget::initializeGL() {
    #if defined(Q_OS_WIN32)
        glewExperimental = true;
        GLenum GlewInitResult = glewInit();
        if (GlewInitResult != GLEW_OK) {
            const GLubyte* errorStr = glewGetErrorString(GlewInitResult);
            size_t size = strlen(reinterpret_cast<const char*>(errorStr));
            qDebug() << "Glew error "
                     << QString::fromUtf8(
                            reinterpret_cast<const char*>(errorStr), size);
        }
    #endif

    CE();

    glClearColor(1.0, 1.0, 1.0, 1.0);
    glEnable(GL_DEPTH_TEST);
    //glEnable(GL_CULL_FACE);
    glEnable(GL_MULTISAMPLE);
    //glEnable(GL_POINT_SMOOTH);
    
    //
    // Load shader program
    //
    bool succ = program_.addShaderFromSourceFile(
                QGLShader::Vertex, ":/points.vert"); CE();
    if (!succ) qWarning() << "Shader compile log:" << program_.log();
    succ = program_.addShaderFromSourceFile(
                QGLShader::Fragment, ":/points.frag"); CE();
    if (!succ) qWarning() << "Shader compile log:" << program_.log();
    succ = program_.link(); CE();
    if (!succ) {
        qWarning() << "Could not link shader program_:" << program_.log();
        qWarning() << "Exiting...";
        abort();
    }


    //
    // Load bg shader program
    //
    succ = program_bg_.addShaderFromSourceFile(
                QGLShader::Vertex, ":/bg.vs.glsl"); CE();
    if (!succ) qWarning() << "Shader compile log:" << program_bg_.log();
    succ = program_bg_.addShaderFromSourceFile(
                QGLShader::Fragment, ":/bg.fs.glsl"); CE();
    if (!succ) qWarning() << "Shader compile log:" << program_bg_.log();
    succ = program_bg_.link(); CE();
    if (!succ) {
        qWarning() << "Could not link shader program_:" << program_bg_.log();
        qWarning() << "Exiting...";
        abort();
    }

    program_bg_.bind();
    uni_resolution_ = program_bg_.uniformLocation("resolution"); RC(uni_resolution_);
    program_bg_.release();

    glGenVertexArrays(1, &vao_bg_);

    bg_buff_.create(); CE();
    bg_buff_.bind(); CE();
    size_t bsize = 4*sizeof(float)*2;
    bg_buff_.allocate(bsize); CE();
    float data[] = { -1, -1,
                     -1,  1,
                      1,  -1,
                      1,  1};

    bg_buff_.write(0, data, bsize);
    bg_buff_.release();


    //
    // Resolve uniforms
    //
    program_.bind(); CE();
    uni_sampler_ = program_.uniformLocation("sampler"); RC(uni_sampler_);
    uni_projection_ = program_.uniformLocation("projection"); RC(uni_projection_);
    uni_modelview_ = program_.uniformLocation("modelview"); RC(uni_modelview_);

    program_.release(); CE();

    //
    // Set camera
    //
    camera_.setDepthRange(0.1f, 100000.0f);
    camera_.setAspect(width() / static_cast<float>(height()));

    //
    // Set up textures & point size
    //
    glGenTextures(1, &texture_id_); CE();
    glPointSize(point_render_size_);

    // Generate vao
    glGenVertexArrays(1, &vao_);
    gl_init_ = true;

    // fix incorrect initial viewport
    connect(gld_, &GLData::update, [=] (){
        resizeGL(width(), height());
    });
}


void GLWidget::paintEvent(QPaintEvent *event) {
    if(!isValid())
        return;

    makeCurrent();

    if(!gl_init_){
        initializeGL();
    }

    //makeCurrent();
    // Make sure the labels are updates
    // Make sure nothing has changed

//    program_bg_.bind(); CE();

//    glUniform2f(uni_resolution_, width(), height()); CE();

//    glBindVertexArray(vao_bg_); CE();
//    bg_buff_.bind(); CE();
//    glEnableVertexAttribArray(0); CE();
//    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0); CE();

//    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4); CE();
//    bg_buff_.release(); CE();
//    glBindVertexArray(0); CE();
//    program_bg_.release(); CE();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClear(GL_DEPTH_BUFFER_BIT); CE();
    glEnable(GL_DEPTH_TEST); CE();

    program_.bind(); CE();


    glUniformMatrix4fv(uni_modelview_, 1, GL_FALSE,
                       camera_.modelviewMatrix().data());CE();
    glUniformMatrix4fv(uni_projection_, 1, GL_FALSE,
                       camera_.projectionMatrix().data());

    glUniform1i(uni_sampler_, 0); CE();
    glBindTexture(GL_TEXTURE_BUFFER, texture_id_); CE();
    auto buff = gld_->color_lookup_buffer_;
	auto id = buff->bufferId();
    glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, id); CE();

    bool clouds_moving = false;

    // Draw all clouds
    for(std::pair<boost::shared_ptr<PointCloud>, boost::shared_ptr<CloudGLData> > pair: gld_->cloudgldata_) {
        boost::shared_ptr<PointCloud> pc = pair.first;
        boost::shared_ptr<CloudGLData> cd = pair.second;

        // TODO: Why can this happen?
        if(!pc)
            continue;

        if(pc->isMoving())
            clouds_moving = true;

        if(!pc->isVisible() && cd.get() != nullptr){
            qDebug() << "Should be deleted now";
            gld_->cloudgldata_[pc].reset();
        }

        if(!pc->isVisible())
            continue;

        if(pc->isVisible() && cd.get() == nullptr){
            qDebug() << "Should be recreated now";
            gld_->cloudgldata_[pc].reset(new CloudGLData(pc));
            cd = gld_->cloudgldata_[pc];
        }

        glBindVertexArray(vao_);

        // Point buffer
        cd->point_buffer_->bind(); CE();
        glEnableVertexAttribArray(0); CE();
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float)*4, 0); CE();
        glEnableVertexAttribArray(1); CE();
        int offset = sizeof(float)*3;
        glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(float)*4,
                              reinterpret_cast<const void *>(offset)); CE();
        cd->point_buffer_->release(); CE();

        // Label buffer
        cd->label_buffer_->bind(); CE();
        glEnableVertexAttribArray(2); CE(); CE();
        glVertexAttribIPointer(2, 1, GL_SHORT, 0, 0); CE();
        cd->label_buffer_->release(); CE();

        // Flag buffer
        cd->flag_buffer_->bind(); CE();
        glEnableVertexAttribArray(3); CE();
        glVertexAttribIPointer(3, 1, GL_BYTE, 0, 0); CE();
        cd->flag_buffer_->release(); CE();

        // Color buffer
        cd->color_buffer_->bind(); CE();
        glEnableVertexAttribArray(4); CE();
        glVertexAttribPointer(4, 3, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(uint8_t)*3, 0); CE();
        cd->color_buffer_->release(); CE();

        glUniformMatrix4fv(uni_modelview_, 1, GL_FALSE,
                           (camera_.modelviewMatrix()*pc->modelview())
                           .data());CE();

        cd->draw(vao_);
        glBindVertexArray(0);
    }

    camera_.alwaysUpdate(clouds_moving);

    glBindTexture(GL_TEXTURE_BUFFER, 0); CE();

    emit pluginPaint(camera_.projectionMatrix(), camera_.modelviewMatrix());
    glFinish();
    swapBuffers();
}

void GLWidget::resizeGL(int width, int height) {
    camera_.setAspect(width / static_cast<float>(height));
    glViewport(0, 0, width, qMax(height, 1));
}

void GLWidget::mouseDoubleClickEvent(QMouseEvent * event) {

}

void GLWidget::mouseMoveEvent(QMouseEvent * event) {
    float damp = 0.005;
    Vector2f rot(event->x()-last_mouse_pos_.x(), event->y()-last_mouse_pos_.y());
    last_mouse_pos_ << event->x(), event->y();
    rot*=damp;

    if(event->buttons() == Qt::LeftButton && event->modifiers() != Qt::ControlModifier){
        camera_.rotate2D(rot.x(), rot.y());
    }
    else if(event->buttons() && event->modifiers() == Qt::ControlModifier){
        boost::shared_ptr<PointCloud> pc = cl_->active_;
        pc->rotate2D(rot.x(), -rot.y());
    }
}

void GLWidget::mousePressEvent(QMouseEvent * event) {
    mouse_drag_start_ = QVector2D(0.0f, 0.0f);
    last_mouse_pos_ << event->x(), event->y();
}

void GLWidget::mouseReleaseEvent(QMouseEvent * event) {
    last_mouse_pos_ << event->x(), event->y();
}

void GLWidget::wheelEvent(QWheelEvent * event) {
    camera_.adjustFov(event->delta());
    update();
}

void GLWidget::keyPressEvent(QKeyEvent * event) {
    float push_factor = 0.1;
    switch (event->key()) {

    //
    // Translations:
    //
    case Qt::Key_D:
    case Qt::Key_Right:
        if (event->modifiers() == Qt::ControlModifier && cl_->active_ != nullptr)
            cl_->active_->translate(translate_unit_*push_factor * camera_.rotation_current_.matrix().inverse() * Vector3f::UnitX());
        else if(event->modifiers() != Qt::ControlModifier)
            camera_.translate(-translate_unit_ * Vector3f::UnitX());
        return;
    case Qt::Key_A:
    case Qt::Key_Left:
        if (event->modifiers() == Qt::ControlModifier && cl_->active_ != nullptr)
            cl_->active_->translate(-translate_unit_*push_factor * camera_.rotation_current_.matrix().inverse() * Vector3f::UnitX());
        else if (event->modifiers() != Qt::ControlModifier)
            camera_.translate(translate_unit_ * Vector3f::UnitX());
        return;
    case Qt::Key_W:
    case Qt::Key_Up:
        if (event->modifiers() == Qt::ControlModifier && cl_->active_ != nullptr)
            cl_->active_->translate(-translate_unit_*push_factor * camera_.rotation_current_.matrix().inverse() * Vector3f::UnitZ());
        else if (event->modifiers() != Qt::ControlModifier)
            camera_.translate(translate_unit_ * Vector3f::UnitZ());
        return;
    case Qt::Key_S:
    case Qt::Key_Down:
        if (event->modifiers() == Qt::ControlModifier && cl_->active_ != nullptr)
            cl_->active_->translate(translate_unit_*push_factor * camera_.rotation_current_.matrix().inverse() * Vector3f::UnitZ());
        else if (event->modifiers() != Qt::ControlModifier)
            camera_.translate(-translate_unit_ * Vector3f::UnitZ());
        return;
    case Qt::Key_Q:
        if (event->modifiers() == Qt::ControlModifier && cl_->active_ != nullptr)
            cl_->active_->translate(-translate_unit_*push_factor * camera_.rotation_current_.matrix().inverse() * Vector3f::UnitY());
        else if (event->modifiers() != Qt::ControlModifier)
            camera_.translate(translate_unit_ * Vector3f::UnitY());
        return;
    case Qt::Key_E:
        if (event->modifiers() == Qt::ControlModifier && cl_->active_ != nullptr)
            cl_->active_->translate(translate_unit_*push_factor * camera_.rotation_current_.matrix().inverse() * Vector3f::UnitY());
        else if (event->modifiers() != Qt::ControlModifier)
            camera_.translate(-translate_unit_ * Vector3f::UnitY());
        return;
//   case Qt::Key_C:
//        camera_.toggleRollCorrection();
//        emit rollCorrectionToggle(camera_.roll_correction());
//        return;

    //
    // Reset
    //
    case Qt::Key_R:
        if (event->modifiers() == Qt::ControlModifier && cl_->active_ != nullptr){
            camera_.setPosition(0, 0, 0);
            camera_.setRotate3D(-M_PI/2, M_PI/4, 0);
        }
        return;

    // Birds eye view
    case Qt::Key_B:
        if (event->modifiers() == Qt::ControlModifier && cl_->active_ != nullptr){
            camera_.birds_eye();
        }
        return;

    //
    // Zoom
    //
    case Qt::Key_Equal:
        //if(event->modifiers() != Qt::ShiftModifier)
        //    break;
    case Qt::Key_Plus:
        if (point_render_size_ < 30)
            point_render_size_++;
        glPointSize(point_render_size_);
        break;
    case Qt::Key_Minus:
        if ( point_render_size_ > 1 )
            point_render_size_--;
        glPointSize(point_render_size_);
        break;
    }
    update();
}


bool GLWidget::eventFilter(QObject *object, QEvent *event) {
    Q_UNUSED(object);
    if (event->type() == QEvent::KeyPress) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        keyPressEvent(keyEvent);
        return true;
    }
    return false;
}

void GLWidget::contextMenu(const QPoint &pos) {

}

GLData * GLWidget::gld(){
    return gld_;
}
