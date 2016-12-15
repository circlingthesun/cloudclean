#include "flatview.h"
#include <algorithm>
#include <QDebug>
#include <QMouseEvent>
#include <QApplication>
#include <QLabel>
#include <QMessageBox>
#include <boost/make_shared.hpp>
#include "commands/select.h"

FlatView::FlatView(QGLFormat & fmt, CloudList * cl,
                   LayerList * ll, QWidget *parent, QGLWidget *sharing)
    : QGLWidget(fmt, parent, sharing), rotation_(0.0f) {
    setFocusPolicy(Qt::StrongFocus);
    cl_ = cl;
    ll_ = ll;
    current_scale_ = 1;
    transform_.setIdentity();
    setMouseTracking(true);
    setAutoFillBackground(false);
    setAutoBufferSwap(false);

    setContextMenuPolicy(Qt::CustomContextMenu);
    gl_init_ = false;
}

FlatView::~FlatView(){

}

void FlatView::setGLD(GLData * gld){
    gld_ = gld;
    rotate(M_PI/2 *3);
}


/*            height
 *
 *           ********
 *           ********
 *           ********
 *           ********
 *  width    ********
 *           ********
 *           ********
 *           ********
 *           ********
 */

const Eigen::Affine2f FlatView::getCamera() {
    auto pc = pc_.lock();

    return
            transform_ *
            Eigen::AlignedScaling2f(aspect_) *
            Eigen::Rotation2Df(rotation_) *
            Eigen::Translation2f(-pc->scan_width()*0.5, -pc->scan_height()*0.5)  *
            Eigen::Affine2f::Identity();
}

const Eigen::Affine2f FlatView::getNDCCamera() {
    if(pc_.expired())
        return Eigen::Affine2f::Identity();
    auto pc = pc_.lock();

    Eigen::Vector2f aspect;

    float war = width()/float(height());
    float sar = pc->scan_width()/float(pc->scan_height());

    float cfx = sar/war;
    float cfy = (1/sar)/(1/war);

    // screen if wider than scan
    if(war <  sar){
        aspect = Eigen::Vector2f(1.0, 1.0/cfx);
    } else {
        aspect = Eigen::Vector2f(1.0/cfy, 1.0);
    }


    return
            transform_ *
            Eigen::AlignedScaling2f(aspect) *
            Eigen::Rotation2Df(rotation_) *
            Eigen::Affine2f::Identity();
}

int binary_search(std::vector<int> A, int key) {
    int imin = 0;
    int imax = A.size();

    while (imax >= imin) {
        int imid = (imin+imax)/2;
        if (A[imid] < key)
            imin = imid + 1;
        else if (A[imid] > key)
            imax = imid - 1;
        else
            return imid;
    }
    return -1;
}

int FlatView::imageToCloudIdx(int x, int y, boost::shared_ptr<PointCloud> &pc){
    if(x < 0 || x > pc->scan_width()){
        return -1;
    }
    else if(y < 0 || y > pc->scan_height()){
        return -1;
    }

    uint idx = x + y*pc->scan_width();

    if(idx < 0 || idx > cloud_idx_lookup_.size())
        return -1;

    return cloud_idx_lookup_[idx];
}

// scan lines go from top to bottom & left to right

inline QPoint FlatView::cloudToImageCoord(int idx){
    boost::shared_ptr<PointCloud> pc = pc_.lock();
    int i = pc->cloudToGridMap()[idx];
    int x = i/pc->scan_height();
    int y = pc->scan_height() - 1 - (i%pc->scan_height());
    return QPoint(x, y);
}

void FlatView::setCloud(boost::shared_ptr<PointCloud> new_pc) {
    if(new_pc == pc_.lock())
        return;

    if(!pc_.expired()){
        boost::shared_ptr<PointCloud> old_pc = pc_.lock();
        disconnect(old_pc.get(), SIGNAL(flagUpdate()), this,
                   SLOT(update()));
        disconnect(old_pc.get(), SIGNAL(labelUpdate()), this,
                   SLOT(update()));
    }

    pc_ = new_pc;

    // Late event cause deletion possible? Seems to help?
    if(pc_.expired())
        return;

    boost::shared_ptr<PointCloud> pc = pc_.lock();
    cloud_idx_lookup_.resize(pc->scan_width()*pc->scan_height(), -1);
    for(uint idx = 0 ; idx < pc->cloudToGridMap().size(); idx++){
        QPoint p = cloudToImageCoord(idx);
        cloud_idx_lookup_[p.x() + p.y()*pc->scan_width()] = idx;
    }

    connect(pc.get(), SIGNAL(flagUpdate()), this, SLOT(update()));
    connect(pc.get(), SIGNAL(labelUpdate()), this, SLOT(update()));


    if(gl_init_)
        resizeGL(width(), height());
    if(this->isVisible())
        update();
}

void FlatView::mouseMoveEvent(QMouseEvent * event) {
    Eigen::Vector2f delta = Eigen::Vector2f(event->x(), event->y()) - last_mouse_pos_;
    last_mouse_pos_ = Eigen::Vector2f(event->x(), event->y());
    if(pc_.expired())
        return;

    if(event->buttons()){
        delta = delta.cwiseProduct(Eigen::Vector2f(2.0f/width(), -2.0f/height()));
        //qDebug() << "Delta " << delta.x() << delta.y();
        transform_ = Eigen::Translation2f(delta) * transform_;
        update();
    }
}

void FlatView::mousePressEvent(QMouseEvent * event) {
    last_mouse_pos_ = Eigen::Vector2f(event->x(), event->y());
}

void FlatView::mouseReleaseEvent(QMouseEvent * event) {
    if (event->button() == Qt::RightButton)
    update();
}

void FlatView::wheelEvent(QWheelEvent * event) {
    auto sgn = [] (float val) {
        if(val > 0)
            return 1.0;
        else
            return -1.0;
    };

    float delta = sgn(event->delta())*1.0+(event->delta())/120.0;
    float s = 1.0f;
    if(delta > 0 && current_scale_ < 100)
        s = delta;
    else if (delta < 0 && current_scale_ > 0.01)
        s = 1.0/-delta;


    float w = width();
    float h = height();

    float tr_x = -2.0 * (event->x()/w - 0.5);
    float tr_y = -2.0 * ((height()-event->y())/h - 0.5);

    Eigen::Translation2f tr(tr_x, tr_y);

    transform_ =  tr.inverse() * Eigen::Scaling(s) * tr * transform_;

    update();
}

void FlatView::initializeGL() {
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

    //glClearColor(0.0, 0.0, 0.0, 1.0); CE();
    glClearColor(1.0, 1.0, 1.0, 1.0); CE();
    //glEnable(GL_CULL_FACE);
    glEnable(GL_MULTISAMPLE); CE();

    //
    // Load shader program
    //
    bool succ = program_.addShaderFromSourceFile(
                QGLShader::Vertex, ":/flatview.vs.glsl"); CE();
    if (!succ)
        qWarning() << "Shader compile log:" << program_.log();
    succ = program_.addShaderFromSourceFile(
                QGLShader::Fragment, ":/flatview.fs.glsl"); CE();
    if (!succ)
        qWarning() << "Shader compile log:" << program_.log();
    succ = program_.addShaderFromSourceFile(
                QGLShader::Geometry, ":/flatview.gs.glsl"); CE();
    if (!succ)
        qWarning() << "Shader compile log:" << program_.log();

    succ = program_.link(); CE();
    if (!succ) {
        qWarning() << "Could not link shader program_:" << program_.log();
        qWarning() << "Exiting...";
        abort();
    }

    //
    // Resolve uniforms
    //
    program_.bind(); CE();
    uni_sampler_ = program_.uniformLocation("sampler"); RC(uni_sampler_);
    uni_camera_ = program_.uniformLocation("camera"); RC(uni_camera_);
    program_.release();

    //
    // Set up textures & point size
    //
    glGenTextures(1, &texture_id_); CE();
    //
    // Generate vao
    //
    glGenVertexArrays(1, &vao_);
    gl_init_ = true;
}


void FlatView::paintEvent(QPaintEvent *event) {
    makeCurrent();
    if(!gl_init_)
        initializeGL();
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    boost::shared_ptr<PointCloud> pc = pc_.lock();
    boost::shared_ptr<CloudGLData> cd = gld_->cloudgldata_[pc];

    if(pc_.expired()){
        qDebug() << "Nothing to paint on flatview";
        return;
    }

    if(!pc->isVisible() && cd != nullptr){
        qDebug() << "Should be deleted now";
        gld_->cloudgldata_[pc].reset();
        return;
    }

    if(pc->isVisible() && cd == nullptr){
        gld_->cloudgldata_[pc].reset(new CloudGLData(pc));
        cd = gld_->cloudgldata_[pc];
    }

    glEnable(GL_DEPTH_TEST);


    program_.bind(); CE();
    glUniformMatrix3fv(uni_camera_, 1, GL_FALSE, getCamera().data()); CE();
    glUniform1i(uni_sampler_, 0); CE();
    glBindTexture(GL_TEXTURE_BUFFER, texture_id_); CE();
    glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, gld_->color_lookup_buffer_->bufferId()); CE();

    glBindVertexArray(vao_);CE();

    // Point intensity buffer
    IF_FAIL("buffer bind") = cd->point_buffer_->bind(); CE();
    glEnableVertexAttribArray(0); CE();
    int offset = sizeof(float)*3;
    glVertexAttribPointer(0, 1, GL_FLOAT, GL_FALSE, sizeof(float)*4,
                          reinterpret_cast<const void *>(offset)); CE();
    cd->point_buffer_->release(); CE();

    // Label buffer
    IF_FAIL("buffer bind") = cd->label_buffer_->bind(); CE();
    glEnableVertexAttribArray(1); CE(); CE();
    glVertexAttribIPointer(1, 1, GL_SHORT, 0, 0); CE();
    cd->label_buffer_->release(); CE();

    // Flag buffer
    IF_FAIL("buffer bind") = cd->flag_buffer_->bind(); CE();
    glEnableVertexAttribArray(2); CE();
    glVertexAttribIPointer(2, 1, GL_BYTE, 0, 0); CE();
    cd->flag_buffer_->release(); CE();

    // Grid buffer
    IF_FAIL("buffer bind") = cd->grid_buffer_->bind(); CE();
    glEnableVertexAttribArray(3); CE();
    glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, 0, 0); CE();
    cd->grid_buffer_->release(); CE();

    cd->draw(vao_); CE();

    glBindVertexArray(0); CE();
    glBindTexture(GL_TEXTURE_BUFFER, 0); CE();

    program_.release();

    emit pluginPaint();
    glFinish();
    swapBuffers();
}

void FlatView::resizeGL(int width, int height) {
    glViewport(0, 0, width, qMax(height, 1));

    if(pc_.expired())
        return;
    auto pc = pc_.lock();

    float war = width/float(height);
    float sar = pc->scan_width()/float(pc->scan_height());

    float cfx = sar/war;
    float cfy = (1/sar)/(1/war);

    // screen if wider than scan
    if(war <  sar){
        aspect_ = Eigen::Vector2f(2.0f/pc->scan_width(), 2.0/(cfx*pc->scan_height()));
    } else {
        aspect_ = Eigen::Vector2f(2.0/(cfy*pc->scan_width()), 2.0f/pc->scan_height());
    }

    program_.bind(); CE();   
    glUniformMatrix3fv(uni_camera_, 1, GL_FALSE, getCamera().data()); CE();
    program_.release(); CE();
}

void FlatView::rotate(float angle) {
    rotation_ += angle;
    update();
}

void FlatView::contextMenu(const QPoint &pos) {

}
