#include "lasso.h"

#include <GL/glu.h>
#include <math.h>

#include <QDebug>
#include <QPen>
#include <QGLShaderProgram>
#include <QGLBuffer>

#include <time.h>
#include <cstdlib>

Lasso::Lasso()
{
    initgl_ = false;
}

Eigen::Vector2i Lasso::getScreenPoint(Eigen::Vector2f & p, int w, int h) {
    float x = (p.x()+1)*(w/2.0f);
    float y = (-p.y()+1)*(h/2.0f);
    return Eigen::Vector2i(x, y);
}

Eigen::Vector2f Lasso::NDCPoint(Eigen::Vector2i & p, int w, int h) {
    float fx = 2.0*float(p.x())/w-1.0f;
    float fy = -2.0*float(p.y())/h+1.0f;
    return Eigen::Vector2f(fx, fy);
}

int inline side(float a){
    if(a < -1e-6)
        return -1;
    if(a > 1e-6)
        return 1;
    return 0;
}

bool isPointOnLineSegment(Eigen::Vector2f lineA,
                        Eigen::Vector2f lineB,
                        Eigen::Vector2f pointC)
{
    float lineLength = (lineA - lineB).norm();
    float viaPoint = (lineA - pointC).norm() + (lineB - pointC).norm();
    return fabs(viaPoint - lineLength) < 1e-6;
}

bool oppositeSides(Eigen::Vector2f lineStart, Eigen::Vector2f lineEnd, Eigen::Vector2f pointA, Eigen::Vector2f pointB)
{
    Eigen::Vector2f lineDir = lineStart-lineEnd;
    Eigen::Vector2f pointDir1 = lineStart-pointA;
    Eigen::Vector2f pointDir2 = lineStart-pointB;

    float cross1 = lineDir.x()*pointDir1.y() - lineDir.y()*pointDir1.x();
    float cross2 = lineDir.x()*pointDir2.y() - lineDir.y()*pointDir2.x();

    int sideC = side(cross1);
    int sideD = side(cross2);
    return sideC != sideD;
}

bool intersects(Eigen::Vector2f line1Start,
                Eigen::Vector2f line1End,
                Eigen::Vector2f line2Start,
                Eigen::Vector2f line2End) {
    if(oppositeSides(line1Start, line1End, line2Start, line2End) &&
            oppositeSides(line2Start, line2End, line1Start, line1End))
        return true;
    return false;
}

Eigen::Vector2f randomLineSegment(Eigen::Vector2f & origin){
    float rand_angle = 2.0f*M_PI*(rand() % 10000)/10000.0f;
    Eigen::Vector2f endPoint;
    endPoint << (10000.0f*cos(rand_angle) + origin.x()),
            (10000.0f*sin(rand_angle) + origin.y());
    return endPoint;
}

bool pointInsidePolygon(std::vector<Eigen::Vector2f> & polygon,
                        Eigen::Vector2f point){

    while(true) {
        Eigen::Vector2f endPoint = randomLineSegment(point);

        for(uint i = 0; i < polygon.size(); ++i)
            if(isPointOnLineSegment(point, endPoint, polygon[i]))
                continue;

        int hits = 0;

        for(uint i = 0; i < polygon.size(); ++i){
            if(intersects(polygon[i], polygon[(i + 1) % polygon.size()], point, endPoint)){
                ++hits;
            }
        }
        return (hits % 2 == 1);
    }
}




void Lasso::addNormPoint(Eigen::Vector2f point) {
    points_.push_back(point);
}

void Lasso::moveLastScreenPoint(int x, int y, QPaintDevice *device) {
    float fx = 2.0*float(x)/device->width()-1.0f;
    float fy = -2.0*float(y)/device->height()+1.0f;
    if(points_.size() != 0)
        points_.pop_back();
    addNormPoint(Eigen::Vector2f(fx, fy));
}

void Lasso::addScreenPoint(int x, int y, int w, int h) {
    float fx = 2.0*float(x)/w-1.0f;
    float fy = -2.0*float(y)/h+1.0f;
    addNormPoint(Eigen::Vector2f(fx, fy));
}

inline QPointF screenPoint(Eigen::Vector2f & p, int width, int height){
    float x = (p.x()+1)*(width/2.0f);
    float y = (-p.y()+1)*(height/2.0f);
    return QPointF(x, y);
}

void Lasso::drawLasso(int x, int y, QPaintDevice *device) {
    float fx = 2.0*float(x)/device->width()-1.0f;
    float fy = -2.0*float(y)/device->height()+1.0f;
    drawLasso(Eigen::Vector2f(fx, fy), device);
}

void Lasso::drawLasso(Eigen::Vector2f mouseLoc, QPaintDevice * device){
    if(!initgl_){
        initGL();
        initgl_ = true;
    }

    if(points_.size() == 1)
        return;

    // create buffer
    QGLBuffer buff(QGLBuffer::VertexBuffer); CE();
    buff.create(); CE();
    buff.bind(); CE();
    size_t b_size = 3*sizeof(float)*(points_.size()+1);
    buff.allocate(b_size); CE();

    float third_component = -1;

//    buff.write(0, mouseLoc.data(), 2*sizeof(float));
//    buff.write(2*sizeof(float), &third_component, sizeof(float));

    for(uint i = 0; i < points_.size(); i++){
        buff.write((i+0)*3*sizeof(float), points_[i].data(), 2*sizeof(float));
        buff.write((i+0)*3*sizeof(float) + 2*sizeof(float), &third_component, sizeof(float));
    }



    // bind buffer
    program_->bind();
    glBindVertexArray(vao_);


    glEnableVertexAttribArray(0); CE();
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0); CE();

    int uni_projection = program_->uniformLocation("proj"); RC(uni_projection);
    int uni_modelview = program_->uniformLocation("mv"); RC(uni_modelview);
    int uni_color = program_->uniformLocation("colour"); RC(uni_color);

    Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();

    glUniformMatrix4fv(uni_modelview, 1, GL_FALSE,
                       identity.data());CE();
    glUniformMatrix4fv(uni_projection, 1, GL_FALSE,
                       identity.data());CE();

    float color[] = {0.0f, 1.0f, 0.0f};
    glUniform3fv(uni_color, 1, color); CE();

    glLineWidth(2.0f); CE();
    glDrawArrays(GL_LINE_LOOP, 0, points_.size()); CE();

    buff.release(); CE();
    glBindVertexArray(0); CE();
    program_->release();
}

void Lasso::clear(){
    points_.clear();
}

bool Lasso::initGL(){
    program_ = new QGLShaderProgram();
    bool succ = program_->addShaderFromSourceFile(
                QGLShader::Vertex, ":/basic.vert"); CE();
    if (!succ) qWarning() << "Shader compile log:" << program_->log();
    succ = program_->addShaderFromSourceFile(
                QGLShader::Fragment, ":/basic.frag"); CE();
    if (!succ) qWarning() << "Shader compile log:" << program_->log();
    succ = program_->link(); CE();
    if (!succ) {
        qWarning() << "Could not link shader program_:" << program_->log();
        qWarning() << "Exiting...";
        abort();
    }

    glGenVertexArrays(1, &vao_);
    return succ;
}

std::vector<Eigen::Vector2f> Lasso::getPolygon(){
    return points_;
}

std::vector<Eigen::Vector2f> Lasso::getPoints() {
    return points_;
}

void Lasso::getIndices(Eigen::Affine3f & proj,
                        Eigen::Affine3f & mv,
                pcl::PointCloud<pcl::PointXYZRGB> * cloud,
                boost::shared_ptr<std::vector<int> > source_indices){

    Eigen::Affine3f ndc_mat = proj * mv;

    auto copyd = [] (float * i, double * o) {
        for(int idx = 0; idx < 16; idx++){
            o[idx] = i[idx];
        }
    };

    double mv1 [16];
    double proj1 [16];
    copyd(proj.data(), proj1);
    copyd(mv.data(), mv1);

    int view [4] = {0, 0, 1000, 1000};

    auto inside_lasso = [&] (pcl::PointXYZRGB & p) {
        /// project point
        Eigen::Vector4f p_4 = p.getVector4fMap();
        p_4 = ndc_mat.matrix() * p_4;

        // Limit to front of camera
        if(p_4.z() < 0.0f)
            return false;

        double wx, wy, wz;
        gluProject(p.x, p.y, p.z, mv1, proj1, view, &wx, &wy, &wz);
        wx = (wx/500.0f)-1.0f, wy = (wy/500.0f)-1.0f;

        Eigen::Vector2f p_2(wx, wy);

        /// do lasso test
        return  pointInsidePolygon(points_, p_2);
    };

    if(source_indices->size() == 0) {
        for (uint idx = 0; idx < cloud->size(); idx++) {
            if(inside_lasso(cloud->points[idx])) {
                source_indices->push_back(idx);
            }
        }
    }

}

void Lasso::getIndices2D(int height, const Eigen::Affine2f & cam,
                const std::vector<int> & cloud_to_grid_map,
                boost::shared_ptr<std::vector<int> > source_indices) {

    auto inside_lasso = [&] (int idx) {
        /// do lasso test
        int i = cloud_to_grid_map[idx];

        Eigen::Vector2f point = cam * Eigen::Vector2f(i/height, i%height);

        return  pointInsidePolygon(points_, point);
    };

    if(source_indices->size() == 0) {
        for (uint idx = 0; idx < cloud_to_grid_map.size(); idx++) {
            if(inside_lasso(idx)) {
                source_indices->push_back(idx);
            }
        }
    }

}
