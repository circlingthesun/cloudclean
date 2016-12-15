#ifndef LASSO_H
#define LASSO_H

#include <vector>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "glheaders.h"
#include "utilities/export.h"

class QPaintDevice;
class QGLShaderProgram;

bool UTIL_API pointInsidePolygon(std::vector<Eigen::Vector2f> &polygon,
                        Eigen::Vector2f point);

class UTIL_API Lasso
{
public:
    Lasso();

    static Eigen::Vector2i getScreenPoint(Eigen::Vector2f &p, int w, int h);
    static Eigen::Vector2f NDCPoint(Eigen::Vector2i & p, int w, int h);

    // add a new point in normalised screen coordinates
    void addScreenPoint(int x, int y, int w, int h);
    void moveLastScreenPoint(int x, int y, QPaintDevice *device);
    void addNormPoint(Eigen::Vector2f point);
    void drawLasso(int x, int y, QPaintDevice *device);
    void drawLasso(Eigen::Vector2f mouseLoc, QPaintDevice *device);
    void clear();
    bool initGL();
    std::vector<Eigen::Vector2f> getPoints();

    std::vector<Eigen::Vector2f> getPolygon();
    void getIndices(Eigen::Affine3f &proj, Eigen::Affine3f &mv,
                    pcl::PointCloud<pcl::PointXYZI> *cloud,
                    boost::shared_ptr<std::vector<int> > source_indices);
    void getIndices2D(int height, const Eigen::Affine2f & cam,
                      const std::vector<int> &cloud_to_grid_map,
                      boost::shared_ptr<std::vector<int> > source_indices);


private:
    // Points normalised
    std::vector<Eigen::Vector2f>    points_;
    QGLShaderProgram * program_;
    bool initgl_;
    GLuint vao_;
};

#endif // LASSO_H
