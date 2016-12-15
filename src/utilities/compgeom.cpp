#include "compgeom.h"
#include <cmath>
#include <cfloat>
#include <vector>

Eigen::Vector3f anyPointInPlane(Eigen::Vector4f & plane){
    Eigen::Vector3f point_on_plane(0,0,0);

    // Find non zero coef
    int non_zero_coef_idx = -1;
    for(int i = 0; i < 3; i++){
        if(plane[i] != 0){
            non_zero_coef_idx = i;
        }
    }
    assert(non_zero_coef_idx != -1 && "Invalid plane");
    point_on_plane[non_zero_coef_idx] = -plane.w()/plane[non_zero_coef_idx];
    return point_on_plane;
}

Eigen::Vector4f pointsToPlane(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3){
    Eigen::Vector3f normal = (p2 - p1).cross(p3 - p1);
    float d = -(normal.dot(p1));
    Eigen::Vector4f plane; plane << normal.x(), normal.y(), normal.z(), d;
    return plane;
}

Eigen::Vector3f projPointToLine(Eigen::Vector3f p, Eigen::Vector3f a, Eigen::Vector3f b){
    Eigen::Vector3f v1 = p-a;
    Eigen::Vector3f v2 = b-a;
    return (v1.dot(v2)/v2.norm())*v2;
}

float crossProduct2D(Eigen::Vector2f a, Eigen::Vector2f b){
    return a.x()*b.y() - a.y()*b.x();
}

bool isPointInTriangle(Eigen::Vector2f p, Eigen::Vector2f a, Eigen::Vector2f b, Eigen::Vector2f c){
    bool sign1 = crossProduct2D(p-a, b-a) < 0;
    bool sign2 = crossProduct2D(p-b, c-b) < 0;
    bool sign3 = crossProduct2D(p-c, a-c) < 0;
    return ((sign1 == sign2) == sign3);
}

Eigen::Vector2f closestCoord(Eigen::Vector2f p, Eigen::Vector2f a, Eigen::Vector2f b, Eigen::Vector2f c){
    std::vector<Eigen::Vector2f> points;// = {a,b,c};
    points.push_back(a); points.push_back(b); points.push_back(c);
    Eigen::Vector2f minCoord;
    float min = FLT_MAX;
    for(int i = 0; i < 3; i++){
        int i2 = (i+1)%3;
        Eigen::Vector2f v1 = p-points[i2];
        Eigen::Vector2f v2 = points[i] - points[i2];
        //project v1 onto v2
        float t = v1.dot(v2)/(v2.dot(v2));

        if(t < 0)
            t = 0;
        if(t > 1)
            t = 1;

        Eigen::Vector2f p2 = points[i] + v2*t;

        float dist = (p2 - p).norm();
        if(dist < min){
            min = dist;
            minCoord = p2;
        }

    }

    assert(min != FLT_MAX);

    return minCoord;
}

float distToPlane(Eigen::Vector3f point, Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3){
    //Eigen::Vector4f plane = getPlane(p1, p2, p3);

    // Calculate rotation quarterion
    Eigen::Vector3f triangle_normal = ((p2 - p1).cross(p3 - p1)).normalized();
    Eigen::Vector3f z_axis(0,0,1);
    float angle = -acos(triangle_normal.dot(z_axis));
    Eigen::Vector3f axis = triangle_normal.cross(z_axis).normalized();
    Eigen::Quaternion<float> q; q = Eigen::AngleAxis<float>(angle, axis);

    // rotate to xy plane
    point=q*point;
    p1=q*p1;
    p2=q*p2;
    p3=q*p3;

    // make 2d
    Eigen::Vector2f point_ = point.head<2>();
    Eigen::Vector2f p1_ = p1.head<2>();
    Eigen::Vector2f p2_ = p2.head<2>();
    Eigen::Vector2f p3_ = p3.head<2>();

    // if in triangle
    bool in_triangle = isPointInTriangle(point_, p1_, p2_, p3_);
    if(in_triangle){
        return fabs(point.z() - p1.z());
    }

    // if not in triangle find the closest point the line
    Eigen::Vector2f coord = closestCoord(point_, p1_, p2_, p3_);
    Eigen::Vector3f cpoint(coord.x(), coord.y(), p1.z());

    return (point-cpoint).norm();
}

float pointToLineDist(Eigen::Vector3f point,
                      Eigen::Vector3f x1,
                      Eigen::Vector3f x2) {
    return (x2-x1).cross(x1-point).squaredNorm()/(x2-x1).squaredNorm();
}
