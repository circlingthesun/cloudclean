#ifndef COMPGEOM_H
#define COMPGEOM_H

#include <Eigen/Dense>
#include "utilities/export.h"

UTIL_API Eigen::Vector3f anyPointInPlane(Eigen::Vector4f & plane);

UTIL_API Eigen::Vector4f pointsToPlane(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3);

UTIL_API Eigen::Vector3f projPointToLine(Eigen::Vector3f p, Eigen::Vector3f a, Eigen::Vector3f b);

UTIL_API float crossProduct2D(Eigen::Vector2f a, Eigen::Vector2f b);

UTIL_API bool isPointInTriangle(Eigen::Vector2f p, Eigen::Vector2f a, Eigen::Vector2f b, Eigen::Vector2f c);

UTIL_API Eigen::Vector2f closestCoord(Eigen::Vector2f p, Eigen::Vector2f a, Eigen::Vector2f b, Eigen::Vector2f c);

UTIL_API float distToPlane(Eigen::Vector3f point, Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3);

UTIL_API float pointToLineDist(Eigen::Vector3f point,
                      Eigen::Vector3f x1,
                      Eigen::Vector3f x2);

#endif //COMPGEOM_H
