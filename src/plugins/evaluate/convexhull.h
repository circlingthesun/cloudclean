#ifndef CONVEXHULL_H
#define CONVEXHULL_H

#include <vector>
#include <functional>
#include <Eigen/Core>

std::vector<int> convex_hull(std::vector<int> idxs, std::function<Eigen::Vector2f(int)> & getPoint);

#endif
