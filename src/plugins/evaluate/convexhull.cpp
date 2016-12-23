#include "plugins/evaluate/convexhull.h"
#include <Eigen/Core>

// Implementation of Andrew's monotone chain 2D convex hull algorithm.
// Asymptotic complexity: O(n log n).
// Practical performance: 0.5-1.0 seconds for n=1000000 on a 1GHz machine.
#include <algorithm>
#include <vector>

// 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
// Returns a positive value, if OAB makes a counter-clockwise turn,
// negative for clockwise turn, and zero if the points are collinear.
float cross(const Eigen::Vector2f O, const Eigen::Vector2f A, const Eigen::Vector2f B) {
    return (long)(A.x() - O.x()) * (B.y() - O.y()) - (long)(A.y() - O.y()) * (B.x() - O.x());
}

// Returns a list of points on the convex hull in counter-clockwise order.
// Note: the last point in the returned list is the same as the first one.
std::vector<int> convex_hull(std::vector<int> idxs, std::function<Eigen::Vector2f(int)> & getPoint){
    int n = idxs.size(), k = 0;
    std::vector<int> hull(2*n);

    // Sort points lexicographically
    sort(idxs.begin(), idxs.end(), [&getPoint](int a_, int b_){
        Eigen::Vector2f a = getPoint(a_);
        Eigen::Vector2f b = getPoint(b_);
        return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
    });

    // Build lower hull
    for (int i = 0; i < n; ++i) {
        while (k >= 2 && cross(getPoint(hull[k-2]), getPoint(hull[k-1]), getPoint(idxs[i])) <= 0) k--;
        hull[k++] = idxs[i];
    }

    // Build upper hull
    for (int i = n-2, t = k+1; i >= 0; i--) {
        while (k >= t && cross(getPoint(hull[k-2]), getPoint(hull[k-1]), getPoint(idxs[i])) <= 0) k--;
        hull[k++] = idxs[i];
    }

    hull.resize(k);
    return hull;
}
