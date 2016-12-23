#ifndef ALGO_H
#define ALGO_H

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

bool snake_iteration(boost::shared_ptr<std::vector<float> > img,
               int w,
               int h,
               std::vector<Eigen::Vector2i> & points,
               int win_w,
               int win_h,
               float alpha,
               float beta,
               float gamma,
               float delta);

#endif  // ALGO_H
