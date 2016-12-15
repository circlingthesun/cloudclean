/*
 * Software License Agreement (BSD License)
 *
 *  CloudClean
 *  Copyright (c) 2013, Rickert Mulder
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Rickert Mulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef CLOUDCLEAN_SRC_CLOUDCLEAN_POINTPICKER_H_
#define CLOUDCLEAN_SRC_CLOUDCLEAN_POINTPICKER_H_

#include <Eigen/Dense>
#include <pcl/octree/octree.h>
#include <set>
#include "utilities/export.h"

class Camera;
class PointCloud;
class Layer;

UTIL_API void screenToRay(int x, int y, int win_width, int win_height, const Eigen::Affine3f& mv,
                              const Eigen::Affine3f& projPointToLine,
                              Eigen::Vector3f& p1,
                              Eigen::Vector3f& p2,
                              float near = 0.0f,
                              float far = 1.0f);

UTIL_API void screenToRay2(int x, int y, int win_width, int win_height, const Eigen::Affine3f& mv,
                              const Eigen::Affine3f& projPointToLine,
                              Eigen::Vector3f& p1,
                              Eigen::Vector3f& p2,
                              float near = 0.0f,
                              float far = 1.0f);

UTIL_API void screenToWorld(int x1, int y1, int x2, int y2, int win_width, int win_height,
                              const Eigen::Affine3f& mv,
                              const Eigen::Affine3f& proj,
                              Eigen::Vector3f& p1,
                              Eigen::Vector3f& p2,
                              float z);

UTIL_API int pick(int win_x, int win_y, int win_width, int win_height, float max_dist,
        const Eigen::Affine3f& proj, const Eigen::Affine3f& cam_mv,
        boost::shared_ptr<PointCloud> pc,
        std::set<uint8_t> labels = std::set<uint8_t>());

#endif  // CLOUDCLEAN_SRC_CLOUDCLEAN_POINTPICKER_H_
