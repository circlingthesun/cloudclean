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

#include <cmath>
#include <cfloat>
#include <vector>

#include <QDebug>

#include "utilities/compgeom.h"
#include "utilities/pointpicker.h"

#include "model/pointcloud.h"
#include "model/layer.h"
#include "gui/camera.h"

#include "glheaders.h"
#include <GL/glu.h>

void screenToRay(int x, int y, int win_width, int win_height,
                              const Eigen::Affine3f& mv,
                              const Eigen::Affine3f& proj,
                              Eigen::Vector3f& p1,
                              Eigen::Vector3f& p2,
                              float near,
                              float far) {
    double dX, dY, dZ;

    // Convert to double matrices
    double mvmatrix[16];
    double projmatrix[16];
    for (int i = 0; i < 16; ++i) {
        projmatrix[i] = proj.data()[i];
        mvmatrix[i] = mv.data()[i];
    }

    // Fetch current viewport
    int viewport[4] = {0, 0, win_width, win_height};

    // Invert y axis
    y = win_height - y;

    // Unproject
    gluUnProject(x, y, near, mvmatrix, projmatrix, viewport, &dX, &dY, &dZ);
    p1 = Eigen::Vector3f(static_cast<float>(dX), static_cast<float>(dY),
                         static_cast<float>(dZ));
    gluUnProject(x, y, far, mvmatrix, projmatrix, viewport, &dX, &dY, &dZ);
    p2 = Eigen::Vector3f(static_cast<float>(dX), static_cast<float>(dY),
                         static_cast<float>(dZ));
}

//Eigen::Vector3f unproject(int x, int y, int win_width, int win_height,
//                          const Eigen::Affine3f& mv,
//                          const Eigen::Affine3f& proj,
//                          float z){

//    float y1 = (win_height - y)/float(win_height) *2 -1;
//    float x1 = x/float(win_width)*2 - 1;
//    float z1 = (z-0.5) * 2;

//    Eigen::Vector3f p = (mv * proj).inverse() * Eigen::Vector3f(x1, y1, z1);
//    p = p*p.w();
//    qDebug() << "yay";
//    return p;
//}

void screenToRay2(int x, int y, int win_width, int win_height,
                              const Eigen::Affine3f& mv,
                              const Eigen::Affine3f& proj,
                              Eigen::Vector3f& p1,
                              Eigen::Vector3f& p2,
                              float near,
                              float far) {
    double dX, dY, dZ;

    // Convert to double matrices
    double mvmatrix[16];
    double projmatrix[16];
    for (int i = 0; i < 16; ++i) {
        projmatrix[i] = proj.data()[i];
        mvmatrix[i] = mv.data()[i];
    }

    // Fetch current viewport
    int viewport[4] = {0, 0, win_width, win_height};

    // Invert y axis
    y = win_height - y;

    // Unproject
    gluUnProject(x, y, near, mvmatrix, projmatrix, viewport, &dX, &dY, &dZ);
    p1 = Eigen::Vector3f(static_cast<float>(dX), static_cast<float>(dY),
                         static_cast<float>(dZ));
    gluUnProject(x, y, far, mvmatrix, projmatrix, viewport, &dX, &dY, &dZ);
    p2 = Eigen::Vector3f(static_cast<float>(dX), static_cast<float>(dY),
                         static_cast<float>(dZ));
}

void screenToWorld(int x1, int y1, int x2, int y2, int win_width, int win_height,
                              const Eigen::Affine3f& mv,
                              const Eigen::Affine3f& proj,
                              Eigen::Vector3f& p1,
                              Eigen::Vector3f& p2,
                              float z) {
    double dX, dY, dZ;

    // Convert to double matrices
    double mvmatrix[16];
    double projmatrix[16];
    for (int i = 0; i < 16; ++i) {
        projmatrix[i] = proj.data()[i];
        mvmatrix[i] = mv.data()[i];
    }

//    p1 = unproject(x1, y1, win_width, win_height, mv, proj, z);
//    p2 = unproject(x2, y2, win_width, win_height, mv, proj, z);

    // Fetch current viewport
    int viewport[4] = {0, 0, win_width, win_height};

    // Invert y axis
    y1 = win_height - y1;
    y2 = win_height - y2;

    // Unproject
    gluUnProject(x1, y1, z, mvmatrix, projmatrix, viewport, &dX, &dY, &dZ);
    p1 = Eigen::Vector3f(static_cast<float>(dX), static_cast<float>(dY),
                         static_cast<float>(dZ));
    gluUnProject(x2, y2, z, mvmatrix, projmatrix, viewport, &dX, &dY, &dZ);
    p2 = Eigen::Vector3f(static_cast<float>(dX), static_cast<float>(dY),
                         static_cast<float>(dZ));
}

int pick(int win_x, int win_y, int win_width, int win_height, float max_dist,
        const Eigen::Affine3f& proj, const Eigen::Affine3f& cam_mv,
        boost::shared_ptr<PointCloud> pc,
        std::set<uint8_t> labels) {

    Eigen::Vector3f p1, p2;

    Eigen::Affine3f mv = cam_mv * pc->modelview();

    screenToRay(win_x, win_y, win_width, win_height, mv, proj, p1, p2);

    int min_index = -1;
    float min_val = FLT_MAX;

    //vector<int> candidates;

    // Need to narrow down candidates
    Eigen::Vector3f& origin = p1;
    Eigen::Vector3f direction = p2-p1;
    std::vector<int> intercept_indices;

    // this may block
    const Octree::Ptr octree_ptr = pc->octree();
    octree_ptr->getIntersectedVoxelIndices(origin, direction, intercept_indices);

    // get ray point on far plane
    Eigen::Vector3f ray_point = proj * cam_mv * (p1+p2)/2;
    ray_point[2] = 0.0f;

    // Find point
    for (int i : intercept_indices) {

        // Skip points not in layer
        if(labels.size() != 0){
            bool in_layer = false;
            for(auto label : labels){
                if(pc->labels_[i] == label){
                    in_layer = true;
                    break;
                }
            }
            if(!in_layer)
                continue;
        }

        pcl::PointXYZRGB & p = pc->points[i];

        Eigen::Vector3f query_point = proj * mv * p.getVector3fMap();
        query_point[2] = 0.0f;

        float dist = (query_point - ray_point).norm();
        if (dist < min_val) {
            min_index = i;
            min_val = dist;
        }
    }

    if (min_index == -1)
        qDebug("Cannot find closest point\n");
    return min_index;
}
