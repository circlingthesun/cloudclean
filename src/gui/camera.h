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

#ifndef CLOUDCLEAN_SRC_CLOUDCLEAN_CAMERA_H_
#define CLOUDCLEAN_SRC_CLOUDCLEAN_CAMERA_H_

#include <Eigen/Geometry>
#include <QObject>
#include "gui/export.h"

class QTimer;
class GLWidget;

namespace std {
    class mutex;
}

class GUI_API Camera : public QObject {
    Q_OBJECT

 public:
    Camera();

    ~Camera();

    void setViewport(int x, int y, int width, int height);

    void setFoV(float fov);

    void setAspect(float aspect);

    void setDepthRange(float near, float far);

    inline float getNear(){
        return depth_near_;
    }

    inline float getFar(){
        return depth_far_;
    }

    void setPosition(const Eigen::Vector3f& pos);
    void setPosition(float x, float y, float z)  {
        setPosition(Eigen::Vector3f(x, y, z));
    }

    Eigen::Vector3f getPosition(){
        return translation_future_;
    }

    Eigen::Quaternion<float> getRotation(){
        return rotation_future_;
    }

    void setRotation(Eigen::Quaternion<float> & rot){
        rotation_future_ = rot;
    }

    Eigen::Affine3f modelviewMatrix();
    Eigen::Affine3f projectionMatrix() const;

    void translate(const Eigen::Vector3f& pos);
    void translate(float x, float y, float z)  {
        translate(Eigen::Vector3f(x, y, z));
    }

    void setRotate3D(const Eigen::Vector3f& rot);
    void setRotate3D(float yaw, float pitch, float roll) {
        setRotate3D(Eigen::Vector3f(yaw, pitch, roll));
    }

    void rotate2D(float x, float y);
    void rotate3D(float _yaw, float _pitch, float _roll);
    void adjustFov(int val);

    void alwaysUpdate(bool on){
        always_update_ = on;
    }

    bool roll_correction(){
        return roll_correction_;
    }

 public slots:
    void birds_eye();
    void toggleRollCorrection();
    void toggleRollCorrection(bool on);

 signals:
    void updated();
    void modified();

 private:
    void recalculateProjectionMatrix();

 private:
    Eigen::Quaternion<float> rotation_current_;
    Eigen::Quaternion<float> rotation_future_;

    Eigen::Vector3f translation_current_;
    Eigen::Vector3f translation_future_;

    float fov_current_;
    float fov_future_;

    float aspect_, depth_near_, depth_far_;

    Eigen::Affine3f projection_matrix_;

    bool projection_dirty_;
    bool modelview_dirty_;

    QTimer *timer_;

    float translation_speed_;

    //std::mutex * mtx_;
    bool roll_correction_;
    bool always_update_;
    friend class GLWidget;
};

#endif  // CLOUDCLEAN_SRC_CLOUDCLEAN_CAMERA_H_
