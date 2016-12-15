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

#include "gui/camera.h"
#include <cmath>
#include <mutex>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <QDebug>
#include <QTimer>
#include <iostream>

using Eigen::Vector3f;
using Eigen::Vector2f;
using Eigen::AngleAxis;

Camera::Camera() {
    //mtx_ = new std::mutex();
    fov_current_ = 60.0f;
    fov_future_ = 60.0f;
    aspect_ = 1.0f;
    depth_near_ = 1.0f;
    depth_far_ = 1000.f;

    roll_correction_ = true;
    always_update_ = false;

    rotation_current_ = AngleAxis<float>(0, Vector3f(1.0f, 0.0f, 0.0f));
    rotation_future_ = AngleAxis<float>(0, Vector3f(1.0f, 0.0f, 0.0f));

    translation_current_ = Vector3f(0, 0, 0);
    translation_future_ = Vector3f(0, 0, 0);

    translation_speed_ = 1;

    projection_dirty_ = true;

    timer_ = new QTimer();
    timer_->connect(timer_, &QTimer::timeout, [&] () {
        Eigen::Vector3f trans_diff = translation_future_ - translation_current_;
        bool good_enough_rotation = rotation_current_.isApprox(rotation_future_);
        float fov_diff = fov_future_ - fov_current_;

        if(trans_diff.norm() > 1e-4 || !good_enough_rotation || fabs(fov_diff) > 1e-10) {

            translation_current_ = translation_current_ + trans_diff * 0.5;
            rotation_current_ = rotation_current_.slerp(0.5, rotation_future_);
            fov_current_ = fov_future_ * 0.5 + fov_current_ * 0.5;

            if(fabs(fov_diff) > 1e-10)
                projection_dirty_ = true;

            emit updated();
        }
        else {
            if(always_update_)
                emit updated();
            translation_speed_ = 1;
        }


    });

    timer_->start(1000/60);
}

Camera::~Camera() {
    //delete mtx_;
}

double angle (Vector3f a, Vector3f b){
    double dotp = a.dot(b);
    if(dotp >= 1 || dotp <= -1){
        return 0.0;
    }
    return acos(dotp);
}

double normalizeRad(double angle){
    if(angle < -M_PI_2){
        return -M_PI_2 - (angle + M_PI_2);
    } else if(angle > M_PI_2) {
        return M_PI_2 - (angle - M_PI_2);
    }
    return angle;
}

Eigen::Quaternionf rollcorrect(Eigen::Quaternionf rotation){
    Vector3f local_z_axis = rotation.inverse() * -Vector3f::UnitZ();
    Vector3f local_x_axis = rotation.inverse() * Vector3f::UnitX();
    Vector3f local_y_axis = rotation.inverse() * Vector3f::UnitY();

    Vector3f z_proj_zx = local_z_axis; z_proj_zx(1) = 0; z_proj_zx.normalize();
    Vector3f x_proj_zx = local_x_axis; x_proj_zx(1) = 0; x_proj_zx.normalize();
    Vector3f y_proj_zx = local_y_axis; y_proj_zx(1) = 0; y_proj_zx.normalize();

    double pitch = angle(local_z_axis, z_proj_zx);
    double roll = angle(local_x_axis, x_proj_zx);

    int proj_side = local_x_axis.cross(x_proj_zx).dot(local_z_axis) > 0 ? -1 : 1;
    int side_up = local_y_axis.dot(Vector3f::UnitY()) > 0 ? 1 : -1;

    if(side_up == -1){
        roll = M_PI - roll;
    }

    roll = roll * proj_side * side_up;

    double norm_pitch = normalizeRad(pitch);
    double correction_factor = (M_PI_2 - fabs(norm_pitch)) / M_PI_2;
    correction_factor = pow(correction_factor - 0.5, 1);

    if(pitch > 0.7){
        correction_factor = 0;
    }

    AngleAxis<float> roll_correction(correction_factor*-roll, Vector3f::UnitZ());
    rotation = roll_correction * rotation;

    return rotation;
}

void Camera::setFoV(float fov) {
    fov_future_ = fov;
    projection_dirty_ = true;
}

void Camera::setAspect(float aspect) {
    aspect_ = aspect;
    projection_dirty_ = true;
}

void Camera::setDepthRange(float near, float far) {
    depth_near_ = near;
    depth_far_ = far;
    projection_dirty_ = true;
}

void Camera::setPosition(const Eigen::Vector3f& pos) {
    translation_future_ = pos;
}


void Camera::recalculateProjectionMatrix() {
    // Code from Mesa project, src/glu/sgi/libutil/project.c
    projection_matrix_.setIdentity();
    float radians = fov_current_ / 2 * M_PI / 180;

    float deltaZ = depth_far_ - depth_near_;
    float sine = sin(radians);
    if ((deltaZ == 0) || (sine == 0) || (aspect_ == 0)) {
        return;
    }
    float cotangent = cos(radians) / sine;

    projection_matrix_(0, 0) = cotangent / aspect_;
    projection_matrix_(1, 1) = cotangent;
    projection_matrix_(2, 2) = -(depth_far_ + depth_near_) / deltaZ;
    projection_matrix_(3, 2) = -1;
    projection_matrix_(2, 3) = -2 * depth_near_ * depth_far_ / deltaZ;
    projection_matrix_(3, 3) = 0;
    projection_dirty_ = false;
}

Eigen::Affine3f Camera::modelviewMatrix() {
    return rotation_current_  * Eigen::Translation3f(translation_current_) * Eigen::Affine3f::Identity();
}

Eigen::Affine3f Camera::projectionMatrix() const {
    if (projection_dirty_) {
        const_cast<Camera*>(this)->recalculateProjectionMatrix();
    }
    return projection_matrix_;
}

void Camera::translate(const Eigen::Vector3f& pos) {
    translation_future_ = Eigen::Translation3f(rotation_current_.inverse() * (translation_speed_ * pos)) * translation_current_;

    if(translation_speed_ < 10){
        translation_speed_ *= 1.1f; // If succesive tranlations are performed, speed things up
    }

    emit modified();
}

void Camera::setRotate3D(const Eigen::Vector3f& rot) {
    Eigen::AngleAxis<float> aaZ(rot.z(), Eigen::Vector3f::UnitZ());
    Eigen::AngleAxis<float> aaY(rot.y(), Eigen::Vector3f::UnitY());
    Eigen::AngleAxis<float> aaX(rot.x(), Eigen::Vector3f::UnitX());

    rotation_future_ = aaZ * aaY * aaX;
    emit modified();
}

void Camera::rotate2D(float x, float y) {
    Vector2f rot = Vector2f(x, y);

    AngleAxis<float> rotX(rot.x(), Vector3f::UnitY()); // look left right
    AngleAxis<float> rotY(rot.y(), Vector3f::UnitX()); // look up down

    rotation_future_ = (rotX * rotY) * rotation_current_;

    if(roll_correction_){
        rotation_future_ = rollcorrect(rotation_future_);
    }

    emit modified();
}

void Camera::rotate3D(float _yaw, float _pitch, float _roll) {

    AngleAxis<float> rotX(_yaw, Vector3f::UnitY()); // look left right
    AngleAxis<float> rotY(_pitch, Vector3f::UnitX()); // look up down

    rotation_future_ = (rotX * rotY) * rotation_future_;

    auto clamp = [] (double num, double low, double high) {
        if (num > high)
            return high;
        if (num < low)
            return low;
        return num;
    };

    Eigen::Matrix3f r = rotation_future_.toRotationMatrix();
    double roll = -atan2(r(0,2), r(1, 2));
    double pitch = acos(r(2,2));
    //double yaw = atan2(r(2, 0), r(2, 1));


    Vector3f dir = rotation_future_ * Vector3f::UnitZ();
    double dotp = dir.dot(Vector3f::UnitZ());

    double sign = dir.dot(Vector3f::UnitY()) > 0 ? 1.0 : -1.0;
    double angle = sign * acos(dotp);

/*
    qDebug() << "Y angle" << angle;
    qDebug() << "Y angle (DEG)" << (angle/M_PI) * 180;
    qDebug() << "Roll" << roll;
    qDebug() << "Pitch" << pitch;
    qDebug() << "Yaw" << yaw;
*/
    double correction_factor = 1.0 - fabs(pitch-M_PI/2)/(M_PI/2);
    correction_factor = -0.5 + 1.5 * correction_factor;
    correction_factor = clamp(correction_factor, 0, 1);

    if(angle < 0)
        correction_factor = 0;

    //qDebug() << "Correction factor:" << correction_factor;

    AngleAxis<float> roll_correction(correction_factor*-roll, Vector3f::UnitZ());
    rotation_future_ = roll_correction * rotation_future_;
    rotation_future_.normalize();

    emit modified();
}

void Camera::adjustFov(int val) {
    // Mouse seems to move in increments of 120
    val = -val/60.0f;
    if (fov_future_ + val < 100.0f && fov_future_ + val > 2.0f)
        setFoV(fov_future_ + val);

}

void Camera::birds_eye() {
    rotation_future_ = AngleAxis<float>(0, Vector3f(1, 0, 0));
    translation_future_ = 20 * -Vector3f::UnitZ();
}

void Camera::toggleRollCorrection(){
    roll_correction_ = !roll_correction_;
}

void Camera::toggleRollCorrection(bool on){
    roll_correction_ = on;
}
