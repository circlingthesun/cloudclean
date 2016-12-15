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

#ifndef CLOUDCLEAN_SRC_CLOUDCLEAN_PICKER_H_
#define CLOUDCLEAN_SRC_CLOUDCLEAN_PICKER_H_

#include "glheaders.h"
#include <functional>
#include <QObject>
#include <QGLShaderProgram>
#include "utilities/export.h"

class QEvent;
class QMouseEvent;
class GLWidget;
class FlatView;
class CloudList;

static bool yes = true;

class UTIL_API Picker : public QObject{
    Q_OBJECT
 public:
    Picker(GLWidget *glwidget, FlatView *flatview, CloudList * cl, std::function<void (int)> callback = nullptr, bool *enabled = &yes);
    ~Picker();
    uint renderPick3d(int x, int y);
    uint renderPick3d_(int x, int y);
    bool eventFilter(QObject *object, QEvent *event);
    bool is3d();

 private:
    bool mouseReleaseEvent(QMouseEvent * event);

 private:
    QGLShaderProgram program_3d_;
    GLWidget * glwidget_;
    FlatView * flatview_;
    CloudList * cl_;
    int idx_;
    std::function<void(int)> callback_;
    bool * enabled_;

    // gl stuff
    GLuint texture_id_;
    GLuint picking_texture_id_;
    GLuint depth_texture_id_;
    GLuint vao_;
    GLuint fbo_;
    GLuint color_rbo_;
    GLuint depth_rbo_;
    int uni_sampler_;
    int uni_projection_;
    int uni_modelview_;
};

#endif  // CLOUDCLEAN_SRC_CLOUDCLEAN_PICKER_H_
