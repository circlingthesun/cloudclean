#include "plugins/${lower_name}/${lower_name}.h"
#include <QDebug>
#include <QEvent>
#include <QKeyEvent>
#include <QAction>
#include <QTabWidget>
#include <QApplication>
#include <QToolBar>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QSpacerItem>
#include <QStackedWidget>
#include <QSlider>
#include <QButtonGroup>
#include <QDockWidget>
#include <QPushButton>
#include <QGridLayout>
#include <QCheckBox>
#include <boost/make_shared.hpp>
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "utilities/picker.h"
#include "utilities/pointpicker.h"
#include "commands/select.h"
#include "pluginsystem/core.h"

QString ${camel_name}::getName(){
    return "${name} Tool";
}

void ${camel_name}::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    ll_ = core_->ll_;
    glwidget_ = core_->mw_->glwidget_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;

    picker_ = new Picker(glwidget_, cl_);

    enable_ = new QAction(QIcon(":/${icon}"), "Enable ${name}", 0);
    enable_->setCheckable(true);
    enable_->setChecked(false);

    last_picked_point_ = -1;
    last_rad_ = 0;
    is_enabled_ = false;
    radius_ = 0.5f;

    connect(enable_, SIGNAL(triggered()), this, SLOT(enable()));
    connect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));

    mw_->addMenu(enable_, "Edit");
    mw_->toolbar_->addAction(enable_);

    settings_ = new QWidget();
    QVBoxLayout * layout = new QVBoxLayout(settings_);
    settings_->setLayout(layout);

    mw_->tooloptions_->addWidget(settings_);

    QSlider * slider = new QSlider(settings_);
    slider->setOrientation(Qt::Horizontal);
    slider->setRange(1, 300);
    slider->setSingleStep(1);
    slider->setToolTip("Radius in cm");
    slider->setValue(radius_*100);
    slider->setTickPosition(QSlider::TicksBelow);
    connect(slider, &QSlider::valueChanged, [this] (int val){
        radius_ = val/100.0;
    });


    QLabel * l = new QLabel("Radius:", settings_);
    layout->addWidget(l);
    layout->addWidget(slider);

    l = new QLabel("Depth correct:", settings_);
    layout->addWidget(l);

    QCheckBox * cb = new QCheckBox(settings_);
    depth_adjust_ = true;
    cb->setChecked(depth_adjust_);
    layout->addWidget(cb);
    cb->connect(cb, &QCheckBox::toggled, [this] (bool on) {
        depth_adjust_ = on;
    });

    layout->addItem(new QSpacerItem(0, 0, QSizePolicy::Maximum, QSizePolicy::Maximum));
    layout->addStretch();


}

void ${camel_name}::cleanup(){
    disable();
    mw_->removeMenu(enable_, "Edit");
    mw_->toolbar_->removeAction(enable_);
    mw_->tooloptions_->removeWidget(settings_);
    disconnect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));
    disconnect(enable_, SIGNAL(triggered()), this, SLOT(enable()));
}

float getZ(Eigen::Vector3f p, Eigen::Affine3f mv, Eigen::Affine3f proj, int w, int h){

    double wx, wy, wz;
    int viewport[4] = {0, 0, w, h};

    double mv1 [16];
    double proj1 [16];

    const float * mvt = mv.data();
    const float * projt = proj.data();

    for(int i = 0; i < 16; i++){
        mv1[i] = mvt[i];
        proj1[i] = projt[i];
    }

    gluProject(p.x(), p.y(), p.z(),
        mv1,
        proj1,
        viewport,
        &wx, &wy, &wz);

    return wz;
}


void ${camel_name}::select2D(int x, int y){
    boost::shared_ptr<PointCloud> pc = cl_->active_;

    Eigen::Vector3f coord;
    coord << 2.0f* (x/float(flatview_->width()) - 0.5),
        -2.0f* (y/float(flatview_->height()) - 0.5), 1;
    coord = flatview_->getCamera().inverse() * coord;

    coord[1] = cl_->active_->scan_height()-coord[1];

    boost::shared_ptr<std::vector<int> > indices;
    indices.reset(new std::vector<int>());

    int rad = radius_ * 500;

    for(int x = -rad; x < rad; x++){
        for(int y = -rad; y < rad; y++){
            if(x*x + y*y > (rad)*(rad) )
                continue;

            int idx = flatview_->imageToCloudIdx(int(coord.x() + x + 0.5),
                                      int(coord.y() + y + 0.5), pc);
            if (idx < 0) {
                if(idx < -1)
                    qDebug() << "Bug! Idx < -1 : idx = " << idx;
                continue;
            }

            indices->push_back(idx);
        }
    }

    bool negative_select = QApplication::keyboardModifiers() == Qt::ControlModifier;
    core_->us_->push(new Select(pc, indices, core_->mw_->deselect_ || negative_select, core_->mw_->select_mask_));

}

int ${camel_name}::select3D(float x, float y){

    int idx = picker_->renderPick(x, y);

    if(idx == -1)
        return -1;

    Eigen::Vector3f p1, p2;
    int r = radius_ * 100;

    float rad;

    if(depth_adjust_){

        GLfloat wz = getZ(
                    cl_->active_->at(idx).getVector3fMap(),
                    glwidget_->camera_.modelviewMatrix() * cl_->active_->modelview(),
                    glwidget_->camera_.projectionMatrix(),
                    glwidget_->width(), glwidget_->height()
                    );

        screenToWorld(x-r, y-r, x+r, y+r, glwidget_->width(), glwidget_->height(), Eigen::Affine3f::Identity(), glwidget_->camera_.projectionMatrix(), p1, p2, wz);
        rad = (p2-p1).norm()/2.0f;
    } else {
        // near radius
        screenToWorld(x-r, y-r, x+r, y+r, glwidget_->width(), glwidget_->height(), Eigen::Affine3f::Identity(), glwidget_->camera_.projectionMatrix(), p1, p2, 0);
        rad = (p2-p1).norm()/2.0f;
        rad = rad * 10;
    }
    last_rad_ = rad;

    boost::shared_ptr<std::vector<int> > indices;
    indices.reset(new std::vector<int>());

    std::vector<float> distsq;
    cl_->active_->octree()->radiusSearch(idx, rad, *indices, distsq);

    bool negative_select = QApplication::keyboardModifiers() == Qt::ControlModifier;

    core_->us_->push(new Select(cl_->active_, indices, core_->mw_->deselect_ || negative_select, core_->mw_->select_mask_));

    return idx;
}

bool ${camel_name}::mouseClickEvent(QMouseEvent * event){
    return true;
}

bool ${camel_name}::mouseMoveEvent(QMouseEvent * event) {
    // How is the mouse moving?
    Eigen::Vector2d pos(event->x(), event->y());
    Eigen::Vector2d last_pos = last_mouse_pos_;

    last_mouse_pos_ << event->x(), event->y();
    if(event->buttons() != Qt::LeftButton && !event->modifiers())
        return false;
    if(event->buttons() != Qt::LeftButton && event->modifiers() == Qt::Key_Control)
        return true;
    if(cl_->clouds_.size() == 0)
        return false;

    if(is3d()){
        if(event->buttons()){
            if(last_picked_point_ != -1){

                Eigen::Vector3f p1 = cl_->active_->at(last_picked_point_).getArray3fMap();
                last_picked_point_ = select3D(event->x(), event->y());
                Eigen::Vector3f p2 = cl_->active_->at(last_picked_point_).getArray3fMap();

                Eigen::Vector3f diff = p2-p1;
                float len = diff.norm();

                if(len > last_rad_/4) {
                    float dist = 0;
                    Eigen::Vector3f dir = diff.normalized();
                    // Interpolate
                    while(dist < len){
                        Eigen::Vector3f p = p1 + dist*dir;
                        dist+=last_rad_/4;

                        pcl::PointXYZI q;
                        q.getVector3fMap() = p;

                        // calculate radius

                        Eigen::Affine3f mv = (glwidget_->camera_.modelviewMatrix() * cl_->active_->modelview());
                        Eigen::Affine3f pj = glwidget_->camera_.projectionMatrix();

                        int r = radius_ * 100;

                        float rad = last_rad_;

                        if(depth_adjust_){
                            double z = getZ(p, mv, pj, glwidget_->x(), glwidget_->y());
                            Eigen::Vector3f q1, q2;
                            screenToWorld(event->x()-r, event->y()-r, event->x()+r, event->y()+r, glwidget_->width(), glwidget_->height(), Eigen::Affine3f::Identity(), glwidget_->camera_.projectionMatrix(), q1, q2, z);
                            rad = (q2-q1).norm()/2.0f;
                        }

                        last_rad_ = rad;

                        boost::shared_ptr<std::vector<int> > indices = boost::make_shared<std::vector<int>>();

                        std::vector<float> distsq;
                        cl_->active_->octree()->radiusSearch(q, last_rad_, *indices, distsq);

                        bool negative_select = QApplication::keyboardModifiers() == Qt::ControlModifier;
                        core_->us_->push(new Select(cl_->active_, indices, core_->mw_->deselect_ || negative_select, core_->mw_->select_mask_));

                    }
                }

            } else {
                last_picked_point_ = select3D(event->x(), event->y());
            }
        }
    } else {

        if(event->buttons()){
            Eigen::Vector2d diff(pos - last_pos);
            float len = diff.norm();
            if(len > (radius_*500)/4) {
                float dist = 0;
                Eigen::Vector2d dir = diff.normalized();
                // Interpolate
                while(dist <= len){
                    Eigen::Vector2d p = last_pos + dist*dir;
                    dist+=(radius_*500)/4;
                    select2D(p.x(), p.y());
                }
            }
            select2D(event->x(), event->y());
        }
    }

    return true;
}

bool ${camel_name}::mousePressEvent(QMouseEvent * event) {
    if(event->buttons() != Qt::LeftButton)
        return false;
    if(cl_->clouds_.size() == 0)
        return false;
    core_->us_->beginMacro("${name}");
    if(is3d()){
        last_picked_point_ = select3D(event->x(), event->y());
    } else {
        select2D(event->x(), event->y());
    }
    last_mouse_pos_ << event->x(), event->y();
    mouse_down_pos_ = last_mouse_pos_;
    return true;
}

bool ${camel_name}::mouseReleaseEvent(QMouseEvent * event){
    core_->us_->endMacro();
    last_mouse_pos_ << event->x(), event->y();
    float dist = (last_mouse_pos_ - mouse_down_pos_).norm();
    last_picked_point_ = -1;
    if(dist < 2){
        return mouseClickEvent(event);
    }

    return true;
}

void ${camel_name}::enable() {
    if(is_enabled_){
        disable();
        return;
    }

    mw_->options_dock_->show();
    mw_->tooloptions_->setCurrentWidget(settings_);

    emit enabling();
    glwidget_->installEventFilter(this);
    flatview_->installEventFilter(this);
    connect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    is_enabled_ = true;
}

void ${camel_name}::disable() {
    enable_->setChecked(false);
    disconnect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    glwidget_->removeEventFilter(this);
    flatview_->removeEventFilter(this);
    is_enabled_ = false;
}

void ${camel_name}::setSelectMask(uint8_t mask){
    core_->mw_->setSelectMask(mask);
}

bool ${camel_name}::is3d(){
    QTabWidget * tabs = qobject_cast<QTabWidget *>(glwidget_->parent()->parent());
    return tabs->currentIndex() == tabs->indexOf(glwidget_);
}

bool ${camel_name}::eventFilter(QObject *object, QEvent *event){

    // Bypass plugin via shift
    if(QApplication::keyboardModifiers() == Qt::SHIFT || !core_->mw_->edit_mode_)
        return false;

    switch(event->type()){
    case QEvent::MouseButtonPress:
        return mousePressEvent(static_cast<QMouseEvent*>(event));
    case QEvent::MouseButtonRelease:
        return mouseReleaseEvent(static_cast<QMouseEvent*>(event));
    case QEvent::MouseMove:
        return mouseMoveEvent(static_cast<QMouseEvent*>(event));
    case QEvent::KeyPress:
        if(static_cast<QKeyEvent*>(event)->key() == Qt::Key_Control)
            return true;

    default:
        return false;
    }
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.${lower_name}")
