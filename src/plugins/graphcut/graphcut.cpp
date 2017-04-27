#include "plugins/graphcut/graphcut.h"
#include <QDebug>
#include <QEvent>
#include <QKeyEvent>
#include <QAction>
#include <QGLShaderProgram>
#include <QGLBuffer>
#include <QTabWidget>
#include <QApplication>
#include <QToolBar>
#include <QVBoxLayout>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QSpacerItem>
#include <QStackedWidget>
#include <QSlider>
#include <QDockWidget>
#include <QMessageBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QProgressDialog>
#include <boost/serialization/shared_ptr.hpp>
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "utilities/pointpicker.h"
#include "commands/select.h"
#include "pluginsystem/core.h"
#include "plugins/graphcut/mincut.h"

QString GraphCut::getName(){
    return "graph_cut";
}

GraphCut::~GraphCut(){
    qDebug() << "Destroyed graph cut";
}

void GraphCut::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    ll_ = core_->ll_;
    glwidget_ = core_->mw_->glwidget_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;

    enable_ = new QAction(QIcon(":/images/graph_cut.png"), "Graph cut tool", 0);
    enable_->setCheckable(true);
    enable_->setChecked(false);

    is_enabled_ = false;
    radius_ = 0.5f;

    connect(enable_, SIGNAL(triggered()), this, SLOT(enable()));
    connect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));

    mw_->addMenu(enable_, "Edit");
    mw_->toolbar_->addAction(enable_);

    settings_ = new QWidget();
    QGridLayout * layout = new QGridLayout(settings_);
    settings_->setLayout(layout);
    mw_->tooloptions_->addWidget(settings_);

    QSpinBox * kconnect = new QSpinBox(settings_);
    QDoubleSpinBox * sigma = new QDoubleSpinBox(settings_);
    QDoubleSpinBox * source_weight = new QDoubleSpinBox(settings_);
    QDoubleSpinBox * radius = new QDoubleSpinBox(settings_);

    k_connect_ = 14;
    sigma_ = 0.25;
    source_weight_ = 0.80;
    radius_ = 3;

    kconnect->setValue(k_connect_);
    sigma->setValue(sigma_);
    source_weight->setValue(source_weight_);
    radius->setValue(radius_);

    QLabel * label_kcon = new QLabel("K connectivity", settings_);
    QLabel * label_sigma = new QLabel("Sigma (Density)", settings_);
    QLabel * label_source_w = new QLabel("Source weight", settings_);
    QLabel * label_radius = new QLabel("Radius (meters)", settings_);

    layout->addWidget(label_kcon);
    layout->addWidget(kconnect);

    layout->addWidget(label_sigma);
    layout->addWidget(sigma);

    layout->addWidget(label_source_w);
    layout->addWidget(source_weight);

    layout->addWidget(label_radius);
    layout->addWidget(radius);

    connect(kconnect, (void (QSpinBox:: *)(int)) &QSpinBox::valueChanged, [this] (int val) {
        k_connect_ = val;
    });

    connect(sigma, (void (QDoubleSpinBox:: *)(double)) &QDoubleSpinBox::valueChanged, [this] (double val) {
        sigma_ = val;
    });

    connect(source_weight, (void (QDoubleSpinBox:: *)(double)) &QDoubleSpinBox::valueChanged, [this] (double val) {
        radius_ = val;
    });

    connect(radius, (void (QDoubleSpinBox:: *)(double)) &QDoubleSpinBox::valueChanged, [this] (double val) {
        radius_ = val;
    });

}

void GraphCut::cleanup(){
    disable();
    disconnect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));
    disconnect(enable_, SIGNAL(triggered()), this, SLOT(enable()));
    disconnect();
    mw_->toolbar_->removeAction(enable_);
    mw_->removeMenu(enable_, "Edit");
    mw_->tooloptions_->removeWidget(settings_);
    delete settings_;
    delete enable_;
}

void GraphCut::completeSegment(Select * select) {
    if(select != nullptr) {
        core_->us_->beginMacro("Min cut");
        core_->us_->push(select);
        core_->us_->endMacro();
        core_->cl_->updated();
        core_->mw_->stopBgAction("Graph cut completed.");
    } else {
        core_->mw_->stopBgAction("Too few points selected for graph cut.");
    }

    core_->mw_->setEnabled(true);
}

void GraphCut::segment(int idx){


    MinCut mc;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr(cl_->active_.get(), boost::serialization::null_deleter());
    mc.setInputCloud(ptr);

    pcl::IndicesPtr source_indices(new std::vector<int>);

    for(uint idx2 = 0; idx2 < cl_->active_->flags_.size(); idx2++){
        PointFlags & flag = cl_->active_->flags_[idx2];
        if((uint8_t)flag && (uint8_t)PointFlags::selected)
            source_indices->push_back(idx2);
    }

    if(source_indices->size() < 10) {
        qDebug() << "Less than 10 source points. Aborting";
        qRegisterMetaType<Select *>("Select *");
        QMetaObject::invokeMethod(this, "completeSegment", Qt::QueuedConnection,
                                  Q_ARG( Select *, nullptr));
        return;
    }

    mc.setIndices(source_indices);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZRGB> ());
    foreground_points->points.push_back(cl_->active_->points[idx]); // What? There can be more than one?

    mc.setForegroundPoints (foreground_points);
    mc.setRadius (radius_);
    mc.setSigma (sigma_); // This is density?
    mc.setNumberOfNeighbours (k_connect_);
    mc.setSourceWeight (source_weight_);

    std::vector <pcl::PointIndices> clusters;
    mc.extract (clusters);

    auto select = boost::make_shared<std::vector<int>>(clusters[1].indices.size());
    auto deselect = boost::make_shared<std::vector<int>>(clusters[0].indices.size());
    std::copy(clusters[0].indices.begin(), clusters[0].indices.end(), deselect->begin());
    std::copy(clusters[1].indices.begin(), clusters[1].indices.end(), select->begin());

    Select * selectcmd = new Select(cl_->active_, select);
    Select * deselectcmd = new Select(cl_->active_, select, true);

    qRegisterMetaType<Select *>("Select *");
    QMetaObject::invokeMethod(this, "completeSegment", Qt::QueuedConnection,
                              Q_ARG( Select *, selectcmd));
    QMetaObject::invokeMethod(this, "completeSegment", Qt::QueuedConnection,
                              Q_ARG( Select *, deselectcmd));

}

bool GraphCut::mouseClickEvent(QMouseEvent * event){

    return true;
}

bool GraphCut::mousePressEvent(QMouseEvent * event) {
    if(event->buttons() != Qt::LeftButton)
        return false;
    if(cl_->clouds_.size() == 0)
        return false;

    last_mouse_pos_ << event->x(), event->y();
    mouse_down_pos_ = last_mouse_pos_;
    return true;
}

bool GraphCut::mouseReleaseEvent(QMouseEvent * event){
    last_mouse_pos_ << event->x(), event->y();
    float dist = (last_mouse_pos_ - mouse_down_pos_).norm();
    if(dist < 2){
        //return mouseClickEvent(event);
    }

    int idx = pick(event->x(), event->y(), glwidget_->width(),
                   glwidget_->height(), 1e-04,
                   glwidget_->camera_.projectionMatrix(),
                   glwidget_->camera_.modelviewMatrix(),
                   cl_->active_);

    if(idx == -1)
        return true;


    core_->mw_->setEnabled(false);
    core_->mw_->startBgAction("Graph cut in progress...");

    std::thread(&GraphCut::segment, this, idx).detach();

    return true;
}

void GraphCut::enable() {
    if(is_enabled_){
        disable();
        return;
    }
    QTabWidget * tabs = qobject_cast<QTabWidget *>(glwidget_->parent()->parent());
    tabs->setCurrentWidget(glwidget_);
    enable_->setChecked(true);

    mw_->options_dock_->show();
    mw_->tooloptions_->setCurrentWidget(settings_);

    emit enabling();

    glwidget_->installEventFilter(this);
    connect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    is_enabled_ = true;

    // Let the user know what to do
    QMessageBox::information(nullptr, tr("Make a selection"),
                    tr("Select the center of an object..."),
                    QMessageBox::Ok, QMessageBox::Ok);

}

void GraphCut::disable() {
    enable_->setChecked(false);
    disconnect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    glwidget_->removeEventFilter(this);
    is_enabled_ = false;
}

bool GraphCut::eventFilter(QObject *object, QEvent *event){

    // Bypass plugin via shift
    if(QApplication::keyboardModifiers() == Qt::SHIFT)
        return false;

    switch(event->type()){
    case QEvent::MouseButtonPress:
        return mousePressEvent(static_cast<QMouseEvent*>(event));
    case QEvent::MouseButtonRelease:
        return mouseReleaseEvent(static_cast<QMouseEvent*>(event));
    case QEvent::KeyPress:
        if(static_cast<QKeyEvent*>(event)->key() == Qt::Key_Control)
            return true;
    default:
        return false;
    }
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin")
