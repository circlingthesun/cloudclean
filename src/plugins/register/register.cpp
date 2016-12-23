#include "plugins/register/register.h"
#include <QDebug>
#include <QAction>
#include <QToolBar>
#include <QWidget>
#include <QDockWidget>
#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QVBoxLayout>
#include <QStackedWidget>
#include <QComboBox>
#include <QFileInfo>
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "commands/select.h"
#include "pluginsystem/core.h"
#include "utilities/utils.h"
#include <pcl/registration/icp.h>

QString Register::getName(){
    return "Register";
}

void Register::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    ll_ = core_->ll_;
    glwidget_ = core_->mw_->glwidget_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;

    stationary_idx_ = -1;
    moving_idx_ = -1;

    is_enabled_ = false;
    connect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));
    enable_ = new QAction(QIcon(":/register.png"), "Enable Register", 0);
    enable_->setCheckable(true);
    connect(enable_, SIGNAL(triggered()), this, SLOT(enable()));

    mw_->toolbar_->addAction(enable_);

    settings_ = new QWidget();
    QVBoxLayout * layout = new QVBoxLayout(settings_);
    settings_->setLayout(layout);

    mw_->tooloptions_->addWidget(settings_);

//    QLabel * l = new QLabel(QString("Radius: %1 cm").arg(radius_), settings_);

//    QSlider * slider = new QSlider(settings_);
//    slider->setOrientation(Qt::Horizontal);
//    slider->setRange(1, 300);
//    slider->setSingleStep(1);
//    slider->setToolTip("Radius in cm");
//    slider->setValue(radius_);
//    slider->setTickPosition(QSlider::TicksBelow);
//    connect(slider, &QSlider::valueChanged, [this, l] (int val){
//        radius_ = val;
//        l->setText(QString("Radius: %1 cm").arg(radius_));
//    });
//    layout->addWidget(slider);

    layout->addWidget(new QLabel("Stationary cloud"));
    stationary_cb_ = new QComboBox(settings_);
    layout->addWidget(stationary_cb_);

    layout->addWidget(new QLabel("Moving cloud"));
    moving_cb_ = new QComboBox(settings_);
    layout->addWidget(moving_cb_);

    connect(cl_, SIGNAL(listModified()), this, SLOT(clModified()));
    connect(cl_, SIGNAL(listModified()), this, SLOT(clModified()));

    connect(stationary_cb_, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged), [this] (int idx){
        stationary_idx_ = stationary_cb_->itemData(idx).toInt();
        if(stationary_idx_ != -1 && stationary_idx_ == moving_idx_){
            stationary_idx_ = -1;
            stationary_cb_->setCurrentIndex(0);
        }
    });

    connect(moving_cb_, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged), [this] (int idx){
        moving_idx_ = moving_cb_->itemData(idx).toInt();
        if(moving_idx_ != -1 && stationary_idx_ == moving_idx_){
            moving_idx_ = -1;
            moving_cb_->setCurrentIndex(0);
        }
    });

    clModified();

    QPushButton * run = new QPushButton("Run");
    connect(run, SIGNAL(clicked()), this, SLOT(align()));
    layout->addWidget(run);

    layout->addStretch();
}

void Register::clModified(){
    stationary_cb_->clear();
    moving_cb_->clear();

    stationary_cb_->addItem("Select", -1);
    moving_cb_->addItem("Select", -1);

    stationary_idx_ = -1;
    moving_idx_ = -1;

    for(int idx = 0; idx < cl_->clouds_.size(); idx++) {
        boost::shared_ptr<PointCloud> cloud = cl_->clouds_[idx];
        QFileInfo fi(cloud->filepath());
        stationary_cb_->addItem(fi.fileName(), idx);
        moving_cb_->addItem(fi.fileName(), idx);
    }
}

void Register::align() {
    if(stationary_idx_ == -1 || moving_idx_== -1)
        return;

    boost::shared_ptr<PointCloud> stationary = cl_->clouds_[stationary_idx_];
    boost::shared_ptr<PointCloud> moving = cl_->clouds_[moving_idx_];

    std::vector<int> big_to_small;;
    pcl::PointCloud<pcl::PointXYZI>::Ptr s = octreeDownsample(stationary.get(), 0.01, big_to_small);
    pcl::PointCloud<pcl::PointXYZI>::Ptr m = octreeDownsample(moving.get(), 0.01, big_to_small);

    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setTransformationEpsilon(1e-6);
    icp.setMaxCorrespondenceDistance (0.1);
    icp.setInputCloud(m);
    icp.setInputTarget(s);
    icp.setMaximumIterations (2);
    icp.align(*m);

    moving->sensor_origin_ = m->sensor_origin_;
    moving->sensor_orientation_ = m->sensor_orientation_;

    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
      icp.getFitnessScore() << std::endl;
}

void Register::cleanup(){
    disconnect(cl_, SIGNAL(rowsInserted(QModelIndex,int,int,QPrivateSignal)), this, SLOT(clModified()));
    disconnect(cl_, SIGNAL(rowsRemoved(QModelIndex,int,int,QPrivateSignal)), this, SLOT(clModified()));
    mw_->toolbar_->removeAction(enable_);
    mw_->tooloptions_->removeWidget(settings_);
    delete enable_;
}

Register::~Register(){
    
}

void Register::enable() {
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

void Register::disable(){
    enable_->setChecked(false);
    disconnect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    glwidget_->removeEventFilter(this);
    flatview_->removeEventFilter(this);
    is_enabled_ = false;
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.register")
