#include "plugins/radiusfilter/radiusfilter.h"
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
#include <boost/make_shared.hpp>
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "commands/select.h"
#include "pluginsystem/core.h"


QString RadiusFilter::getName(){
    return "RadiusFilter";
}

void RadiusFilter::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    ll_ = core_->ll_;
    glwidget_ = core_->mw_->glwidget_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;

    is_enabled_ = false;
    connect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));
    enable_ = new QAction(QIcon(":/radiusfilter.png"), "Enable RadiusFilter", 0);
    enable_->setCheckable(true);
    connect(enable_, SIGNAL(triggered()), this, SLOT(enable()));

    mw_->toolbar_->addAction(enable_);

    settings_ = new QWidget();
    QVBoxLayout * layout = new QVBoxLayout(settings_);
    settings_->setLayout(layout);

    mw_->tooloptions_->addWidget(settings_);

    radius_ = 100;

    QLabel * l = new QLabel(QString("Radius: %1 meters").arg(radius_), settings_);

    QSlider * slider = new QSlider(settings_);
    slider->setOrientation(Qt::Horizontal);
    slider->setRange(1, 300);
    slider->setSingleStep(1);
    slider->setToolTip("Radius in meters");
    slider->setValue(radius_);
    slider->setTickPosition(QSlider::TicksBelow);
    connect(slider, &QSlider::valueChanged, [this, l] (int val){
        radius_ = val;
        l->setText(QString("Radius: %1 meters").arg(radius_));
    });

    QPushButton * run = new QPushButton("Run");
    connect(run, SIGNAL(clicked()), this, SLOT(filter()));

    layout->addWidget(l);
    layout->addWidget(slider);
    layout->addWidget(run);
    layout->addStretch();
}

void RadiusFilter::filter() {
    boost::shared_ptr<std::vector<int>> selected_indices = boost::make_shared<std::vector<int>>();
    auto & points = cl_->active_->points;
    for(int idx = 0; idx < cl_->active_->points.size(); idx++){
        float dist = sqrt(points[idx].x * points[idx].x + points[idx].y * points[idx].y + points[idx].z * points[idx].z);
        if(dist > radius_) {
            selected_indices->push_back(idx);
        }
    }

    core_->us_->beginMacro("Radius filter");
    core_->us_->push(new Select(cl_->active_, selected_indices, core_->mw_->deselect_, core_->mw_->select_mask_, true, ll_->getHiddenLabels()));
    core_->us_->endMacro();

}

void RadiusFilter::cleanup(){
    mw_->toolbar_->removeAction(enable_);
    mw_->tooloptions_->removeWidget(settings_);
    delete enable_;
}

RadiusFilter::~RadiusFilter(){
    
}

void RadiusFilter::enable() {
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

void RadiusFilter::disable(){
    enable_->setChecked(false);
    disconnect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    glwidget_->removeEventFilter(this);
    flatview_->removeEventFilter(this);
    is_enabled_ = false;
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.radiusfilter")
