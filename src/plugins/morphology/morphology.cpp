#include "plugins/morphology/morphology.h"
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

QString Morphology::getName(){
    return "Morphology";
}

void Morphology::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    ll_ = core_->ll_;
    glwidget_ = core_->mw_->glwidget_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;

    is_enabled_ = false;
    connect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));
    enable_ = new QAction(QIcon(":/morphology.png"), "Enable Morphology", 0);
    enable_->setCheckable(true);
    connect(enable_, SIGNAL(triggered()), this, SLOT(enable()));

    radius_ = 10;

    mw_->toolbar_->addAction(enable_);

    settings_ = new QWidget();
    QVBoxLayout * layout = new QVBoxLayout(settings_);
    settings_->setLayout(layout);

    mw_->tooloptions_->addWidget(settings_);

    QLabel * l = new QLabel(QString("Radius: %1 cm").arg(radius_), settings_);

    QSlider * slider = new QSlider(settings_);
    slider->setOrientation(Qt::Horizontal);
    slider->setRange(1, 300);
    slider->setSingleStep(1);
    slider->setToolTip("Radius in cm");
    slider->setValue(radius_);
    slider->setTickPosition(QSlider::TicksBelow);
    connect(slider, &QSlider::valueChanged, [this, l] (int val){
        radius_ = val;
        l->setText(QString("Radius: %1 cm").arg(radius_));
    });

    QPushButton * open = new QPushButton("Open");
    connect(open, SIGNAL(clicked()), this, SLOT(open()));

    layout->addWidget(l);
    layout->addWidget(slider);
    layout->addWidget(open);
    layout->addStretch();
}

void Morphology::open() {
    core_->us_->beginMacro("Open");

    auto & points = cl_->active_->points;
    const Octree::Ptr octree = cl_->active_->octree();
    std::vector<uint16_t> & labels = cl_->active_->labels_;

    std::vector<float> dists;
    std::vector<int> idxs;

    // for all selections
    for(uint8_t s = 0; s < 8; s++){

        // erode
        boost::shared_ptr<std::vector<int>> indices = boost::make_shared<std::vector<int>>();

        for(int idx = 0; idx < cl_->active_->points.size(); idx++){
            bool selected = labels[idx] & 1 < s;

            if(!selected)
                continue;

            idxs.clear(); dists.clear();
            octree->radiusSearch(idx, radius_, idxs, dists, 20); // TODO: set resonable max nn

            // all neighbours need to be deselected
            bool on = true;
            for(int idx2 : idxs){
                if(idx2 == idx)
                    continue;
                bool selected = labels[idx2] & 1 < s;
                if(!selected){
                    on = false;
                    break;
                }
            }

            if(!on)
                indices->push_back(idx);

        }

        core_->us_->push(new Select(cl_->active_, indices, true, s, true, ll_->getHiddenLabels()));


        // dilate
        indices = boost::make_shared<std::vector<int>>();
        for(int idx = 0; idx < cl_->active_->points.size(); idx++){
            bool selected = labels[idx] & 1 < s;

            if(selected)
                continue;

            idxs.clear(); dists.clear();
            octree->radiusSearch(idx, radius_, idxs, dists, 20); // TODO: set resonable max nn

            // all neighbours need to be selected
            bool on = true;
            for(int idx2 : idxs){
                if(idx2 == idx)
                    continue;
                bool selected = labels[idx2] & 1 < s;
                if(!selected){
                    on = false;
                    break;
                }
            }

            if(on)
                indices->push_back(idx);

        }

        core_->us_->push(new Select(cl_->active_, indices, false, s, true, ll_->getHiddenLabels()));

    }


    core_->us_->endMacro();
}

void Morphology::cleanup(){
    mw_->toolbar_->removeAction(enable_);
    mw_->tooloptions_->removeWidget(settings_);
    delete enable_;
}

Morphology::~Morphology(){
    
}

void Morphology::enable() {
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

void Morphology::disable(){
    enable_->setChecked(false);
    disconnect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    glwidget_->removeEventFilter(this);
    flatview_->removeEventFilter(this);
    is_enabled_ = false;
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.morphology")
