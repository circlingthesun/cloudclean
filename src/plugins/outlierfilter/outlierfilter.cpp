#include "plugins/outlierfilter/outlierfilter.h"
#include <QDebug>
#include <QAction>
#include <QToolBar>
#include <QSlider>
#include <QLabel>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QStackedWidget>
#include <QPushButton>
#include <QDockWidget>
#include <QSpinBox>
#include "utilities/utils.h"
#include <boost/make_shared.hpp>
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "commands/select.h"
#include "pluginsystem/core.h"


#include <pcl/search/kdtree.h>

QString OutlierFilter::getName(){
    return "OutlierFilter";
}

void OutlierFilter::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    ll_ = core_->ll_;
    glwidget_ = core_->mw_->glwidget_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;

    enable_ = new QAction(QIcon(":/outlierfilter.png"), "Outlier filter", 0);
    enable_->setCheckable(true);

    connect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));
    connect(enable_, SIGNAL(triggered()), this, SLOT(enable()));

    mw_->toolbar_->addAction(enable_);

    radius_ = 100;
    min_neigbours_ = 10;
    bool non_uniform_ = false;

    is_enabled_ = false;

    ////////////////////////

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
    layout->addWidget(l);
    layout->addWidget(slider);

    QLabel * l2 = new QLabel("Minimum neigbours:", settings_);
    layout->addWidget(l2);

    QSpinBox * spinbox = new QSpinBox(settings_);
    spinbox->setMinimum(1);
    spinbox->setMaximum(9999);
    spinbox->setAccelerated(true);
    spinbox->setValue(min_neigbours_);
    connect(spinbox, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), [this] (int val){
        min_neigbours_ = val;
    });
    layout->addWidget(spinbox);

    QLabel * l3 = new QLabel("Normalise:", settings_);
    layout->addWidget(l3);

    QCheckBox * cb = new QCheckBox(settings_);
    cb->setChecked(non_uniform_);
    layout->addWidget(cb);

    connect(cb, &QCheckBox::toggled, [=] (bool on){
        this->non_uniform_ = on;
    });


    QPushButton * start = new QPushButton("Filter", settings_);
    layout->addWidget(start);
    connect(start, SIGNAL(clicked()), this, SLOT(filter()));

    layout->addItem(new QSpacerItem(0, 0, QSizePolicy::Maximum, QSizePolicy::Maximum));
    layout->addStretch();

}

void OutlierFilter::cleanup(){
    mw_->toolbar_->removeAction(enable_);
    mw_->tooloptions_->removeWidget(settings_);
    delete enable_;
}

OutlierFilter::~OutlierFilter(){
    qDebug() << "OutlierFilter deleted";
}

void OutlierFilter::filter() {
    boost::shared_ptr<PointCloud> cloud = cl_->active_;

    if(cloud == nullptr)
        return;

    // downsample
    std::vector<int> big_to_small;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr smallcloud = octreeDownsample(cloud.get(), (radius_*2)/100, big_to_small);

    // create small to big map
    std::vector<std::vector<int>> small_to_big(smallcloud->size());
    for(size_t big_idx = 0; big_idx < big_to_small.size(); big_idx++) {
        int small_idx = big_to_small[big_idx];
        small_to_big[small_idx].push_back(big_idx);
    }


    std::vector<uint> candidates;
    boost::shared_ptr<std::vector<int> > indices = boost::make_shared<std::vector<int> >();

    for(uint idx = 0; idx < smallcloud->size(); idx++){
        if(small_to_big[idx].size() < min_neigbours_){
            for(uint idx2: small_to_big[idx]){
                candidates.push_back(idx2);
            }
        }
    }

//    pcl::KdTreeFLANN<pcl::PointXYZRGB> search;
//    search.setInputCloud(cloud);

    qDebug() << "radius:" << radius_;
    qDebug() << "min_neigbours_:" << min_neigbours_;
    qDebug() << "Candidates: " << candidates.size();


    for(uint idx : candidates){
        std::vector<int> foundindices;
        std::vector<float> distsq;
        //search.radiusSearch(idx, radius_/100, foundindices, distsq);

        float dist = cl_->active_->at(idx).getVector3fMap().norm();

        int min_neighbours = min_neigbours_;

        if(non_uniform_)
            min_neighbours = sqrt((min_neigbours_/(radius_/100))*dist);

        cloud->octree()->radiusSearch(idx, radius_/100, foundindices, distsq);
        if(foundindices.size() < min_neigbours_){
            indices->push_back(idx);
        }
    }

    qDebug() << "Points selected: " << indices->size();

    core_->us_->push(new Select(cl_->active_, indices, core_->mw_->deselect_, core_->mw_->select_mask_, true, ll_->getHiddenLabels()));

}

void OutlierFilter::enable() {
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

void OutlierFilter::disable(){
    enable_->setChecked(false);
    disconnect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    glwidget_->removeEventFilter(this);
    flatview_->removeEventFilter(this);
    is_enabled_ = false;
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.outlierfilter")
