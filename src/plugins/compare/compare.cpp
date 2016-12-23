#include "plugins/compare/compare.h"
#include <QDebug>
#include <QAction>
#include <QToolBar>
#include <QWidget>
#include <QDockWidget>
#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QListWidget>
#include <QVBoxLayout>
#include <QStackedWidget>
#include <QLineEdit>
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "commands/select.h"
#include "pluginsystem/core.h"

QString Compare::getName(){
    return "Compare";
}

void Compare::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    ll_ = core_->ll_;
    glwidget_ = core_->mw_->glwidget_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;

    is_enabled_ = false;
    connect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));
    enable_ = new QAction(QIcon(":/compare.png"), "Enable Compare", 0);
    enable_->setCheckable(true);
    connect(enable_, SIGNAL(triggered()), this, SLOT(enable()));

    mw_->toolbar_->addAction(enable_);

    settings_ = new QWidget();
    QVBoxLayout * layout = new QVBoxLayout(settings_);
    settings_->setLayout(layout);

    mw_->tooloptions_->addWidget(settings_);

    layout->addWidget(new QLabel("Truth:", settings_));
    QListWidget * l1 = new QListWidget(settings_);
    layout->addWidget(l1);
    QHBoxLayout * split1 = new QHBoxLayout(settings_);
    layout->addLayout(split1);
    QPushButton * add1 = new QPushButton("Add selected layers", settings_);
    split1->addWidget(add1);
    QPushButton * clear1 = new QPushButton("Clear", settings_);
    split1->addWidget(clear1);

    layout->addWidget(new QLabel("Segementation:", settings_));
    QListWidget * l2 = new QListWidget(settings_);
    layout->addWidget(l2);
    QHBoxLayout * split2 = new QHBoxLayout(settings_);
    layout->addLayout(split2);
    QPushButton * add2 = new QPushButton("Add selected layers", settings_);
    split2->addWidget(add2);
    QPushButton * clear2 = new QPushButton("Clear", settings_);
    split2->addWidget(clear2);

    QPushButton * compare = new QPushButton("Compare", settings_);
    layout->addWidget(compare);

    precision_ = new QLineEdit(settings_);
    precision_->setReadOnly(true);
    recall_ = new QLineEdit(settings_);
    recall_->setReadOnly(true);

    QHBoxLayout * split = new QHBoxLayout(settings_);
    layout->addLayout(split);
    split->addWidget(new QLabel("Recall", settings_));
    split->addWidget(recall_);
    split->addWidget(new QLabel("Precision", settings_));
    split->addWidget(precision_);

    // connect

    connect(add1, &QPushButton::clicked, [=] (){
        for(boost::weak_ptr<Layer> s : ll_->getSelection() ){
            bool found = false;
            for(boost::weak_ptr<Layer> e : layers2_){
                if(s.lock() == e.lock()){
                    found = true;
                    break;
                }
            }
            for(boost::weak_ptr<Layer> e : layers1_){
                if(s.lock() == e.lock()){
                    found = true;
                    break;
                }
            }
            if(!found){
                layers1_.push_back(s);
                l1->addItem(s.lock()->getName());
            }
        }
    });

    connect(add2, &QPushButton::clicked, [=] (){
        for(boost::weak_ptr<Layer> s : ll_->getSelection() ){
            bool found = false;
            for(boost::weak_ptr<Layer> e : layers2_){
                if(s.lock() == e.lock()){
                    found = true;
                    break;
                }
            }
            for(boost::weak_ptr<Layer> e : layers1_){
                if(s.lock() == e.lock()){
                    found = true;
                    break;
                }
            }
            if(!found){
                layers2_.push_back(s);
                l2->addItem(s.lock()->getName());
            }
        }
    });

    connect(clear1, &QPushButton::clicked, [=] (){
        l1->clear();
        layers1_.clear();
    });

    connect(clear2, &QPushButton::clicked, [=] (){
        l2->clear();
        layers2_.clear();
    });

    connect(compare, SIGNAL(clicked()), this, SLOT(compare()));

    layout->addStretch();
}

void Compare::compare() {
    auto is_label_in_set = [=] (uint16_t label, std::vector<boost::weak_ptr<Layer> > & layers){
        const LayerSet & ls = ll_->getLayersForLabel(label);

        for(Layer * x : ls){
            for(boost::weak_ptr<Layer> y: layers){
                if(y.lock().get() == x)
                    return true;
            }
        }

        return false;
    };

    int count = 0;
    int count1 = 0;
    int count2 = 0;

    for(uint16_t label : cl_->active_->labels_){
        bool in1 = is_label_in_set(label, layers1_);
        bool in2 = is_label_in_set(label, layers2_);

        if(in1)
            count1++;
        if(in2)
            count2++;
        if(in1 && in2)
            count++;
    }

    // http://en.wikipedia.org/wiki/Precision_and_recall
    qDebug() << "overlap: " << count << "truth: " << count1 << "result: " << count2;
    qDebug() << "Recall: " << float(count)/count1;
    qDebug() << "Precision: " << float(count)/count2;

    recall_->setText(QString("%1").arg(float(count)/count1));
    precision_->setText(QString("%1").arg(float(count)/count2));
}

void Compare::cleanup(){
    mw_->toolbar_->removeAction(enable_);
    mw_->tooloptions_->removeWidget(settings_);
    delete enable_;
}

Compare::~Compare(){
    
}

void Compare::enable() {
    qDebug() << "enabled?" << is_enabled_;
    if(is_enabled_){
        disable();
        return;
    }

    qDebug() << "enabled!";

    mw_->options_dock_->show();
    mw_->tooloptions_->setCurrentWidget(settings_);
    emit enabling();
    glwidget_->installEventFilter(this);
    flatview_->installEventFilter(this);
    connect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    is_enabled_ = true;
}

void Compare::disable(){
    enable_->setChecked(false);
    disconnect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    glwidget_->removeEventFilter(this);
    flatview_->removeEventFilter(this);
    is_enabled_ = false;
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.compare")
