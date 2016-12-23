#include "plugins/accuracy/accuracy.h"
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
#include <QLabel>
#include <QDoubleSpinBox>
#include <QApplication>
#include <QDockWidget>
#include <QFile>
#include <QStringList>
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "commands/select.h"
#include "pluginsystem/core.h"

QString Accuracy::getName(){
    return "Accuracy";
}

void Accuracy::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    ll_ = core_->ll_;
    glwidget_ = core_->mw_->glwidget_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;

    is_enabled_ = false;
    //connect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));
    enable_ = new QAction(QIcon(":/accuracy.png"), "Enable Accuracy", 0);
    enable_->setCheckable(true);
    connect(enable_, SIGNAL(triggered()), this, SLOT(enable()));

    mw_->toolbar_->addAction(enable_);


    QVBoxLayout * dock_layout = new QVBoxLayout();
    QHBoxLayout * dock_layout2 = new QHBoxLayout();

    dock_layout->addWidget(new QLabel("Target:"));
    QListWidget * l1 = new QListWidget();
    dock_layout->addWidget(l1);
    QHBoxLayout * split1 = new QHBoxLayout();
    dock_layout->addLayout(split1);
    QPushButton * add1 = new QPushButton("Add selected layers");
    split1->addWidget(add1);
    QPushButton * clear1 = new QPushButton("Clear");
    split1->addWidget(clear1);


    target_accuracy_ = 0.98;
    target_accuracy_input_ = new QDoubleSpinBox();
    target_accuracy_input_->setRange(0, 1);
    target_accuracy_input_->setValue(target_accuracy_);

    QHBoxLayout * split = new QHBoxLayout();
    dock_layout->addLayout(split);
    split->addWidget(new QLabel("Target accuracy"));
    split->addWidget(target_accuracy_input_);


    dock_layout->addWidget(new QLabel("File:"));
    filename_text_ = new QLineEdit();
    filename_text_->setPlaceholderText("CSV to append results to.");
    dock_layout->addWidget(filename_text_);

    connect(target_accuracy_input_, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=] (double value){
        target_accuracy_ = target_accuracy_input_->value();
    });


    connect(add1, &QPushButton::clicked, [=] (){
        for(boost::weak_ptr<Layer> s : ll_->getSelection() ){
            bool found = false;
            for(boost::weak_ptr<Layer> e : target_){
                if(s.lock() == e.lock()){
                    found = true;
                    break;
                }
            }
            if(!found){
                target_.push_back(s);
                l1->addItem(s.lock()->getName());
            }
        }
    });


    connect(clear1, &QPushButton::clicked, [=] (){
        l1->clear();
        target_.clear();
    });

    dock_layout->addStretch();

    dock_widget_ = new QWidget();
    dock_widget_->setLayout(dock_layout);
    dock_ = new QDockWidget();
    dock_->setWidget(dock_widget_);
    dock_->setWindowTitle(tr("Accuracy options"));

    mw_->addDockWidget(Qt::RightDockWidgetArea, dock_);
    mw_->removeDockWidget(dock_);

    //mw_->tooloptions_->addWidget(dock_widget_);


    ///////// Bottom dock

    QHBoxLayout * split2 = new QHBoxLayout();
    dock_layout2->addLayout(split2);

    split2->addWidget(new QLabel("Accuracy"));
    accuracy_text_ = new QLineEdit();
    split2->addWidget(accuracy_text_);

    split2->addWidget(new QLabel("Time elapsed"));
    elapsed_time_text_ = new QLineEdit();
    split2->addWidget(elapsed_time_text_);

    QHBoxLayout * split3 = new QHBoxLayout();
    dock_layout2->addLayout(split3);
    start_button_ = new QPushButton("Start");
    QPushButton * reset_button = new QPushButton("Reset");

    test_key_text_ = new QLineEdit();
    test_key_text_->setPlaceholderText("Test label (for inside CSV)");

    QPushButton * save_button = new QPushButton("Save result");

    split3->addWidget(start_button_);
    split3->addWidget(reset_button);
    split3->addWidget(test_key_text_);
    split3->addWidget(save_button);

    connect(start_button_, &QPushButton::pressed, this, &Accuracy::start_stop);
    connect(reset_button, &QPushButton::pressed, this, &Accuracy::reset);
    connect(save_button, &QPushButton::pressed, this, &Accuracy::save);

    dock_layout2->addStretch();

    dock_widget2_ = new QWidget();
    dock_widget2_->setLayout(dock_layout2);
    dock2_ = new QDockWidget();
    dock2_->setWidget(dock_widget2_);
    //dock_->setWindowTitle(tr("Accuracy options"));

    ///////////////////////////////
    timer_.setInterval(1000);
    connect(&timer_, &QTimer::timeout, this, &Accuracy::sample);
}

void Accuracy::start_stop(){
    if(!started_){
        time_accuracy_precision_recall_.clear();
        time_.restart();
        timer_.start();
        start_button_->setText("Stop");
        started_ = true;
    } else {
        timer_.stop();
        start_button_->setText("Start");
        started_ = false;
    }
}

void Accuracy::reset(){
    timer_.stop();
    time_accuracy_precision_recall_.clear();
    started_ = false;
}

void Accuracy::save(){
    timer_.stop();
    QString filename = filename_text_->text();
    QString test_key = test_key_text_->text();
    QFile data(filename);

    if(data.open(QFile::WriteOnly | QFile::Append)) {
        QTextStream output(&data);

        QStringList seconds_list;
        QStringList accuracy_list;
        QStringList precision_list;
        QStringList recall_list;

        seconds_list.push_back("Seconds");
        accuracy_list.push_back("Acurracy");
        precision_list.push_back("Precision");
        recall_list.push_back("Recall");

        for(std::tuple<int, float, float, float> & item :time_accuracy_precision_recall_){

            int seconds = std::get<0>(item);
            float accuracy = std::get<1>(item);
            float precision = std::get<2>(item);
            float recall = std::get<3>(item);

            seconds_list.push_back(QString::number(seconds));
            accuracy_list.push_back(QString::number(accuracy));
            precision_list.push_back(QString::number(precision));
            recall_list.push_back(QString::number(recall));
        }

        output << "\"" + test_key + "\"\n";
        output << "\"Total seconds\"; " << "\"" + QString::number(std::get<0>(time_accuracy_precision_recall_.back())) + "\"\n";
        output << "\"Final accuracy\"; " << "\"" + QString::number(std::get<1>(time_accuracy_precision_recall_.back())) + "\"\n";
        output << "\"" + seconds_list.join("\"; \"") + "\"\n";
        output << "\"" + accuracy_list.join("\"; \"") + "\"\n";
        output << "\"" + precision_list.join("\"; \"") + "\"\n";
        output << "\"" + recall_list.join("\"; \"") + "\"\n";

    }



}

void Accuracy::sample() {

    if(!cl_->active_){
        return;
    }

    if(!target_.size()){
        return;
    }

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

    int true_positive_count = 0;
    int targetted_count = 0;
    int selected_count = 0;

    for(uint i = 0; i < cl_->active_->points.size(); i++){
        uint16_t label = cl_->active_->labels_[i];
        bool selected = int8_t(cl_->active_->flags_[i]) & 1; // Only count red
        bool targetted = is_label_in_set(label, target_);

        if(targetted){
            targetted_count++;
        }
        if(selected){
            selected_count++;
        }
        if(targetted && selected) {
            true_positive_count++;
        }
    }

    // http://en.wikipedia.org/wiki/Precision_and_recall
    qDebug() << "overlap: " << true_positive_count << "truth: " << targetted_count << "result: " << selected_count;
    qDebug() << "Recall: " << float(true_positive_count)/targetted_count;
    qDebug() << "Precision: " << float(true_positive_count)/selected_count;

    float recall = float(true_positive_count)/targetted_count;
    float precision = float(true_positive_count)/selected_count;
    float fscore = 2 * (precision * recall) / (precision + recall);

    fscore = std::isnan(fscore) ? 0 : fscore;
    precision = std::isnan(precision) ? 0 : precision;
    recall = std::isnan(recall) ? 0 : recall;

    accuracy_text_->setText(QString::number(fscore));

    int seconds = time_.elapsed()/1000;

    time_accuracy_precision_recall_.push_back(std::make_tuple(seconds, fscore, recall, precision));

    elapsed_time_text_->setText(QString::number(seconds));

    if(fscore > target_accuracy_){
        elapsed_time_text_->setStyleSheet("QLineEdit { background: #C2DC5F;}");
        accuracy_text_->setStyleSheet("QLineEdit { background: #C2DC5F;}");

        if(started_){
            start_stop();
        }

    } else {
        elapsed_time_text_->setStyleSheet("QLineEdit { background: #FFFFFF;}");
        accuracy_text_->setStyleSheet("QLineEdit { background: #FFFFFF;}");
    }


}

void Accuracy::cleanup(){
    mw_->toolbar_->removeAction(enable_);
    mw_->tooloptions_->removeWidget(dock_widget_);
    delete enable_;
}

Accuracy::~Accuracy(){
    
}

void Accuracy::enable() {

    if(is_enabled_){
        disable();
        return;
    }


    mw_->addDockWidget(Qt::RightDockWidgetArea, dock_);
    mw_->tabifyDockWidget(mw_->options_dock_, dock_);
    dock_->show();
    dock_->raise();

    mw_->addDockWidget(Qt::BottomDockWidgetArea, dock2_);
    dock2_->show();
    dock2_->raise();

    emit enabling();
    glwidget_->installEventFilter(this);
    flatview_->installEventFilter(this);
    is_enabled_ = true;
}

void Accuracy::disable(){
    mw_->removeDockWidget(dock_);
    mw_->removeDockWidget(dock2_);
    enable_->setChecked(false);
    glwidget_->removeEventFilter(this);
    flatview_->removeEventFilter(this);
    is_enabled_ = false;
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.accuracy")
