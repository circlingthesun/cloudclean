#include "plugins/autotest/autotest.h"
#include <utility>
#include <QDebug>
#include <QCoreApplication>
#include <QAction>
#include <QToolBar>
#include <QWidget>
#include <QDockWidget>
#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QVBoxLayout>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QStackedWidget>
#include <QSettings>
#include <QDir>
#include <QFileDialog>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonValue>
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "commands/select.h"
#include "pluginsystem/core.h"
#include "pluginsystem/pluginmanager.h"

#include "plugins/project/project.h"
#include "plugins/markov/markov.h"
#include "plugins/accuracy/accuracy.h"

QString AutoTest::getName(){
    return "AutoTest";
}

void AutoTest::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    ll_ = core_->ll_;
    glwidget_ = core_->mw_->glwidget_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;

    is_enabled_ = false;

    dock_widget_ = new QWidget();
    dock_ = new QDockWidget();



    enable_ = new QAction(QIcon(":/autotest.png"), "Enable AutoTest", 0);
    enable_->setCheckable(true);

    connect(enable_, SIGNAL(triggered()), this, SLOT(enable()));
}

void AutoTest::initialize2(PluginManager *pm){
//    project_ = pm->findPlugin<Project>();
    markov_ = pm->findPlugin<Markov>();
    accuracy_ = pm->findPlugin<Accuracy>();

    // Widgets


    QPushButton * open_button = new QPushButton("Open test");
    QPushButton * run_button = new QPushButton("Run test");

    run_button->setDisabled(true);

    connect(open_button, &QPushButton::clicked, [=] (){
        QSettings settings;

        QString path = settings.value("load/lasttestlocation", QDir::home().absolutePath()).toString();
        QString filename = QFileDialog::getOpenFileName(
                     nullptr, tr("Open test"), path , tr("JSON test files (*.json)"));
        if (filename.length() == 0)
            return;

        settings.setValue("load/lasttestlocation", QFileInfo(filename).absolutePath());
        settings.sync();
        run_button->setDisabled(false);
        test_path_ = filename;

    });

    connect(run_button, &QPushButton::clicked, (std::function<void()>) std::bind(&AutoTest::runtests, this));

//    connect(run_button, &QPushButton::clicked, [&](){

//    });


    // Toolbar
    mw_->toolbar_->addAction(enable_);


    // Dock
    QVBoxLayout * dock_layout = new QVBoxLayout();

    dock_layout->addWidget(new QLabel("GAR!"));
    dock_layout->addWidget(open_button);
    dock_layout->addWidget(run_button);
    dock_layout->setStretch(100, 100);


    dock_widget_->setLayout(dock_layout);
    dock_->setWindowTitle(tr("Autotest options"));
    dock_->setWidget(dock_widget_);
}

std::tuple<float, float, float> AutoTest::runTest(
        std::vector<std::string> features,
        float downsample,
        float curvature_radius,
        float pca_radius,
        float density_radius,
        int pca_max_nn,
        int tree_count,
        int tree_depth){

    markov_->pca_radius_spinner_->setValue(pca_radius);
    markov_->curvature_radius_spinner_->setValue(curvature_radius);
    markov_->octree_cell_size_spinner_->setValue(downsample);
    markov_->density_radius_spinner_->setValue(density_radius);

    markov_->tree_count_spinner_->setValue(tree_count);
    markov_->tree_depth_spinner_->setValue(tree_depth);
    markov_->max_nn_spinner_->setValue(pca_max_nn);;

    markov_->feature_list_->selectOnly(features);

    QCoreApplication::processEvents(); // Need this for settings to take effect?

    markov_->randomforest();

    float accuracy, precision, recall;
    std::tie(accuracy, precision, recall) = accuracy_->sample();

    core_->us_->undo();

    QCoreApplication::processEvents();

    qDebug() << "Results" << accuracy << precision <<  recall;
    return std::make_tuple(accuracy, precision, recall);
}

void AutoTest::runtests() {

    qDebug() << "Opening file: " << test_path_;
    QFile file(test_path_);
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    if(!file.isOpen()){
        qDebug() << "Could not open file";
        return;
    }

    QJsonParseError err;
    QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &err);
    file.close();

    if(doc.isNull()){
        qDebug() << "No test data: " << err.errorString();
    }

    QJsonObject root = doc.object();

    QJsonObject defaults = root["defaults"].toObject();
    QJsonArray tests = root["tests"].toArray();

    // Setup reporting
    QFile report_file("report.csv");
    report_file.open(QIODevice::Append | QIODevice::Text);
    QDebug report(&report_file);
    report.setAutoInsertSpaces(false);

    report << "File:" << test_path_ << "\n";

//    report << "Feature,downsample,curvature radius,pca radius,density_radius,tree_count,tree_depth,accuracy,precision,recall" << "\n";

    report << "Feature,accuracy,precision,recall" << "\n";

    for(QJsonValueRef test : tests){
        QJsonObject t = test.toObject();
        QJsonArray featuresArray = t["features"].toArray();

        qDebug() << "Test:" << t["name"].toString();

        std::vector<std::string> features;
        for(QJsonValueRef nameRef : featuresArray){
            features.push_back(nameRef.toString().toStdString());
        }

        QString name = t["name"].toString();
        float downsample = t.contains("downsample") ? t["downsample"].toDouble() : defaults["downsample"].toDouble();
        float curvature_radius = t.contains("curvature_radius") ? t["curvature_radius"].toDouble() : defaults["curvature_radius"].toDouble();;
        float pca_radius = t.contains("pca_radius") ? t["pca_radius"].toDouble() : defaults["pca_radius"].toDouble();;
        float density_radius = t.contains("downsample") ? t["density_radius"].toDouble() : defaults["density_radius"].toDouble();;
        int pca_max_nn = t.contains("pca_max_nn") ? t["pca_max_nn"].toInt() : defaults["pca_max_nn"].toInt();;
        int tree_count = t.contains("tree_count") ? t["tree_count"].toInt() : defaults["tree_count"].toInt();;
        int tree_depth = t.contains("tree_depth") ? t["tree_depth"].toInt() : defaults["tree_depth"].toInt();;

        float accuracy, precision, recall;
        std::tie(accuracy, precision, recall) = runTest(
                features,
                downsample,
                curvature_radius,
                pca_radius,
                density_radius,
                pca_max_nn,
                tree_count,
                tree_depth);

        report << name << ","
//                << downsample << ","
//                << curvature_radius << ","
//                << pca_radius << ","
//                << density_radius << ","
//                << tree_count << ","
//                << tree_depth << ","
                << accuracy << ","
                << precision << ","
                << recall << "\n";
    }

    report_file.close();
}

void AutoTest::cleanup(){
    disable();
    mw_->toolbar_->removeAction(enable_);
    delete enable_;
}

AutoTest::~AutoTest(){
    
}

void AutoTest::enable() {
    if(is_enabled_){
        disable();
        return;
    }


    emit enabling();
    connect(core_, SIGNAL(endEdit()), this, SLOT(disable()));

    mw_->addDockWidget(Qt::RightDockWidgetArea, dock_);
    mw_->tabifyDockWidget(mw_->options_dock_, dock_);
    dock_->show();
    dock_->raise();

    is_enabled_ = true;
}

void AutoTest::disable(){
    enable_->setChecked(false);
    disconnect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    mw_->removeDockWidget(dock_);
    is_enabled_ = false;
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.autotest")
