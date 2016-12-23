#include "plugins/autotest/autotest.h"
#include <utility>
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
#include "plugins/featureeval/featureeval.h"

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
    connect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));
    enable_ = new QAction(QIcon(":/autotest.png"), "Enable AutoTest", 0);
    enable_->setCheckable(true);
    connect(enable_, SIGNAL(triggered()), this, SLOT(enable()));

    mw_->toolbar_->addAction(enable_);

    settings_ = new QWidget();
    QVBoxLayout * layout = new QVBoxLayout(settings_);
    settings_->setLayout(layout);

    mw_->tooloptions_->addWidget(settings_);

    QPushButton * open = new QPushButton("Open test");

    QPushButton * run = new QPushButton("Run test");
    run->setDisabled(true);

    connect(open, &QPushButton::clicked, [=] (){
        QSettings settings;

        QString path = settings.value("load/lasttestlocation", QDir::home().absolutePath()).toString();
        QString filename = QFileDialog::getOpenFileName(
                     nullptr, tr("Open test"), path , tr("JSON test files (*.json)"));
        if (filename.length() == 0)
            return;

        settings.setValue("load/lasttestlocation", QFileInfo(filename).absolutePath());
        settings.sync();
        run->setDisabled(false);
        test_path_ = filename;

    });

    layout->addWidget(open);
    layout->addWidget(run);

    connect(run, &QPushButton::clicked, (std::function<void()>) std::bind(&AutoTest::runtest, this));

    layout->addStretch();
}

void AutoTest::initialize2(PluginManager *pm){
    project_ = pm->findPlugin<Project>();
    feature_eval_ = pm->findPlugin<FeatureEval>();
}

void AutoTest::setPermuteAndRun(std::vector<std::pair<QString, QJsonArray>> & params, QString fname, int idx){
    // end recursion and run
    if(idx == params.size()){
        feature_eval_->getFunction(fname)();
        return;
    }

    std::pair<QString, QJsonArray> values = params[idx];
    QString name = values.first;
    QJsonArray array = values.second;


    // STL PERMUTE
    for(QJsonValueRef valref : array){
        if(feature_eval_->param_map_.find(name) == feature_eval_->param_map_.end()){
            qDebug() << "cannot find: " << name;
        } else {
            qDebug() << "found: " << name;
        }
        qDebug() << "---------------------!!!!!!!!!!!!!!!!!!!!!!";

        if(valref.isObject() && name == "subsample_res"){
            *(feature_eval_->param_map_[name + ".small"].f) = valref.toObject()["small"].toDouble();
            *(feature_eval_->param_map_[name + ".big"].f) = valref.toObject()["big"].toDouble();
        } else {
            // set parameter
            if(name.trimmed() == "bins" || name.trimmed() == "max_nn"){
                *(feature_eval_->param_map_[name].i) = int(valref.toDouble());
            } else {
                *(feature_eval_->param_map_[name].f) = valref.toDouble();
            }
        }

        // Recurse
        setPermuteAndRun(params, fname, idx+1);
    }

}

void AutoTest::runtest() {
    feature_eval_->visualise_on_ = false;
    // Setup reporting
    QFile report_file("report.csv");
    report_file.open(QIODevice::WriteOnly | QIODevice::Text);
    QDebug report(&report_file);
    report.setAutoInsertSpaces(false);
    feature_eval_->setReportFuction(&report);
    feature_eval_->reportHeader();

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
        qDebug() << "No text data: " << err.errorString();
    }

    QJsonObject root = doc.object();
    QJsonObject features = root["features"].toObject();
    QJsonArray tests = root["tests"].toArray();

    for(QJsonValueRef test : tests){
        // setup project
        QJsonObject t = test.toObject();
        QString p = t["project"].toString();
        project_->load(p);
        ////////////////

        qDebug() << "Opening project: " << p << " ... ";

        QJsonArray correlations = t["correlations"].toArray();

        for(QJsonValueRef correlation : correlations){
            QJsonObject c = correlation.toObject();
            QString layer_name = c["layer_name"].toString();
            QJsonArray feature_names = c["features"].toArray();

            feature_eval_->layer_ = layer_name;

            // Select layer
            int idx = ll_->getLayerIdxByName(layer_name);
            if(idx == -1){
                qDebug() << "Cannot find layer: " << layer_name;
                continue;
            }
            //std::vector<int> layers = {idx};
            ll_->selectionChanged({idx});


            for(QJsonValueRef feature_name : feature_names){
                QString fname = feature_name.toString();
                QJsonObject p = features[fname].toObject();

                qDebug() << "Correlating " << fname << " with layer " << layer_name;

                feature_eval_->fname_ = fname;
                feature_eval_->resetParams();

                // Build list of params
                std::vector<std::pair<QString, QJsonArray>> params;

                for(QJsonObject::Iterator it = p.begin(); it != p.end(); it++){
                    params.push_back(std::make_pair(it.key(), it.value().toArray()));
                }

                // run test
                setPermuteAndRun(params, fname, 0);

            }


        }




        // clean up project
        while(ll_->rowCount() > 0){
            ll_->deleteLayer(0);
        }

        while(cl_->rowCount() > 0){
            cl_->removeCloud(0);
        }
        //////////////////////
        report_file.close();
    }
    //features;
    feature_eval_->visualise_on_ = true;

}

void AutoTest::cleanup(){
    mw_->toolbar_->removeAction(enable_);
    mw_->tooloptions_->removeWidget(settings_);
    delete enable_;
}

AutoTest::~AutoTest(){
    
}

void AutoTest::enable() {
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

void AutoTest::disable(){
    enable_->setChecked(false);
    disconnect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    glwidget_->removeEventFilter(this);
    flatview_->removeEventFilter(this);
    is_enabled_ = false;
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.autotest")
