#include "plugins/autotest/autotest.h"
#include <utility>
#include <functional>
#include <numeric>
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
#include <QListWidget>
#include <QDoubleSpinBox>
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
#include "utilities/utils.h"

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

    radius_spinner_ = new QDoubleSpinBox();
    radius_ = 0.25;

    enable_ = new QAction(QIcon(":/autotest.png"), "Enable AutoTest", 0);
    enable_->setCheckable(true);

    connect(enable_, SIGNAL(triggered()), this, SLOT(enable()));
}

void AutoTest::initialize2(PluginManager *pm){
    project_ = pm->findPlugin<Project>();
    markov_ = pm->findPlugin<Markov>();
    accuracy_ = pm->findPlugin<Accuracy>();

    // Widgets


    QPushButton * open_button = new QPushButton("Open test");
    QPushButton * run_button = new QPushButton("Run test");
    QPushButton * run_feature_eval_button = new QPushButton("Run feature eval");
    QPushButton * run_knn_button = new QPushButton("Run knn test");

    run_feature_eval_button->setDisabled(true);
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
        run_feature_eval_button->setDisabled(false);
        run_button->setDisabled(false);
        test_path_ = filename;

    });

    connect(run_feature_eval_button, &QPushButton::clicked, (std::function<void()>) std::bind(&AutoTest::run_feature_selection, this));
    connect(run_button, &QPushButton::clicked, (std::function<void()>) std::bind(&AutoTest::runtests, this));

    connect(run_knn_button, &QPushButton::clicked, (std::function<void()>) std::bind(&AutoTest::knntest, this));
//    connect(run_knn_button, &QPushButton::clicked, [&](){

//    });


    // Toolbar
    mw_->toolbar_->addAction(enable_);


    // Dock
    QVBoxLayout * dock_layout = new QVBoxLayout();

    dock_layout->addWidget(new QLabel("GAR!"));
    dock_layout->addWidget(open_button);
    dock_layout->addWidget(run_button);
    dock_layout->addWidget(run_feature_eval_button);
    dock_layout->addWidget(run_knn_button);
    dock_layout->setStretch(100, 100);


    dock_widget_->setLayout(dock_layout);
    dock_->setWindowTitle(tr("Autotest options"));
    dock_->setWidget(dock_widget_);
}

void AutoTest::knntest() {
    boost::shared_ptr<PointCloud> cloud = core_->cl_->active_;
    if(cloud == nullptr)
        return;

    QFile report_file("knnreport.csv");
    report_file.open(QIODevice::Append | QIODevice::Text);
    QDebug report(&report_file);
    report.setAutoInsertSpaces(false);


    report << "voxel size,knn radius,original size,downsampled cloud size,avg points,downsample seconds,knn seconds,total seconds\n";

    qDebug() << "voxel size,knn radius,original size,downsampled cloud size,avg points,downsample seconds,knn seconds,total seconds";

    for(float voxel_size = 0.005f; voxel_size <= 0.1; voxel_size+=0.005) {

        std::vector<int> big_to_small;

        clock_t downsample_start = std::clock();
        pcl::PointCloud<pcl::PointXYZI>::Ptr smallcloud_ = octreeDownsample(cloud.get(), voxel_size, big_to_small);
        double downsample_elapsed = double(std::clock() - downsample_start) / CLOCKS_PER_SEC;

        for(float nn_radius = 0; nn_radius <= 0.5; nn_radius+=0.05) {

            if (nn_radius < voxel_size) continue;

            clock_t nn_search_start = std::clock();
            double total = 0;

            typename pcl::PointCloud<pcl::PointXYZI>::ConstPtr cptr(smallcloud_.get(), boost::serialization::null_deleter());
            pcl::KdTreeFLANN<pcl::PointXYZI> search;
            search.setInputCloud(cptr);

            std::vector<float> kDist;
            std::vector<int> kIdxs;

            bool stopped_early = false;

            for(uint i = 0; i < smallcloud_->size(); i++){
                search.radiusSearch(i, nn_radius, kIdxs, kDist);
                total+=kIdxs.size();

                if (i % 500 == 0) {
                    double nn_search_elapsed = double(std::clock() - nn_search_start) / CLOCKS_PER_SEC;
                    if (nn_search_elapsed > 60) {
                        stopped_early = true;
                        break;
                    }
                }
            }

            if (stopped_early) {
                qDebug() << "stopped early: " << voxel_size << "," << nn_radius;
                break;
            }

            double nn_search_elapsed = double(std::clock() - nn_search_start) / CLOCKS_PER_SEC;

            report << voxel_size << "," <<
                      nn_radius << "," <<
                      cloud->size() << "," <<
                      smallcloud_->size() << "," <<
                      double(total)/smallcloud_->size() << "," <<
                      downsample_elapsed << "," <<
                      nn_search_elapsed << "," <<
                      nn_search_elapsed + downsample_elapsed << "\n";

            qDebug() << voxel_size << "," <<
                      nn_radius << "," <<
                      cloud->size() << "," <<
                      smallcloud_->size() << "," <<
                      double(total)/smallcloud_->size() << "," <<
                      downsample_elapsed << "," <<
                      nn_search_elapsed << "," <<
                      nn_search_elapsed + downsample_elapsed;

//            clock_t upsample_start = std::clock();

//            std::vector<int> trash;
//            for(int idx = 0; idx < cloud->size(); idx++) {
//                trash.push_back(big_to_small[idx]);
//            }

//            double upsample_elapsed = double(std::clock() - upsample_start) / CLOCKS_PER_SEC;

//            qDebug() << "upsample time" << upsample_elapsed << "trash: " << trash.size();
        }
    }


}

std::tuple<float, float, float, double,double,double,double,double,double,double,double,double,double,double> AutoTest::runTest(
        std::vector<std::string> features,
        float downsample,
        float curvature_radius,
        float pca_radius,
        float density_radius,
        int pca_max_nn,
        int tree_count,
        int tree_depth,
        int tree_counter_threshold,
        int tree_random_tests){

    markov_->pca_radius_spinner_->setValue(pca_radius);
    markov_->curvature_radius_spinner_->setValue(curvature_radius);
    markov_->octree_cell_size_spinner_->setValue(downsample);
    markov_->density_radius_spinner_->setValue(density_radius);

    markov_->tree_count_spinner_->setValue(tree_count);
    markov_->tree_depth_spinner_->setValue(tree_depth);
    markov_->max_nn_spinner_->setValue(pca_max_nn);;

    markov_->tree_counter_threshold_spinner_->setValue(tree_counter_threshold);
    markov_->tree_random_tests_spinner_->setValue(tree_random_tests);

    markov_->feature_list_->selectOnly(features);

    QCoreApplication::processEvents(); // Need this for settings to take effect?

    double downsample_elapsed;
    double pca_elapsed;
    double curvature_elapsed;
    double density_elapsed;
    double get_selections_elapsed;
    double feature_compute_elapsed;
    double train_elapsed;
    double feature_compute_2_elapsed;
    double classify_elapsed;
    double result_select_elapsed;
    double action_elapsed;

    std::tie(downsample_elapsed,
             pca_elapsed,
             curvature_elapsed,
             density_elapsed,
             get_selections_elapsed,
             feature_compute_elapsed,
             train_elapsed,
             feature_compute_2_elapsed,
             classify_elapsed,
             result_select_elapsed,
             action_elapsed) = markov_->randomforest();

    float accuracy, precision, recall;
    std::tie(accuracy, precision, recall) = accuracy_->sample();

    core_->us_->undo();

    QCoreApplication::processEvents();

    qDebug() << "Results" << accuracy << precision <<  recall;
    return std::make_tuple(accuracy,
                           precision,
                           recall,
                           downsample_elapsed,
                           pca_elapsed,
                           curvature_elapsed,
                           density_elapsed,
                           get_selections_elapsed,
                           feature_compute_elapsed,
                           train_elapsed,
                           feature_compute_2_elapsed,
                           classify_elapsed,
                           result_select_elapsed,
                           action_elapsed);
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
    QJsonArray scans = root["scans"].toArray();

    // Setup reporting
    QFile report_file("report.csv");
    report_file.open(QIODevice::Append | QIODevice::Text);
    QTextStream report(&report_file);

    for(QJsonValueRef s : scans){
        QJsonObject scan = s.toObject();
        QString path = scan["path"].toString();
        QString target_layer_name = scan["targetLayerName"].toString();
        uint8_t result_selection = scan["resultSelection"].toInt();

        mw_->reset();
        project_->load(path);
        accuracy_->target_layers_->clear();
        accuracy_->target_layers_->addItem(target_layer_name);

        for(const boost::shared_ptr<Layer> & l : ll_->getLayers()){
            if(l->getName() == target_layer_name) {
                accuracy_->target_.push_back(l);
            }
        }

        mw_->setSelectMask(1 << result_selection);

        report << "File:" << test_path_ << ", Project: " << path << "\n";
        report << "Feature,accuracy,accuracy_std,precision,precision_std,recall,recall_std," <<
                "downsample_elapsed," <<
                "pca_elapsed," <<
                "curvature_elapsed," <<
                "density_elapsed," <<
                "get_selections_elapsed," <<
                "feature_compute_elapsed," <<
                "train_elapsed," <<
                "feature_compute_2_elapsed," <<
                "classify_elapsed," <<
                "result_select_elapsed," <<
                "action_elapsed\n";


        qDebug() << "Starting: " << path << "\n";

        report.flush();

        for(QJsonValueRef test : tests){
            QJsonObject t = test.toObject();

            QJsonArray featuresArray = t.contains("features") ? t["features"].toArray() : defaults["features"].toArray();

            qDebug() << "Test:" << t["name"].toString();

            std::vector<std::string> features;
            for(QJsonValueRef nameRef : featuresArray){
                features.push_back(nameRef.toString().toStdString());
            }

            QString name = t["name"].toString();
            float downsample = t.contains("downsample") ? t["downsample"].toDouble() : defaults["downsample"].toDouble();
            float curvature_radius = t.contains("curvature_radius") ? t["curvature_radius"].toDouble() : defaults["curvature_radius"].toDouble();
            float pca_radius = t.contains("pca_radius") ? t["pca_radius"].toDouble() : defaults["pca_radius"].toDouble();
            float density_radius = t.contains("downsample") ? t["density_radius"].toDouble() : defaults["density_radius"].toDouble();
            int pca_max_nn = t.contains("pca_max_nn") ? t["pca_max_nn"].toInt() : defaults["pca_max_nn"].toInt();
            int tree_count = t.contains("tree_count") ? t["tree_count"].toInt() : defaults["tree_count"].toInt();
            int tree_depth = t.contains("tree_depth") ? t["tree_depth"].toInt() : defaults["tree_depth"].toInt();
            int tree_counter_threshold = t.contains("tree_counter_threshold") ? t["tree_counter_threshold"].toInt() : defaults["tree_counter_threshold"].toInt();
            int tree_random_tests = t.contains("tree_random_tests") ? t["tree_random_tests"].toInt() : defaults["tree_random_tests"].toInt();

            std::vector<float> accuracies;
            std::vector<float> precisions;
            std::vector<float> recalls;

            float accuracy = 0, precision = 0, recall = 0;
            float accuracy_std = 0, precision_std = 0, recall_std = 0;
            double downsample_elapsed;
            double pca_elapsed;
            double curvature_elapsed;
            double density_elapsed;
            double get_selections_elapsed;
            double feature_compute_elapsed;
            double train_elapsed;
            double feature_compute_2_elapsed;
            double classify_elapsed;
            double result_select_elapsed;
            double action_elapsed;

            const int iterations = 5;

            for (int i = 0; i < iterations; i++) {
                float _accuracy, _precision, _recall;
                double _downsample_elapsed;
                double _pca_elapsed;
                double _curvature_elapsed;
                double _density_elapsed;
                double _get_selections_elapsed;
                double _feature_compute_elapsed;
                double _train_elapsed;
                double _feature_compute_2_elapsed;
                double _classify_elapsed;
                double _result_select_elapsed;
                double _action_elapsed;

                std::tie(_accuracy,
                         _precision,
                         _recall,
                         _downsample_elapsed,
                         _pca_elapsed,
                         _curvature_elapsed,
                         _density_elapsed,
                         _get_selections_elapsed,
                         _feature_compute_elapsed,
                         _train_elapsed,
                         _feature_compute_2_elapsed,
                         _classify_elapsed,
                         _result_select_elapsed,
                         _action_elapsed) = runTest(
                                                features,
                                                downsample,
                                                curvature_radius,
                                                pca_radius,
                                                density_radius,
                                                pca_max_nn,
                                                tree_count,
                                                tree_depth,
                                                tree_counter_threshold,
                                                tree_random_tests);

                accuracy += _accuracy;
                precision += _precision;
                recall += _recall;

                accuracies.push_back(accuracy);
                precisions.push_back(precision);
                recalls.push_back(recall);

                if (i == 0) {
                    downsample_elapsed = _downsample_elapsed;
                    pca_elapsed = _pca_elapsed;
                    curvature_elapsed = _curvature_elapsed;
                    density_elapsed = _density_elapsed;
                    get_selections_elapsed = _get_selections_elapsed;
                    feature_compute_elapsed = _feature_compute_elapsed;
                    train_elapsed = _train_elapsed;
                    feature_compute_2_elapsed = _feature_compute_2_elapsed;
                    classify_elapsed = _classify_elapsed;
                    result_select_elapsed = _result_select_elapsed;
                    action_elapsed = _action_elapsed;
                }
            }

            accuracy /= iterations;
            precision /= iterations;
            recall /= iterations;

            float a_sum = 0;
            float p_sum = 0;
            float r_sum = 0;

            for (int i = 0; i < iterations; i++) {
                a_sum += pow(accuracies[i] - accuracy_std, 2);
                p_sum += pow(precisions[i] - precision_std, 2);
                r_sum += pow(recalls[i] - recall_std, 2);
            }

            accuracy_std = sqrt(a_sum/(iterations-1));
            precision_std = sqrt(p_sum/(iterations-1));
            recall_std = sqrt(r_sum/(iterations-1));

            report << name << ","
                    << accuracy << ","
                    << accuracy_std << ","
                    << precision << ","
                    << precision_std << ","
                    << recall << ","
                    << recall_std << ","
                    << downsample_elapsed << ","
                    << pca_elapsed << ","
                    << curvature_elapsed << ","
                    << density_elapsed << ","
                    << get_selections_elapsed << ","
                    << feature_compute_elapsed << ","
                    << train_elapsed << ","
                    << feature_compute_2_elapsed << ","
                    << classify_elapsed << ","
                    << result_select_elapsed << ","
                    << action_elapsed << "\n";

            report.flush();
            report_file.flush();

        }

    }

    report_file.close();
}


void AutoTest::run_feature_selection() {

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
    QJsonArray ordered_feature_sets = root["tests"].toArray();
    QJsonArray scans = root["scans"].toArray();

    // Setup reporting
    QFile report_file("feature-selection-report.csv");
    report_file.open(QIODevice::Append | QIODevice::Text);
    QTextStream report(&report_file);

    report << "features,noise,people,tools,tree,mean\n";


    // setup defaults
    float downsample = defaults["downsample"].toDouble();
    float curvature_radius = defaults["curvature_radius"].toDouble();
    float pca_radius = defaults["pca_radius"].toDouble();
    float density_radius = defaults["density_radius"].toDouble();
    int pca_max_nn = defaults["pca_max_nn"].toInt();
    int tree_count = defaults["tree_count"].toInt();
    int tree_depth = defaults["tree_depth"].toInt();
    int tree_counter_threshold = defaults["tree_counter_threshold"].toInt();
    int tree_random_tests = defaults["tree_random_tests"].toInt();


    float max_accuracy = 0;
    std::set<uint> included_feature_sets_idxs = {};


    // here we try to add the next feature

    for(uint i = 0; i < ordered_feature_sets.size(); i++){
        std::vector<std::string> features;
        float feature_accuracy = 0;

        std::set<uint> candidate_feature_sets_idxs = included_feature_sets_idxs;
        candidate_feature_sets_idxs.insert(i);

        QString candidate_feature_name;

        for(uint idx : candidate_feature_sets_idxs) {
            QJsonObject f = ordered_feature_sets[idx].toObject();


            for(QJsonValueRef nameRef : f["features"].toArray()){
                features.push_back(nameRef.toString().toStdString());
            }

            candidate_feature_name += f["name"].toString() + " + ";
        }

        report << candidate_feature_name << ",";

        // Should I add previously viewed features?

        float accuracy, precision, recall;
        double downsample_elapsed;
        double pca_elapsed;
        double curvature_elapsed;
        double density_elapsed;
        double get_selections_elapsed;
        double feature_compute_elapsed;
        double train_elapsed;
        double feature_compute_2_elapsed;
        double classify_elapsed;
        double result_select_elapsed;
        double action_elapsed;

        for(QJsonValueRef s : scans){
            QJsonObject scan = s.toObject();
            QString path = scan["path"].toString();
            QString target_layer_name = scan["targetLayerName"].toString();
            uint8_t result_selection = scan["resultSelection"].toInt();

            mw_->reset();
            project_->load(path);
            accuracy_->target_layers_->clear();
            accuracy_->target_layers_->addItem(target_layer_name);

            for(const boost::shared_ptr<Layer> & l : ll_->getLayers()){
                if(l->getName() == target_layer_name) {
                    accuracy_->target_.push_back(l);
                }
            }

            mw_->setSelectMask(1 << result_selection);

            // after setup...

            float iteration_accuracy_sum = 0;

            for (int k = 0; k < 3; k++) {
                std::tie(accuracy,
                         precision,
                         recall,
                         downsample_elapsed,
                         pca_elapsed,
                         curvature_elapsed,
                         density_elapsed,
                         get_selections_elapsed,
                         feature_compute_elapsed,
                         train_elapsed,
                         feature_compute_2_elapsed,
                         classify_elapsed,
                         result_select_elapsed,
                         action_elapsed) = runTest(
                                                features,
                                                downsample,
                                                curvature_radius,
                                                pca_radius,
                                                density_radius,
                                                pca_max_nn,
                                                tree_count,
                                                tree_depth,
                                                tree_counter_threshold,
                                                tree_random_tests);

                iteration_accuracy_sum += accuracy;
            }

            feature_accuracy += iteration_accuracy_sum/3.0f;
            report << iteration_accuracy_sum/3.0f << ",";
            report.flush();
            report_file.flush();
        }

        feature_accuracy /= 4;

        report << feature_accuracy << "\n";

        if(feature_accuracy > (max_accuracy + 0.01)) {
            max_accuracy = feature_accuracy;
            included_feature_sets_idxs = candidate_feature_sets_idxs;
        }

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
