#include <cmath>
#include <ctime>
#include "plugins/markov/markov.h"
#include <QDebug>
#include <QAction>
#include <QToolBar>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QTableView>
#include <QDockWidget>
#include <QHeaderView>
#include <QPushButton>
#include <QLabel>
#include <QDoubleSpinBox>
#include <functional>
#include <boost/serialization/shared_ptr.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/features/principal_curvatures.h>
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "commands/select.h"
#include "pluginsystem/core.h"
#include "plugins/markov/mincut.h"
#include "plugins/normalestimation/normalestimation.h"
#include "utilities/utils.h"
#include "utilities/cv.h"

#include "data.h"
#include "online_rf.h"
#include "experimenter.h"

QString Markov::getName(){
    return "markov";
}

void Markov::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    ll_ = core_->ll_;
    glwidget_ = core_->mw_->glwidget_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;

    pca_radius_ = 0.5f;
    curvature_radius_ = 0.1;
    octree_cell_size_ = 0.05;

    enable_ = new QAction(QIcon(":/settings.png"), "Forest settings", 0);
    forest_action_ = new QAction(QIcon(":/randomforest.png"), "Run random forest", 0);

    feature_list_ = new FeatureList();
    feature_view_ = new QTableView();
    dock_widget_ = new QWidget();
    dock_ = new QDockWidget();

    pca_radius_spinner_ = new QDoubleSpinBox();
    curvature_radius_spinner_ = new QDoubleSpinBox();
    octree_cell_size_spinner_ = new QDoubleSpinBox();

    pca_radius_spinner_->setValue(pca_radius_);
    curvature_radius_spinner_->setValue(curvature_radius_);
    octree_cell_size_spinner_->setValue(octree_cell_size_);

    connect(cl_, &CloudList::updatedActive, [&](){
        pca_dirty_ = true;
        curvatures_dirty_ = true;
        downsample_dirty_ = true;
    });

    connect(pca_radius_spinner_, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=] (double value){
        pca_radius_ = pca_radius_spinner_->value();
        pca_dirty_ = true;
    });

    connect(curvature_radius_spinner_, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=] (double value){
        curvature_radius_ = curvature_radius_spinner_->value();
        curvatures_dirty_ = true;
    });

    connect(octree_cell_size_spinner_, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=] (double value){
        octree_cell_size_ = octree_cell_size_spinner_->value();
        pca_dirty_ = true;
        curvatures_dirty_ = true;
        downsample_dirty_ = true;
    });

}

void Markov::initialize2(PluginManager * pm) {
    ne_ = pm->findPlugin<NormalEstimator>();
    if (ne_ == nullptr) {
        qDebug() << "Normal estimator plugin needed for markov";
        return;
    }


    enable_->setCheckable(true);
    connect(enable_, SIGNAL(triggered()), this, SLOT(enable()));
    mw_->toolbar_->addAction(enable_);

    connect(forest_action_, &QAction::triggered, [this] (bool on) {
        randomforest();
    });

    mw_->toolbar_->addAction(forest_action_);

    enabled_ = false;

    // DOCK

    feature_view_->setModel(feature_list_);
    feature_view_->setColumnWidth(0, 30);
    feature_view_->verticalHeader()->hide();
    feature_view_->horizontalHeader()->hide();

    feature_view_->horizontalHeader()->setStretchLastSection(true);
    feature_view_->resizeRowsToContents();

    QVBoxLayout * dock_layout = new QVBoxLayout();
    dock_layout->addWidget(feature_view_);

//    QPushButton * cache_reset = new QPushButton("Reset cache");

    dock_layout->addWidget(new QLabel("PCA radius"));
    dock_layout->addWidget(pca_radius_spinner_);
    dock_layout->addWidget(new QLabel("Curvature radius"));
    dock_layout->addWidget(curvature_radius_spinner_);
    dock_layout->addWidget(new QLabel("Downsample radius"));
    dock_layout->addWidget(octree_cell_size_spinner_);


//    dock_layout->addStretch();
    dock_layout->setStretch(100, 100);


    dock_widget_->setLayout(dock_layout);
    dock_->setWindowTitle(tr("Forest options"));
    dock_->setWidget(dock_widget_);
}

void Markov::cleanup(){

    if(enabled_){
        disable();
    }

//    mw_->removeDockWidget(dock_widget_);
    mw_->toolbar_->removeAction(enable_);
    mw_->toolbar_->removeAction(forest_action_);
    delete forest_action_;
    delete enable_;
}

Markov::~Markov(){
    qDebug() << "Markov deleted";
//    delete feature_list_;
//    delete feature_view_;
}

void Markov::enable() {
    if(enabled_){
        disable();
        return;
    }

    QTabWidget * tabs = qobject_cast<QTabWidget *>(glwidget_->parent()->parent());
    tabs->setCurrentWidget(glwidget_);
    enable_->setChecked(true);

    emit enabling();

    connect(core_, SIGNAL(endEdit()), this, SLOT(disable()));

    mw_->addDockWidget(Qt::RightDockWidgetArea, dock_);
    mw_->tabifyDockWidget(mw_->options_dock_, dock_);
    dock_->show();
    dock_->raise();

    enabled_ = true;

}

void Markov::disable() {
    enable_->setChecked(false);
    disconnect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    mw_->removeDockWidget(dock_);
    enabled_ = false;
}

void Markov::randomforest(){
    boost::shared_ptr<PointCloud> cloud = core_->cl_->active_;
    if(cloud == nullptr)
        return;

    uint NUM_FEATURES = feature_list_->activeCount();

    bool use_x = feature_list_->hasFeature("X");
    bool use_y = feature_list_->hasFeature("Y");
    bool use_z = feature_list_->hasFeature("Z");
    bool use_i = feature_list_->hasFeature("Intensity");
    bool use_nx = feature_list_->hasFeature("X Normal");
    bool use_ny = feature_list_->hasFeature("Y Normal");
    bool use_nz = feature_list_->hasFeature("Z Normal");
    bool use_e1 = feature_list_->hasFeature("Eigen 1");
    bool use_e2 = feature_list_->hasFeature("Eigen 2");
    bool use_e3 = feature_list_->hasFeature("Eigen 3");
    bool use_pc1 = feature_list_->hasFeature("Curvature 1");
    bool use_pc2 = feature_list_->hasFeature("Curvature 2");
    bool use_a = feature_list_->hasFeature("Anisotrophy");
    bool use_p = feature_list_->hasFeature("Planarity");
    bool use_s = feature_list_->hasFeature("Spherity");
    bool use_l = feature_list_->hasFeature("Linearity");
    bool use_o = feature_list_->hasFeature("Omnivariance");
    bool use_ee = feature_list_->hasFeature("Eigen entrophy");
    bool use_rc = feature_list_->hasFeature("Rusu Curvature");

    clock_t action_start = std::clock();

    // get normals
    pcl::PointCloud<pcl::Normal>::Ptr normals = ne_->getNormals(cl_->active_);

    clock_t downsample_start = std::clock();

    // zip and downsample
    if(downsample_dirty_) {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr zipped = zipNormals(cl_->active_, normals);
        big_to_small_.clear();
        smallcloud_ = octreeDownsample(zipped.get(), octree_cell_size_, big_to_small_);
        downsample_dirty_ = false;
    }

    double downsample_elapsed = double(std::clock() - downsample_start) / CLOCKS_PER_SEC;

    clock_t pca_start = std::clock();

    // PCA
    if(pca_dirty_) {
        pca_ = getPCA(smallcloud_.get(), pca_radius_, 0);
        pca_dirty_ = false;
    }

    double pca_elapsed = double(std::clock() - pca_start) / CLOCKS_PER_SEC;

    clock_t curvature_start = std::clock();

    // Curvature
    // pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures_;
    if((use_pc1 || use_pc2) && curvatures_dirty_) {
        pcl::PrincipalCurvaturesEstimation<pcl::PointXYZINormal, pcl::PointXYZINormal, pcl::PrincipalCurvatures> principalCurvaturesEstimation;
        principalCurvaturesEstimation.setInputCloud(smallcloud_);
        principalCurvaturesEstimation.setInputNormals(smallcloud_);

        pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZINormal>);
        tree->setInputCloud(smallcloud_);
        principalCurvaturesEstimation.setSearchMethod (tree);
        principalCurvaturesEstimation.setRadiusSearch(curvature_radius_);

        principal_curvatures_ = pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr(new pcl::PointCloud<pcl::PrincipalCurvatures> ());
        principalCurvaturesEstimation.compute (*principal_curvatures_);
        curvatures_dirty_ = false;
    }

    double curvature_elapsed = double(std::clock() - curvature_start) / CLOCKS_PER_SEC;

    // Feature compute
    auto mkFeatureVector = [&](uint idx) {
        Eigen::VectorXd vec(NUM_FEATURES);
        vec.setZero();

        int featnum = -1;

        //set samples
        if(use_x)
            vec(++featnum) = smallcloud_->at(idx).x;
        if(use_y)
            vec(++featnum) = smallcloud_->at(idx).y;
        if(use_z)
            vec(++featnum) = smallcloud_->at(idx).z;
        if(use_i)
            vec(++featnum) = smallcloud_->at(idx).intensity;
        if(use_nx)
            vec(++featnum) = smallcloud_->at(idx).normal_x;
        if(use_ny)
            vec(++featnum) = smallcloud_->at(idx).normal_y;
        if(use_nz)
            vec(++featnum) = smallcloud_->at(idx).normal_z;

        Eigen::Vector3f & pca = (*pca_)[idx];

        if(use_e1)
            vec(++featnum) = pca[0];
        if(use_e2)
            vec(++featnum) = pca[1];
        if(use_e3)
            vec(++featnum) = pca[2];

        if (use_pc1 || use_pc2) {
            const pcl::PrincipalCurvatures & curv = principal_curvatures_->points[idx];

            if(use_pc1)
                vec(++featnum) = curv.pc1;
            if(use_pc2)
                vec(++featnum) = curv.pc2;
        }


        // anisotrophy =
        if(use_a)
            vec(++featnum) = (pca[0] - pca[2]) / pca[0];

        // planarity
        if(use_p)
            vec(++featnum) = (pca[1] - pca[2]) / pca[0];

        // spherity
        if(use_s)
            vec(++featnum) = pca[2] / pca[0];

        // linearity
        if(use_l)
            vec(++featnum) = (pca[0] - pca[1]) / pca[0];

        // omnivariance
        if(use_o)
            vec(++featnum) = std::cbrt(pca[0] * pca[1] * pca[0]);

        // eigen entrophy
        if(use_ee)
            vec(++featnum) = -(pca[0] * std::log(pca[0]) + pca[1] * std::log(pca[1]) + pca[2] * std::log(pca[0]));

        // rusu curvature estimation
        if(use_rc)
            vec(++featnum) = pca[0] / (pca[0] + pca[1] + pca[2]);

        return vec;
    };

    /// Random forest

    Hyperparameters hp;

    // Forest
    hp.maxDepth = 10;
    hp.numRandomTests = 1;
    hp.counterThreshold = 140;
    hp.numTrees = 100;

    // Experimenter
    hp.numEpochs = 1;
    hp.findTrainError = 0;

    // Output
    hp.verbose = 1;

    clock_t get_selections_start = std::clock();

    // Load selection
    std::vector<boost::shared_ptr<std::vector<int>>> selections = cloud->getSelections();

    double get_selections_elapsed = double(std::clock() - get_selections_start) / CLOCKS_PER_SEC;

    clock_t feature_compute_start = std::clock();

    std::vector<int> selection_sources;

    for(uint i = 0; i < selections.size(); i++) {
        if(selections[i]->size() > 30)
            selection_sources.push_back(i);
    }

    if(selection_sources.size() < 2) {
        qDebug() << "Not enough data";
        return;
    }

    // Creating the train data
    DataSet dataset_train;
    dataset_train.m_numFeatures = NUM_FEATURES;
    dataset_train.m_numClasses = 8;

    std::set<int> seen;

    uint sample_size = 0;
    uint smallest_class_size = 999999999;
    for(uint s : selection_sources) {
        sample_size += selections[s]->size();
        smallest_class_size = selections[s]->size() < smallest_class_size ? selections[s]->size() : smallest_class_size;
    }

    const int MAX = 999999999;
    int max_samples = smallest_class_size * selection_sources.size();
    max_samples = max_samples > MAX ? MAX : max_samples;

    const uint max_samples_per_class = MAX/selection_sources.size();
    int samples_per_class = smallest_class_size > max_samples_per_class ? max_samples_per_class : smallest_class_size;

    int count = 0;
    for(uint y : selection_sources) {

        const int sample_count = selections[y]->size();
        const float sample_pct = float(sample_count)/sample_size;
        int s_nth = sample_count / samples_per_class;
        s_nth = s_nth < 1 ? 1 : s_nth;

        qDebug() << "label: " << y << ", pct: " << 100*sample_pct << ", samples " << sample_count << ", every nth point; " << s_nth;

        for(int big_idx : *selections[y]) {
            int idx = big_to_small_[big_idx];

            if(!seen.insert(idx).second)
                continue;

            count++;

            if(count % s_nth != 0)
                continue;

            Sample sample;
            sample.id = idx;
            sample.w = 1.0;
            sample.y = y;
            sample.x = mkFeatureVector(idx);

            // feature compute
            dataset_train.m_samples.push_back(sample);
        }

    }

    dataset_train.m_numSamples = dataset_train.m_samples.size();

    qDebug() << "Size tr:" << dataset_train.m_numSamples;

    if(dataset_train.m_numSamples < 10){
        qDebug() << "Not enough samples";
        return;
    }

    double feature_compute_elapsed = double(std::clock() - feature_compute_start) / CLOCKS_PER_SEC;

    clock_t train_start = std::clock();
    dataset_train.findFeatRange();

    OnlineRF model(hp, dataset_train.m_numClasses, dataset_train.m_numFeatures, dataset_train.m_minFeatRange, dataset_train.m_maxFeatRange);
    train(&model, dataset_train, hp);

    double train_elapsed = double(std::clock() - train_start) / CLOCKS_PER_SEC;

    Result invalid(8);
    invalid.prediction = -1;

    clock_t classify_start = std::clock();

    std::vector<Result> results(smallcloud_->points.size(), invalid);

    for(size_t idx = 0; idx < smallcloud_->points.size(); ++idx) {
        Sample sample;
        sample.id = idx;
        sample.w = 1.0;
        sample.x = mkFeatureVector(idx);
        model.eval(sample, results[idx]);
    }

    double classify_elapsed = double(std::clock() - classify_start) / CLOCKS_PER_SEC;

    clock_t result_select_start = std::clock();

    // Select fg and bg
    std::vector<boost::shared_ptr<std::vector<int>>> seg_selections(8);
    for(int i = 0; i < 8; i++){
        seg_selections[i] = boost::make_shared<std::vector<int>>();
    }

    for(size_t idx = 0; idx < cloud->points.size(); ++idx) {
        int idx_small = big_to_small_[idx];
        Result & res = results[idx_small];
        seg_selections[res.prediction]->push_back(idx);
    }

    core_->us_->beginMacro("Random forest");

    for(int i = 0; i < 8; i++){
        if(seg_selections[i]->size() > 0) {
            core_->us_->push(new Select(cl_->active_, seg_selections[i], false, 1 << i, true, ll_->getHiddenLabels()));
        }
    }

    double result_select_elapsed = double(std::clock() - result_select_start) / CLOCKS_PER_SEC;
    double action_elapsed = double(std::clock() - action_start) / CLOCKS_PER_SEC;

    core_->us_->endMacro();

    qDebug() << "downsample: " << downsample_elapsed << "\n"
                 << "pca: "<< pca_elapsed << "\n"
                 << "curvature: "<< curvature_elapsed << "\n"
                 << "get_selections: "<< get_selections_elapsed << "\n"
                 << "feature_compute: "<< feature_compute_elapsed << "\n"
                 << "train: "<< train_elapsed << "\n"
                 << "classify: "<< classify_elapsed << "\n"
                 << "result_select: "<< result_select_elapsed << "\n"
                 << "total: "<< action_elapsed;
}


Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin")
