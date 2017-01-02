#include "plugins/markov/markov.h"
#include <QDebug>
#include <QAction>
#include <QToolBar>
#include <QMessageBox>
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
#include "utilities/picker.h"
#include "utilities/cv.h"

#include "data.h"
#include "online_rf.h"
#include "experimenter.h"


#include <eigenlibsvm/svm_utils.h>
#include <eigenlibsvm/eigen_extensions.h>

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

}

void Markov::initialize2(PluginManager * pm) {
    ne_ = pm->findPlugin<NormalEstimator>();
    if (ne_ == nullptr) {
        qDebug() << "Normal estimator plugin needed for markov";
        return;
    }

    forrest_action_ = new QAction(QIcon(":/randomforest.png"), "forrest action", 0);
    connect(forrest_action_, &QAction::triggered, [this] (bool on) {
        randomforest();
    });

    mw_->toolbar_->addAction(forrest_action_);

    enabled_ = false;
    fg_idx_ = -1;
}

void Markov::cleanup(){
    mw_->toolbar_->removeAction(forrest_action_);
    delete forrest_action_;
}

Markov::~Markov(){
    qDebug() << "Markov deleted";
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

    glwidget_->installEventFilter(picker_);
    connect(core_, SIGNAL(endEdit()), this, SLOT(disable()));

    enabled_ = true;
    // Let the user know what to do
    QMessageBox::information(nullptr, tr("Select foreground"),
                    tr("Select the center of an object..."),
                    QMessageBox::Ok, QMessageBox::Ok);


}

void Markov::disable() {
    enable_->setChecked(false);
    disconnect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    glwidget_->removeEventFilter(picker_);
    enabled_ = false;
}

double timeIt(int reset) {
    static time_t startTime, endTime;
    static int timerWorking = 0;

    if (reset) {
        startTime = time(NULL);
        timerWorking = 1;
        return -1;
    } else {
        if (timerWorking) {
            endTime = time(NULL);
            timerWorking = 0;
            return (double) (endTime - startTime);
        } else {
            startTime = time(NULL);
            timerWorking = 1;
            return -1;
        }
    }
}

void Markov::randomforest(){
    qDebug() << "Random forest";

    boost::shared_ptr<PointCloud> cloud = core_->cl_->active_;
    if(cloud == nullptr)
        return;

    // get normals
    pcl::PointCloud<pcl::Normal>::Ptr normals = ne_->getNormals(cl_->active_);

    // zip and downsample
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr smallcloud = zipNormals(cl_->active_, normals);
    std::vector<int> big_to_small;
    smallcloud = octreeDownsample(smallcloud.get(), 0.05, big_to_small);




    // PCA
    boost::shared_ptr<std::vector<Eigen::Vector3f> > pca = getPCA(smallcloud.get(), 0.5f, 0);

    // Curvature
    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZINormal, pcl::PointXYZINormal, pcl::PrincipalCurvatures> principalCurvaturesEstimation;
    principalCurvaturesEstimation.setInputCloud(smallcloud);
    principalCurvaturesEstimation.setInputNormals(smallcloud);

    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZINormal>);
    tree->setInputCloud(smallcloud);
    principalCurvaturesEstimation.setSearchMethod (tree);
    principalCurvaturesEstimation.setRadiusSearch(0.5);

    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
    principalCurvaturesEstimation.compute (*principalCurvatures);

    const uint NUM_FEATURES = 16;

    // Feature compute
    auto mkFeatureVector = [&](uint idx) {
        Eigen::VectorXd vec(NUM_FEATURES);


//        vec(0) = 0;
//        vec(1) = 0;
//        vec(2) = 0;
//        vec(3) = 0;
//        vec(4) = 0;
//        vec(5) = 0;
//        vec(6) = 0;
//        vec(7) = 0;
//        vec(8) = 0;
//        vec(9) = 0;
//        vec(10) = 0;
//        vec(11) = 0;
//        vec(12) = 0;

        //set samples
        vec(0) = smallcloud->at(idx).x;
        vec(1) = smallcloud->at(idx).y;
        vec(2) = smallcloud->at(idx).z;
        vec(3) = smallcloud->at(idx).intensity;
        vec(4) = smallcloud->at(idx).normal_x;
        vec(5) = smallcloud->at(idx).normal_y;
        vec(6) = smallcloud->at(idx).normal_z;

        Eigen::Vector3f & pca_ = (*pca)[idx];

        vec(7) = pca_[0];
        vec(8) = pca_[1];
        vec(9) = pca_[2];

        const pcl::PrincipalCurvatures & curv = principalCurvatures->points[idx];

        vec(10) = curv.pc1;
        vec(11) = curv.pc2;

        // anisotrophy
        vec(12) = (pca_[0] - pca_[2]) / pca_[0];

        // planarity
        vec(13) = (pca_[1] - pca_[2]) / pca_[0];

        // spherity
        vec(14) = pca_[2] / pca_[0];

        // linearity
        vec(15) = (pca_[0] - pca_[1]) / pca_[0];

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

    // Load selection
    std::vector<boost::shared_ptr<std::vector<int>>> selections = cloud->getSelections();

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

//    std::set<int> labels;
    std::set<int> seen;

    uint sample_size = 0;
    uint smallest_class_size = 999999999;
    for(uint s : selection_sources) {
        sample_size += selections[s]->size();
        smallest_class_size = selections[s]->size() < smallest_class_size ? selections[s]->size() : smallest_class_size;
    }

//    int MAX = 2000;
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
            int idx = big_to_small[big_idx];

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

    dataset_train.findFeatRange();

    OnlineRF model(hp, dataset_train.m_numClasses, dataset_train.m_numFeatures, dataset_train.m_minFeatRange, dataset_train.m_maxFeatRange);

    timeIt(1);
    train(&model, dataset_train, hp);
    qDebug() << "Training/Test time: " << timeIt(0) << endl;

    Result invalid(8);
    //invalid.confidence.at(0) = 0;
    invalid.prediction = -1;

    std::vector<Result> results(smallcloud->points.size(), invalid);

    for(size_t idx = 0; idx < smallcloud->points.size(); ++idx) {
        Sample sample;
        sample.id = idx;
        sample.w = 1.0;
        sample.x = mkFeatureVector(idx);

        model.eval(sample, results[idx]);
    }

    // Select fg and bg
    std::vector<boost::shared_ptr<std::vector<int>>> seg_selections(8);
    for(int i = 0; i < 8; i++){
        seg_selections[i] = boost::make_shared<std::vector<int>>();
    }

    for(size_t idx = 0; idx < cloud->points.size(); ++idx) {
        int idx_small = big_to_small[idx];
        Result & res = results[idx_small];
        seg_selections[res.prediction]->push_back(idx);
    }

    core_->us_->beginMacro("Random forest");

    for(int i = 0; i < 8; i++){
        if(seg_selections[i]->size() > 0) {
            core_->us_->push(new Select(cl_->active_, seg_selections[i], false, 1 << i, true, ll_->getHiddenLabels()));
        }
    }

    core_->us_->endMacro();
}


Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin")
