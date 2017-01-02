#include "plugins/markov/markov.h"
#include <QDebug>
#include <QAction>
#include <QToolBar>
#include <QMessageBox>
#include <functional>
#include <boost/serialization/shared_ptr.hpp>
#include <pcl/search/kdtree.h>
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

    //enable_ = new QAction(QIcon(":/markov.png"), "markov action", 0);
    //enable_->setCheckable(true);

    //connect(enable_,&QAction::triggered, [this] (bool on) {
    //    graphcut();
    //});

    //mw_->toolbar_->addAction(enable_);
    //std::function<void(int)> func = std::bind(&Markov::graphcut, this, std::placeholders::_1);
    //picker_ = new Picker(glwidget_, cl_, func);

    forrest_action_ = new QAction(QIcon(":/randomforest.png"), "forrest action", 0);
    connect(forrest_action_, &QAction::triggered, [this] (bool on) {
        randomforest();
    });

    mw_->toolbar_->addAction(forrest_action_);


    enabled_ = false;
    fg_idx_ = -1;
}

void Markov::cleanup(){
    //mw_->toolbar_->removeAction(enable_);
    mw_->toolbar_->removeAction(forrest_action_);
    //delete enable_;
    //delete picker_;
    delete forrest_action_;
    //delete svm_action_;
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

    /// Random forest

    // Hyperparameters
    Hyperparameters hp;

    // Forest
    hp.maxDepth = 10;
    hp.numRandomTests = 10;
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
    dataset_train.m_numFeatures = 10;


    std::set<int> labels;
    std::set<int> seen;

    int sample_size = 0;
    for(uint s : selection_sources) {
        sample_size += selections[s]->size();
    }

    int MAX = 1000000;
    int max_samples = sample_size > MAX ? MAX : sample_size;
    int nth = sample_size / max_samples;

    int count = 0;
    for(uint y : selection_sources) {

        int sample_count = selections[y]->size();
        float sample_pct = sample_count/sample_size;
//        int use_samples = max_samples * (1.0f/sample_pct);
//        int nth = use_samples / max_samples;

        int s_nth = nth * sample_pct;
        s_nth = s_nth < 1 ? 1 : nth;

        qDebug() << "label: " << y << ", pct: " << sample_pct << ", samples " << sample_count << ", every nth point; " << s_nth;

        for(int big_idx : *selections[y]) {
            int idx = big_to_small[big_idx];

            if(!seen.insert(idx).second)
                continue;

            count++;

            if(count % s_nth != 0)
                continue;

            Sample sample;
            sample.x = Eigen::VectorXd(dataset_train.m_numFeatures);
            sample.id = idx;
            sample.w = 1.0;
            sample.y = y;

            labels.insert(y);

//            sample.x(0) = 0;
//            sample.x(1) = 0;
//            sample.x(2) = 0;
//            sample.x(3) = 0;
//            sample.x(4) = 0;
//            sample.x(5) = 0;
//            sample.x(6) = 0;
//            sample.x(7) = 0;
//            sample.x(8) = 0;
//            sample.x(9) = 0;

            //set samples
            sample.x(0) = smallcloud->at(idx).x;
            sample.x(1) = smallcloud->at(idx).y;
            sample.x(2) = smallcloud->at(idx).z;
            sample.x(3) = smallcloud->at(idx).intensity;
            sample.x(4) = smallcloud->at(idx).normal_x;
            sample.x(5) = smallcloud->at(idx).normal_y;
            sample.x(6) = smallcloud->at(idx).normal_z;
            sample.x(7) = (*pca)[idx][0];
            sample.x(8) = (*pca)[idx][1];
            sample.x(9) = (*pca)[idx][2];


            dataset_train.m_samples.push_back(sample);

        }

    }

    dataset_train.m_numClasses = 10;
    dataset_train.m_numSamples = dataset_train.m_samples.size();

    qDebug() << "Size tr:" << dataset_train.m_numSamples;

    if(dataset_train.m_numSamples < 10){
        qDebug() << "Not enough samples";
        return;
    }


    dataset_train.findFeatRange();

    OnlineRF model(hp, dataset_train.m_numClasses, dataset_train.m_numFeatures, dataset_train.m_minFeatRange, dataset_train.m_maxFeatRange);

    /*
    for(int i = 0; i < dataset_train.m_numFeatures; i++){
        qDebug() << "Min" << dataset_train.m_minFeatRange[i] << "Max" << dataset_train.m_maxFeatRange[i];
    }
    */


    timeIt(1);
    train(&model, dataset_train, hp);
    //trainAndTest(&model, dataset_train, dataset_test, hp);
    cout << "Training/Test time: " << timeIt(0) << endl;

    // fill in datasets

    // Inference here!
    // for all the other points

    Result invalid(8);
    //invalid.confidence.at(0) = 0;
    invalid.prediction = -1;

    std::vector<Result> results(smallcloud->points.size(), invalid);

    for(size_t idx = 0; idx < smallcloud->points.size(); ++idx) {
        Sample sample;
        sample.x = Eigen::VectorXd(dataset_train.m_numFeatures);
        sample.id = idx;
        sample.w = 1.0;


        //set samples
        sample.x(0) = smallcloud->at(idx).x;
        sample.x(1) = smallcloud->at(idx).y;
        sample.x(2) = smallcloud->at(idx).z;
        sample.x(3) = smallcloud->at(idx).intensity;
        sample.x(4) = smallcloud->at(idx).normal_x;
        sample.x(5) = smallcloud->at(idx).normal_y;
        sample.x(6) = smallcloud->at(idx).normal_z;
        sample.x(7) = (*pca)[idx][0];
        sample.x(8) = (*pca)[idx][1];
        sample.x(9) = (*pca)[idx][2];


        model.eval(sample, results[idx]);

        if(idx % 10000 == 0){
            qDebug() << "Label" << results[idx].prediction << "Confidence: " << results[idx].confidence(0) << results[idx].confidence(1);
        }

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
