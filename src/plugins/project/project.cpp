#include "plugins/project/project.h"
#include <QDebug>
#include <QAction>
#include <QFileDialog>
#include <QToolBar>
#include <QStyle>
#include <QApplication>
#include <QSettings>
#include <QString>
#include <fstream>
#include <cstdlib>

#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "commands/select.h"
#include "pluginsystem/core.h"

#include <pcl/search/kdtree.h>

QString Project::getName(){
    return "project";
}

void Project::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    ll_ = core_->ll_;
    glwidget_ = core_->mw_->glwidget_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;

    QStyle * style = QApplication::style();
    save_action_ = new QAction(style->standardIcon(QStyle::SP_DialogSaveButton),"Save project", 0);
    load_action_ = new QAction(style->standardIcon(QStyle::SP_DirIcon), "Load project", 0);



    connect(save_action_,&QAction::triggered, [this] (bool on) {
        qDebug() << "Click!";
    });

    connect(save_action_, SIGNAL(triggered()), this, SLOT(save()));
    connect(load_action_, SIGNAL(triggered()), this, SLOT(load()));


    mw_->addMenu(save_action_, "File");
    mw_->addMenu(load_action_, "File");
    //mw_->toolbar_->addAction(save_action_);
    //mw_->toolbar_->addAction(load_action_);
}

void Project::cleanup(){
    mw_->removeMenu(save_action_, "File");
    mw_->removeMenu(load_action_, "File");
    //mw_->toolbar_->removeAction(save_action_);
    //mw_->toolbar_->removeAction(load_action_);
    delete save_action_;
    delete load_action_;
}

Project::~Project(){
    qDebug() << "project deleted";
}

void Project::save(){
    qDebug() << "Myfunc";

//    cloudcleanproject
//    1 // version
//    1 // hasselections

//    filecount;
//    filename1;
//    filename2;

//    file1_label_count;
//    1 2 3 4 5 5 6 6 9 9 // label for each point
//    1 2 4 8 8 8 9 9 9 9 // selection masks

//    file2_label_count;
//    667 2 26237 4724 27 // label for each point
//    1 2 4 8 8 8 9 9 9 9 // selection masks

//    layer count;
//    layer1 name;
//    layer1 visibility;
//    layer1 color;
//    layer1_label_count;
//    3 324 242 423 42 34234
//    EOF

    QSettings settings;
    QString path = settings.value("load/lastlocation", QDir::home().absolutePath()).toString();

    QString filename = QFileDialog::getSaveFileName(
                 mw_, tr("Save project"), path, tr("Cloud clean project files (*.ccp)"), 0, QFileDialog::DontUseNativeDialog);
    if (filename.length() == 0)
        return;

    QString ccppath = QFileInfo(filename).absolutePath();
    settings.setValue("load/lastlocation", ccppath);
    settings.sync();

    std::ofstream file(filename.toLocal8Bit().data());
    if (!file.is_open()){
        qDebug() << "File open fail";
        return;
    }

    file << "cloudcleanproject\n1\n";
    file << "1\n";

    file << cl_->clouds_.size() << "\n";

    for(boost::shared_ptr<PointCloud> cloud :cl_->clouds_) {
        file << cloud->filepath().toLocal8Bit().data() << "\n";
        qDebug() << "Saving: " << cloud->filepath();
    }

    for(boost::shared_ptr<PointCloud> cloud :cl_->clouds_){
        // save labels
        file << cloud->labels_.size() << "\n";
        int idx = 0;
        for(; idx < cloud->labels_.size()-1; idx++){
            file << cloud->labels_[idx] << " ";
        }
        file << cloud->labels_[++idx] << "\n";

        // save selections
        idx = 0;
        for(; idx < cloud->labels_.size()-1; idx++){
            file << uint8_t(cloud->flags_[idx]) << " ";
        }
        file << uint8_t(cloud->flags_[++idx]) << "\n";

    }

    file << ll_->getLayers().size() << "\n";

    for(const boost::shared_ptr<Layer> layer : ll_->getLayers()) {

        file << layer->getName().toLocal8Bit().data() << "\n";
        file << layer->isVisible() << "\n";
        file << layer->getColor().name().toLocal8Bit().data() << "\n";
        file << layer->getLabelSet().size() << "\n";

        const std::set<uint16_t> & ls = layer->getLabelSet();
        auto it = ls.begin();
        for(; it != --ls.end(); it++) {
            file << *it << " ";
        }
        file << *it << "\n";
    }

    file.close();
}

void Project::load(QString filename){
    QString ccppath = QFileInfo(filename).absolutePath();

    std::ifstream file(filename.toLocal8Bit().data());
    if (!file.is_open()){
        qDebug() << "File open fail";
        return;
    }

    std::string firstline;
    int num_clouds;
    int version = 0;
    int has_selections = false;

    file >> firstline >> std::ws;

    if(firstline == "cloudcleanproject"){
        file >> version >> std::ws;
        file >> has_selections >> std::ws;
        file >> num_clouds >> std::ws;
    } else {
        num_clouds = atoi(firstline.c_str());
    }

    qDebug() << "Number of clouds" << num_clouds;

    std::vector<QString> filepaths(num_clouds);

    char buff[1024];

    for(QString & path : filepaths) {
        file.getline(buff, 1024);
        path = QString::fromLocal8Bit(buff);
        qDebug() << "cloud path" << path;
    }

    std::map<uint16_t, uint16_t> old_to_new_label;

    auto resolveFile = [] (QString filepath, QString hint) {
        filepath = filepath.trimmed();
        bool exists = QFile(filepath).exists();
        QFileInfo fi(filepath);
        if(exists)
            return filepath;

        QChar sep = QDir::separator();
        QString name = fi.fileName();
        QStringList pathlist = hint.split(sep);

        qDebug() << pathlist;

        for(int i = pathlist.size(); i > 0; i--){
            QStringList c;
            for(int j = 0; j < i; j++){
                c << pathlist[j];
            }
            QString candidate_dir = c.join(sep).trimmed();
            c << name;
            QString candidate_path = c.join(sep).trimmed();
            QFileInfo fi(candidate_path);
            exists = QFile(candidate_path).exists();

            qDebug() << "candidate: |" << candidate_path.toLocal8Bit().data() << '|' << exists;

            if(fi.exists())
                return candidate_path;

            // attempt other names
            QStringList sl = QDir(candidate_dir).entryList();
            QString original_name = name.split(".")[0].split("#")[0];
            for(QString str : sl){
                if(str.startsWith(original_name)){
                    return candidate_dir + sep + str;
                }
            }


        }

        return QString("");
    };


    for(QString path : filepaths) {
        QString f = resolveFile(path, ccppath);
        qDebug() << "loading: " << f;
        if(f == "")
            return;
        boost::shared_ptr<PointCloud> cloud = cl_->loadFile(f);
        if(cloud == nullptr) {
            qDebug() << "could not load cloud: " << f;
            return;
        }
        std::vector<uint16_t> & labels = cloud->labels_;
        int labelcount;
        int label;
        file >> labelcount >> std::ws;
        qDebug() << "Label count: " << labelcount;
        for(int i = 0; i < labelcount; i++) {
            file >> label;
            if(old_to_new_label.find(label) == old_to_new_label.end())
                old_to_new_label[label] = ll_->createLabelId();
            labels[i] = old_to_new_label[label];
        }

        if(version == 0)
            continue;

        // save selections
        uint8_t flags;
        for(int i = 0; i < labelcount; i++) {
            file >> flags;
            cloud->flags_[i] = PointFlags(flags);
        }

    }

    int layercount;
    file >> layercount >> std::ws;

    for(int i = 0; i < layercount; i++) {

        file.getline(buff, 1024);
        QString name(buff);
        boost::shared_ptr<Layer> layer = ll_->addLayer(name);

        bool visible;
        file >> visible;
        if(!visible)
            layer->toggleVisible();

        if(version > 0){
            std::string color_str;
            file >> color_str;
            layer->setColor(QColor(QString(color_str.c_str())));
        }


        int label_count;
        file >> label_count;

        uint16_t label;
        for(int j = 0; j < label_count; j++){
            file >> label >> std::ws;
            layer->addLabel(old_to_new_label[label]);
        }

        emit ll_->layerUpdate(layer);
    }

    file.close();

    emit ll_->lookupTableUpdate();
}

void Project::load(){
    QSettings settings;

    QString path = settings.value("load/lastlocation", QDir::home().absolutePath()).toString();

    QString filename = QFileDialog::getOpenFileName(
                 mw_, tr("Open project"), path, tr("Cloud clean project files (*.ccp)"), 0, QFileDialog::DontUseNativeDialog);
    if (filename.length() == 0)
        return;

    // look up root of where the ccp file is
    QString ccppath = QFileInfo(filename).absolutePath();

    settings.setValue("load/lastlocation", ccppath);
    settings.sync();

    load(filename);


    //std::thread(&CloudList::loadFile, cl_, filename).detach();
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.project")
