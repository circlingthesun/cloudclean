#include "plugins/registration/registration.h"
#include <QDebug>
#include <QAction>
#include <QToolBar>
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "commands/select.h"
#include "pluginsystem/core.h"

#include <pcl/search/kdtree.h>

QString Registration::getName(){
    return "Registration";
}

void Registration::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    ll_ = core_->ll_;
    glwidget_ = core_->mw_->glwidget_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;

    myaction = new QAction(QIcon(":/registration.png"), "Registration action", 0);

    connect(myaction,&QAction::triggered, [this] (bool on) {
        qDebug() << "Click!";
    });

    connect(myaction, SIGNAL(triggered()), this, SLOT(myFunc()));

    mw_->toolbar_->addAction(myaction);
}

void Registration::cleanup(){
    mw_->toolbar_->removeAction(myaction);
    delete myaction;
}

Registration::~Registration(){
    qDebug() << "Registration deleted";
}

void Registration::myFunc(){
    qDebug() << "Myfunc";
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.registration")
