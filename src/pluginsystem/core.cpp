#include "pluginsystem/core.h"
#include <QAction>
#include <QUndoStack>

#include "glheaders.h"
#include "gui/mainwindow.h"
#include <model/cloudlist.h>
#include <model/layerlist.h>

Core::Core() {
    us_ = new QUndoStack();
    ll_ = new LayerList();
    cl_ = new CloudList();
    mw_ = new MainWindow(us_, cl_, ll_);
}

Core::~Core() {
    delete mw_;
    delete cl_;
    delete ll_;
    delete us_;
}
