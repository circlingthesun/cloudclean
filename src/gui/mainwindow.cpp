#include "gui/mainwindow.h"

#include <QProgressBar>
#include <QStatusBar>
#include <QAction>
#include <QUndoStack>
#include <QMenu>
#include <QMenuBar>
#include <QFileDialog>
#include <QStackedWidget>
#include <QToolBox>
#include <QToolButton>
#include <QToolBar>
#include <QStyle>
#include <QApplication>
#include <QGridLayout>
#include <QBoxLayout>
#include <QSettings>
#include <QShortcut>
#include <QButtonGroup>
#include <QLabel>
#include <QGridLayout>
#include <QPushButton>
#include <QVBoxLayout>

#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/gldata.h"
#include "gui/cloudlistview.h"
#include "gui/layerlistview.h"
#include "model/cloudlist.h"
#include "model/layerlist.h"

MainWindow::MainWindow(QUndoStack *us, CloudList * cl, LayerList * ll, QWidget *parent)
    : QMainWindow(parent) {

    this->setObjectName("mainwindow");

    select_mask_ = 1;
    deselect_ = false;

    us_ = us;
    ll_ = ll;
    cl_ = cl;

    progressbar_ = new QProgressBar(this);
    statusbar_ = statusBar();
    tabs_ = new QTabWidget(this);

    QGLFormat base_format;
    // Core profile breaks qpainter
    base_format.setProfile(QGLFormat::CoreProfile);
    // This can be set to 3.3 but setting the version gets us core profile on amd
#ifndef AMD_GRAPHICS
    base_format.setVersion(3, 3);
#endif

    base_format.setSampleBuffers(true);

    //QGLFormat::setDefaultFormat(base_format);

    // Important! Context invalidates when reparenting the glwidget on windows
    glwidget_ = new GLWidget(base_format, cl, ll, this);
    tabs_->addTab(glwidget_, "3D View");
    flatview_ = new FlatView(base_format, cl, ll, this, glwidget_);
    tabs_->addTab(flatview_, "2D View");

    qDebug() << "Format: " << glwidget_->format().majorVersion() << glwidget_->format().minorVersion() << "Core:" << (glwidget_->format().profile() == QGLFormat::CoreProfile);

    QGLContext * ctx = const_cast<QGLContext *>(glwidget_->context());
    gld_ = new GLData(ctx, cl, ll);
    glwidget_->setGLD(gld_);
    flatview_->setGLD(gld_);

    clv_ = new CloudListView(us, ll, cl, glwidget_, this);
    llv_ = new LayerListView(us, ll, cl, select_mask_, this);

    progressbar_->setTextVisible( false );
    progressbar_->setRange( 0, 100 );

    statusbar_->addPermanentWidget( progressbar_, 0 );
    statusbar_->showMessage( tr("Ready") );


    QStyle * style = QApplication::style();
    QAction * undo = us_->createUndoAction(0);
    undo->setIcon(style->standardIcon(QStyle::SP_FileDialogBack));
    QAction * redo = us_->createRedoAction(0);
    redo->setIcon(style->standardIcon(QStyle::SP_ArrowRight));

    options_dock_ = new QDockWidget(this);
    options_dock_->setObjectName("options_dock");
    options_dock_->setWindowTitle(tr("Tool options"));
    tooloptions_ = new QStackedWidget(options_dock_);
    options_dock_->setWidget(tooloptions_);

    toolbar_ = new QToolBar(this);
    toolbar_->setObjectName("toolbar");
    addToolBar(Qt::LeftToolBarArea, toolbar_);
    toolbar_->setToolButtonStyle(Qt::ToolButtonIconOnly);


    addDockWidget(Qt::RightDockWidgetArea, clv_);
    addDockWidget(Qt::RightDockWidgetArea, llv_);
    tabifyDockWidget(clv_, llv_);
    clv_->raise();

    addDockWidget(Qt::RightDockWidgetArea, options_dock_);
    options_dock_->hide();

    setCentralWidget(tabs_);
    setVisible(true);

    mb_ = menuBar();

    file_menu_ = new QMenu(tr("File"), this);
    edit_menu_ = new QMenu(tr("Edit"), this);
    view_menu_ = new QMenu(tr("View"), this);
    window_menu_ = new QMenu(tr("Window"), this);

    mb_->addMenu(file_menu_);
    mb_->addMenu(edit_menu_);
    mb_->addMenu(view_menu_);
    mb_->addMenu(window_menu_);

    menus_.insert(tr("File"), file_menu_);
    menus_.insert(tr("Edit"), edit_menu_);
    menus_.insert(tr("View"), view_menu_);
    menus_.insert(tr("Window"), window_menu_);

    QAction * load = new QAction(tr("Load PTX"), this);
    load->setIcon(style->standardIcon(QStyle::SP_DirIcon));
    //QAction * save = new QAction(tr("Save"), this);
    //save->setIcon(style->standardIcon(QStyle::SP_DialogSaveButton));
    connect(load, SIGNAL(triggered()), this, SLOT(loadFile()));
    //connect(save, SIGNAL(triggered()), this, SLOT(saveLayer()));

    //toolbar_->addAction(load);
    //toolbar_->addAction(save);
    file_menu_->addAction(load);
    //file_menu_->addAction(save);

    QAction * reset = new QAction(tr("Reset"), this);
    connect(reset, &QAction::triggered, [this](){
        ll_->reset();
        gld_->reloadColorLookupBuffer();
        for(boost::shared_ptr<PointCloud> cloud : cl_->clouds_) {
            gld_->deleteCloud(cloud);
        }
        cl_->reset();
    });

    file_menu_->addAction(reset);

    QAction * exitAct = new QAction(tr("E&xit"), this);
    exitAct->setShortcuts(QKeySequence::Quit);
    exitAct->setStatusTip(tr("Exit the application"));
    exitAct->setIcon(QIcon(style->standardIcon(QStyle::SP_DialogCloseButton)));
    connect(exitAct, SIGNAL(triggered()), qApp, SLOT(closeAllWindows()));
    file_menu_->addAction(exitAct);


    toolbar_->addAction(undo);
    toolbar_->addAction(redo);

    window_menu_->addAction(clv_->toggleViewAction());
    window_menu_->addAction(llv_->toggleViewAction());
    window_menu_->addAction(options_dock_->toggleViewAction());

    QAction * roll_correction_action = new QAction(tr("Toggle roll correction"), this);
    roll_correction_action->setCheckable(true);
    roll_correction_action->setChecked(true);

    view_menu_->addAction(roll_correction_action);

    // SIGNALS
    qRegisterMetaType<boost::shared_ptr<PointCloud> >("boost::shared_ptr<PointCloud>");
    qRegisterMetaType<boost::shared_ptr<Layer> >("boost::shared_ptr<Layer>");

    connect(roll_correction_action, &QAction::toggled, [this](bool on){
        if(on){
            statusbar_->showMessage("Roll correction on", 5000);
        } else {
            statusbar_->showMessage("Roll correction off", 5000);
        }
        glwidget_->camera_.toggleRollCorrection(on);
    });

    connect(gld_, &GLData::update, glwidget_, (void (GLWidget:: *)(void)) &GLWidget::update);
    connect(gld_, &GLData::update, flatview_, (void (FlatView:: *)(void)) &FlatView::update);
    connect(cl, &CloudList::cloudUpdate, gld_, &GLData::reloadCloud);
    connect(cl, &CloudList::updated, glwidget_, (void (GLWidget:: *)(void)) &GLWidget::update);
    connect(cl, &CloudList::updatedActive, flatview_, &FlatView::setCloud);
    connect(cl, &CloudList::progressUpdate, progressbar_, &QProgressBar::setValue);
    connect(ll, &LayerList::layerUpdate, gld_, &GLData::reloadColorLookupBuffer);
    connect(ll, &LayerList::lookupTableUpdate, gld_, &GLData::reloadColorLookupBuffer);

    QAction * deselect = new QAction(tr("Deselect all"), this);
    connect(deselect, SIGNAL(triggered()), clv_, SLOT(deselectAllPoints()));
    edit_menu_->addAction(deselect);

    QAction * select = new QAction(tr("Select all"), this);
    connect(select, SIGNAL(triggered()), clv_, SLOT(selectAllPoints()));
    edit_menu_->addAction(select);

    QAction * invert = new QAction(tr("Invert Selection"), this);
    connect(invert, SIGNAL(triggered()), clv_, SLOT(invertSelection()));
    edit_menu_->addAction(invert);

    undo->setShortcut(QKeySequence::Undo);
    redo->setShortcut(QKeySequence::Redo);
    undo->setShortcutContext(Qt::ApplicationShortcut);
    redo->setShortcutContext(Qt::ApplicationShortcut);

    edit_menu_->addAction(undo);
    edit_menu_->addAction(redo);

    gld_->reloadColorLookupBuffer();


    connect(glwidget_, SIGNAL(customContextMenuRequested(const QPoint&)),
            this, SLOT(contextMenu(const QPoint &)));

    connect(flatview_, SIGNAL(customContextMenuRequested(const QPoint&)),
            this, SLOT(contextMenu(const QPoint &)));

    // setup selection color picker
    QColor colors[] = {
        QColor(255, 0, 0, 255), // Red
        QColor(0, 255, 0, 255), // Green
        QColor(0, 0, 255, 255), // Blue
        QColor(255, 255, 0, 255), // Yellow
        QColor(0, 255, 255, 255), // Cyan
        QColor(255, 0, 255, 255), // Magenta
        QColor(255, 128, 0, 255), // Orange
        QColor(128, 0, 255, 255) // Purple
    };

    selection_dock_ = new QDockWidget(this);
    selection_dock_->setObjectName("options_dock");
    selection_dock_->setWindowTitle(tr("Selection options"));
    QWidget * selection_options_widget = new QWidget(this);
    selection_dock_->setWidget(selection_options_widget);

    window_menu_->addAction(selection_dock_->toggleViewAction());

    button_group_ = new QButtonGroup(selection_options_widget);
    button_group_->setExclusive(true);

    QVBoxLayout * selection_options_layout = new QVBoxLayout(selection_options_widget);
    selection_options_widget->setLayout(selection_options_layout);


    QGridLayout * color_layout = new QGridLayout();
    color_layout->setColumnStretch(0, 1);
    color_layout->setColumnStretch(1, 1);
    color_layout->setColumnStretch(2, 1);
    color_layout->setColumnStretch(3, 1);
    color_layout->setColumnStretch(4, 1);
    color_layout->setColumnStretch(5, 1);
    color_layout->setColumnStretch(6, 1);
    color_layout->setColumnStretch(7, 1);

    selection_options_layout->addLayout(color_layout, 1);

    for(int i = 0; i < 8; i++){
        QPushButton * btn = new QPushButton();
        QColor inverted(255-colors[i].red(), 255-colors[i].green(), 255-colors[i].blue());
        QString qss = QString("QPushButton {background-color: %1; color: %2; padding: 0.25em;} QPushButton:checked {background-color: %3}").arg(colors[i].name()).arg(inverted.name()).arg(colors[i].name());
        btn->setStyleSheet(qss);
        btn->setCheckable(true);
        button_group_->addButton(btn);
        buttons_.push_back(btn);
        color_layout->addWidget(btn, 0, i);
        btn->setText(QString("%1").arg(i));
        btn->connect(btn, &QPushButton::pressed, [this, i] (){
            setSelectMask(1 << i);
        });
    }

    QPushButton * deselect_button = new QPushButton("Deselect mode", selection_options_widget);
    deselect_button->setCheckable(true);
    connect(deselect_button, &QPushButton::toggled, [this] (bool on){
        deselect_ = on;
    });
    selection_options_layout->addWidget(deselect_button, 1);

    setSelectMask(select_mask_);

    addDockWidget(Qt::RightDockWidgetArea, selection_dock_);

    edit_mode_ = true;

    // setup keyboard shortcuts

    connect(new QShortcut(QKeySequence(Qt::Key_Z), this), &QShortcut::activated, [this] (){
        edit_mode_ = !edit_mode_;
        toolbar_->setDisabled(!edit_mode_);
    });
    connect(new QShortcut(QKeySequence(Qt::Key_1), this), &QShortcut::activated, [this] (){
        setSelectMask(1);
    });
    connect(new QShortcut(QKeySequence(Qt::Key_2), this), &QShortcut::activated, [this] (){
        setSelectMask(2);
    });
    connect(new QShortcut(QKeySequence(Qt::Key_3), this), &QShortcut::activated, [this] (){
        setSelectMask(4);
    });
    connect(new QShortcut(QKeySequence(Qt::Key_4), this), &QShortcut::activated, [this] (){
        setSelectMask(8);
    });
    connect(new QShortcut(QKeySequence(Qt::Key_5), this), &QShortcut::activated, [this] (){
        setSelectMask(16);
    });
    connect(new QShortcut(QKeySequence(Qt::Key_6), this), &QShortcut::activated, [this] (){
        setSelectMask(32);
    });
    connect(new QShortcut(QKeySequence(Qt::Key_7), this), &QShortcut::activated, [this] (){
        setSelectMask(64);
    });
    connect(new QShortcut(QKeySequence(Qt::Key_8), this), &QShortcut::activated, [this] (){
        setSelectMask(128);
    });

    // Restore state
    QSettings settings;
    if(!restoreGeometry(settings.value("mainwindow/geometry").toByteArray())){
        setGeometry(50, 50, 1280, 700);
    }

    restoreState(settings.value("mainwindow/windowState").toByteArray());

#ifdef LEAP_SUPPORT
    listener = new LeapListener(glwidget_);
    controller = new Leap::Controller();
    controller->addListener(*listener);
#endif

}

MainWindow::~MainWindow() {
    delete gld_;
}


void MainWindow::setSelectMask(uint8_t mask){
    int pos = 0;
    for(; pos < 8; ++pos){
        if(1<<pos == mask)
            break;
    }
    buttons_[pos]->setChecked(true);
    select_mask_ = mask;
}

void MainWindow::addMenu(QAction * action, QString menu_name){
    auto it = menus_.find(menu_name);
    QMenu * menu;
    if(it != menus_.end()){
        menu = *it;
    }
    else {
        menu = new QMenu(menu_name, mb_);
        mb_->addMenu(menu);
        menus_.insert(menu_name, menu);
    }
    menu->addAction(action);
}

void MainWindow::removeMenu(QAction * action, QString menu_name){
    auto it = menus_.find(menu_name);
    if(it != menus_.end()){
        QMenu * menu = *it;
        menu->removeAction(action);
    }
}

void MainWindow::loadFile(){
    QSettings settings;

    QString path = settings.value("load/lastlocation", QDir::home().absolutePath()).toString();
    QString filename = QFileDialog::getOpenFileName(
                 this, tr("Open Scan"), path , tr("PTX Files (*.ptx)"), 0, QFileDialog::DontUseNativeDialog);
    if (filename.length() == 0)
        return;

    settings.setValue("load/lastlocation", QFileInfo(filename).absolutePath());
    settings.sync();
    std::thread(&CloudList::loadFile, cl_, filename).detach();
}

void MainWindow::saveLayer(){
    QString filename = QFileDialog::getSaveFileName(
                this, tr("Save layer as PTX"), QDir::home().absolutePath(), tr("PTX Files (*.ptx)"), 0, QFileDialog::DontUseNativeDialog);
    if (filename.length() == 0)
        return;

    std::set<uint16_t> slabels;
    for(boost::weak_ptr<Layer> wl : ll_->getSelection()) {
        boost::shared_ptr<Layer> l = wl.lock();
        for(uint16_t label : l->getLabelSet()){
            slabels.insert(label);
        }
    }

    std::vector<uint16_t> labels;
    for(uint16_t label : slabels){
        labels.push_back(label);
    }

    std::thread(&CloudList::saveFile, cl_, filename, labels).detach();
}

void MainWindow::startBgAction(QString name, bool deterministic) {
    if(!deterministic) {
        progressbar_->setRange(0,0);
    }
    progressbar_->setValue(0);
    statusbar_->showMessage(name);
}

void MainWindow::stopBgAction(QString name) {
    progressbar_->setRange(0,100);
    progressbar_->setValue(0);
    statusbar_->showMessage("Completed:" + name, 2000);
}

void MainWindow::progBgAction(QString name, int prog) {
    progressbar_->setValue(prog);
}

void MainWindow::contextMenu(const QPoint &pos) {
    QMenu menu;

    menu.addAction("Layer from selection", llv_,
                   SLOT(selectionToLayer()));

    menu.addAction("Deselect all", clv_, SLOT(deselectAllPoints()));
    menu.addAction("Select all", clv_, SLOT(selectAllPoints()));
    menu.addAction("Invert selection", clv_, SLOT(invertSelection()));


    if(flatview_->isVisible())
        menu.addAction("Rotate", flatview_, SLOT(rotate90()));

    menu.addAction("Bird's eye", &glwidget_->camera_, SLOT(birds_eye()));

    menu.exec(glwidget_->mapToGlobal(pos));
}

void MainWindow::closeEvent(QCloseEvent *event) {
    QSettings settings;
    settings.setValue("mainwindow/geometry", saveGeometry());
    settings.setValue("mainwindow/windowState", saveState());
}
