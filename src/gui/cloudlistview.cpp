#include "cloudlistview.h"
#include "ui_cloudlistview.h"
#include <QMenu>
#include <QDebug>
#include <QFileDialog>
#include <QUndoStack>
#include <boost/make_shared.hpp>
#include <thread>
#include <functional>
#include "commands/select.h"

CloudListView::CloudListView(QUndoStack *us, LayerList * ll,
                             CloudList * cl, GLWidget * glwidget, QWidget *parent)
    : QDockWidget(parent),
    ui_(new Ui::CloudListView) {
    ll_ = ll;
    cl_ = cl;
    us_ = us;
    glwidget_ = glwidget;
    ui_->setupUi(this);
    ui_->tableView->setModel(cl_);
    ui_->tableView->setSelectionMode(QAbstractItemView::SingleSelection);

    mw_ = static_cast<MainWindow *>(parent);

    connect(ui_->tableView->selectionModel(),
            SIGNAL(selectionChanged(const QItemSelection &,
                                    const QItemSelection &)),
            cl,
            SLOT(selectionChanged(const QItemSelection &,
                                    const QItemSelection &)));

    connect(ui_->tableView, SIGNAL(customContextMenuRequested(const QPoint&)),
            this, SLOT(contextMenu(const QPoint &)));

    connect(cl_, SIGNAL(cloudUpdate(boost::shared_ptr<PointCloud>)), this, SLOT(dataChanged()));

    ui_->tableView->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui_->tableView->setContextMenuPolicy(Qt::CustomContextMenu);
    ui_->tableView->setColumnWidth(0, 30);
    ui_->tableView->horizontalHeader()->setStretchLastSection(true);
    ui_->tableView->setStyleSheet(
        "QTableView::indicator:unchecked {image: url(:/eye_closed.png);}"
        "QTableView::indicator:checked {image: url(:/eye_open.png);}"
    );
}

CloudListView::~CloudListView() {
    delete ui_;
}

void CloudListView::dataChanged() {
    bool selected = ui_->tableView->selectionModel()->selectedRows().size() != 0;
    if(!selected && cl_->clouds_.size() != 0)
        ui_->tableView->selectRow(0);
}

void CloudListView::contextMenu(const QPoint &pos) {
    QMenu menu;
    QModelIndex cell = ui_->tableView->indexAt(pos);

    if(cell.isValid()){
        int row = cell.row();
        PointCloud * pc = cl_->clouds_[row].get();

        menu.addAction("Reset orientation", pc,
                        SLOT(resetOrientation()));

        QAction del("Delete", 0);
        del.setProperty("cloud_id", row);
        connect(&del, SIGNAL(triggered()), cl_, SLOT(removeCloud()));
        menu.addAction(&del);

        QAction fly("Fly to cloud", 0);
        fly.setProperty("cloud_id", row);
        connect(&fly, &QAction::triggered, [=] () {
            auto pos = cl_->clouds_[row]->sensor_origin_;
            glwidget_->camera_.setPosition(-pos.x(), -pos.y(), -pos.z());
            //qDebug() << "Flying to: " << -pos.x() << -pos.y() << -pos.z();
            glwidget_->update();
        });
        menu.addAction(&fly);


        QAction tfc("Change coordinate frame", 0);
        tfc.setProperty("cloud_id", row);
        connect(&tfc, &QAction::triggered, [=] () {
            if(cl_->clouds_[row]->frame_ == CoordinateFrame::Camera)
                cl_->clouds_[row]->frame_ = CoordinateFrame::Laser;
            else
                cl_->clouds_[row]->frame_ =CoordinateFrame::Camera;

            pc->resetOrientation();
            glwidget_->update();
        });
        menu.addAction(&tfc);


        menu.exec(ui_->tableView->mapToGlobal(pos));
    }
}

void CloudListView::deselectAllPoints(){
    us_->beginMacro("Deselect All");
    for(boost::shared_ptr<PointCloud> cloud : cl_->clouds_){

        std::vector<boost::shared_ptr<std::vector<int> > > selections = cloud->getSelections();

        for(size_t sel_idx = 0; sel_idx < selections.size(); sel_idx++){
            boost::shared_ptr<std::vector<int> > selection = selections[sel_idx];
            us_->push(new Select(cloud, selection, true, 1 << sel_idx, true, ll_->getHiddenLabels()));
        }
    }
    us_->endMacro();
}

void CloudListView::selectAllPoints(){
    us_->beginMacro("Select All");
    for(boost::shared_ptr<PointCloud> cloud : cl_->clouds_){
        boost::shared_ptr<std::vector<int> > indices;
        indices.reset(new std::vector<int>());

        for(uint idx = 0; idx < cloud->flags_.size(); idx++){
            PointFlags & flag =  cloud->flags_[idx];
            if(!(uint8_t(PointFlags::selected) & uint8_t(flag)))
                indices->push_back(idx);
        }

        us_->push(new Select(cloud, indices, false, mw_->select_mask_, true, ll_->getHiddenLabels()));
    }
    us_->endMacro();
}

void CloudListView::invertSelection(){
    us_->beginMacro("Invert selection");
    for(boost::shared_ptr<PointCloud> cloud : cl_->clouds_){
        boost::shared_ptr<std::vector<int> > select = boost::make_shared<std::vector<int> >();
        boost::shared_ptr<std::vector<int> > deselect = boost::make_shared<std::vector<int> >();

        for(uint idx = 0; idx < cloud->flags_.size(); idx++){
            PointFlags & flag = cloud->flags_[idx];
            if(uint8_t(flag) == 0)
                select->push_back(idx);
            else
                deselect->push_back(idx);
        }

        us_->push(new Select(cloud, select, false, mw_->select_mask_, true, ll_->getHiddenLabels()));
        us_->push(new Select(cloud, deselect, true, mw_->select_mask_, true, ll_->getHiddenLabels()));
    }
    us_->endMacro();
}
