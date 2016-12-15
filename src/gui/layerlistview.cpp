#include "layerlistview.h"
#include "ui_layerlistview.h"
#include <QDebug>
#include <QColorDialog>
#include <QMenu>
#include <QFileDialog>
#include <QDir>
#include <boost/make_shared.hpp>

#include "commands/newlayer.h"
#include "commands/select.h"
#include "commands/layerfromlabels.h"
#include "commands/layerdelete.h"

LayerListView::LayerListView(QUndoStack * us, LayerList * ll,
                             CloudList * cl, uint8_t & selection_mask, QWidget *parent) :
    QDockWidget(parent), ui_(new Ui::LayerListView), selection_mask_(selection_mask) {
    ll_ = ll;
    cl_ = cl;
    us_ = us;
    ui_->setupUi(this);
    ui_->tableView->setModel(ll_);


//    connect(ui_->tableView->selectionModel(),
//            SIGNAL(selectionChanged(const QItemSelection &,
//                                 const QItemSelection &)), ll_,
//            SLOT(selectionChanged(const QItemSelection &,
//                                    const QItemSelection &)));

    connect(ui_->tableView->selectionModel(), &QItemSelectionModel::selectionChanged, [=] (const QItemSelection & sel, const QItemSelection & dsel) {
        std::vector<int> selection;
        for(QModelIndex index : ui_->tableView->selectionModel()->selectedIndexes()){
            if(index.column() != 0)
                continue;
            int idx = index.row();
            selection.push_back(idx);
        }
        ll_->selectionChanged(selection);
    });

    ui_->tableView->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(ui_->tableView, SIGNAL(customContextMenuRequested(const QPoint&)),
            this, SLOT(contextMenu(const QPoint &)));
    ui_->tableView->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui_->tableView->setColumnWidth(0, 30);
    ui_->tableView->horizontalHeader()->setStretchLastSection(true);
    ui_->tableView->setStyleSheet(
        "QTableView::indicator:unchecked {image: url(:/eye_closed.png);}"
        "QTableView::indicator:checked {image: url(:/eye_open.png);}"
    );

    ui_->tableView->setEditTriggers(QAbstractItemView::SelectedClicked | QAbstractItemView::DoubleClicked);
}

LayerListView::~LayerListView() {
    delete ui_;
}

void LayerListView::selectLayer(boost::shared_ptr<Layer> layer) {

}

// TODO(Rickert): selections should also be updated via a vector


// This function is here because i do not want to put the layerlist in the
// cloudlist
void LayerListView::selectionToLayer(){
    us_->beginMacro("Layer from selection");
    // Remap all selected points
    for(boost::shared_ptr<PointCloud> & pc : cl_->clouds_){
        std::vector<boost::shared_ptr<std::vector<int> >> selections = pc->getSelections();

        for(size_t sel_idx = 0; sel_idx < selections.size(); sel_idx++){
            boost::shared_ptr<std::vector<int> > selection = selections[sel_idx];
            if(selection->size() == 0)
                continue;

            us_->push(new NewLayer(pc, selection, ll_));
            us_->push(new Select(pc,selection, true, 1 << sel_idx, true, ll_->getHiddenLabels()));
        }
    }

    us_->endMacro();
    emit update();
    ui_->tableView->selectRow(ll_->getLayers().size()-1);
}

void LayerListView::intersectSelectedLayers(bool subtractive){
    if(ll_->getSelection().size() < 2){
        qDebug() << "Select at least 2 layers";
        return;
    }


    // First selected label
    std::set<uint16_t> & labels = ll_->getSelection()[0].lock()->labels_;
    uint intersection_count = 0;

    // Find intersecting labels
    boost::shared_ptr<std::vector<uint16_t> > intersecting_labels;
    intersecting_labels.reset(new std::vector<uint16_t>());

    // For every label index in the first layer
    for(uint8_t label : labels){
        intersection_count = 0;
        // For every other selected layer
        for(uint i = 1; i < ll_->getSelection().size(); i++){
            boost::shared_ptr<Layer> layer = ll_->getSelection()[i].lock();
            // For every label in the other layer
            for(uint8_t qlabel : layer->labels_){
                if(label == qlabel){
                    intersection_count++;
                    break;
                }
            }
        }
        if(intersection_count == ll_->getSelection().size()-1) {
            intersecting_labels->push_back(label);
        }
    }


    if(intersecting_labels->size() == 0){
        qDebug() << "no intersection";
        return;
    }

    us_->beginMacro("Layer intersection");
    us_->push(new LayerFromLabels(intersecting_labels, ll_, "Intersection", subtractive));
    us_->endMacro();

    ui_->tableView->selectRow(ll_->getLayers().size()-1);

}

void LayerListView::intersectSelectedLayersSubtract(){
    intersectSelectedLayers(true);
}

void LayerListView::mergeSelectedLayers(bool copy) {
    // Mark for deletion

    std::vector<boost::weak_ptr<Layer>> selections = ll_->getSelection();

    boost::shared_ptr<std::vector<uint16_t> > labels;
    labels.reset(new std::vector<uint16_t>());

    for(boost::weak_ptr<Layer> l: selections){
        boost::shared_ptr<Layer> layer = l.lock();
        for(int label : layer->labels_){
            labels->push_back(label);
        }
    }

    // Get rid of duplicates
    std::sort(labels->begin(), labels->end());
    labels->erase( unique(labels->begin(), labels->end()), labels->end());

    us_->beginMacro("Merge layers");
    us_->push(new LayerFromLabels(labels, ll_, "Merged layer", !copy));

    if(!copy){
        // Delete marked layers
        for(boost::weak_ptr<Layer> sel : selections) {
            if(sel.expired()){
                qDebug() << "Damn selected layer expired";
                continue;
            }

            us_->push(new LayerDelete(sel.lock(), ll_));
        }
    }

    us_->endMacro();
    ui_->tableView->selectRow(ll_->getLayers().size()-1);
}

void LayerListView::mergeSelectedLayersCopy(){
    mergeSelectedLayers(true);
}

void LayerListView::contextMenu(const QPoint &pos) {
    QMenu menu;
    QModelIndex cell = ui_->tableView->indexAt(pos);

    if(cell.isValid()){
        int row = cell.row();

        QAction randCol("Random color", 0);
        randCol.setProperty("layer_id", row);
        randCol.setProperty("random", true);
        connect(&randCol, SIGNAL(triggered()), this, SLOT(setColor()));
        menu.addAction(&randCol);

        QAction changeCol("Change color", 0);
        changeCol.setProperty("layer_id", row);
        changeCol.setProperty("random", false);
        connect(&changeCol, SIGNAL(triggered()), this, SLOT(setColor()));
        menu.addAction(&changeCol);

        QAction del("Delete", 0);
        del.setProperty("layer_id", row);
        connect(&del, &QAction::triggered, [&] (bool triggered) {
            us_->beginMacro("Delete Layer(s)");

            std::vector<boost::weak_ptr<Layer>> selections = ll_->getSelection();

            for(boost::weak_ptr<Layer> wp : selections) {
                if(wp.expired())
                    continue;
                boost::shared_ptr<Layer> l = wp.lock();
                us_->push(new LayerDelete(l, ll_));
            }
            us_->endMacro();
        });

        menu.addAction(&del);

        QAction merge("Merge", 0);
        connect(&merge, SIGNAL(triggered()), this,
                SLOT(mergeSelectedLayers()));
        menu.addAction(&merge);

        QAction merge_copy("Merge & Copy", 0);
        connect(&merge_copy, SIGNAL(triggered()), this,
                SLOT(mergeSelectedLayersCopy()));
        menu.addAction(&merge_copy);

        QAction inter("Intersect", 0);
        connect(&inter, SIGNAL(triggered()), this,
                SLOT(intersectSelectedLayers()));
        menu.addAction(&inter);

        QAction intersub("Intersect (Subtract)", 0);
        connect(&intersub, SIGNAL(triggered()), this,
                SLOT(intersectSelectedLayersSubtract()));
        menu.addAction(&intersub);

        QAction select_layer("Select points", 0);
        connect(&select_layer, &QAction::triggered, [=] () {

            std::set<uint16_t> selected_labels;
            for(boost::weak_ptr<Layer> wl : ll_->getSelection()) {
                boost::shared_ptr<Layer> l = wl.lock();
                for(uint16_t label  : l->getLabelSet()) {
                    selected_labels.insert(label);
                }
            }

            us_->beginMacro("Select layer");
            for(boost::shared_ptr<PointCloud> pc : cl_->clouds_){
                boost::shared_ptr<std::vector<int>> points = boost::make_shared<std::vector<int>>();
                for(uint idx = 0; idx < pc->size(); ++idx) {
                    for(uint16_t slabel : selected_labels) {
                        if(slabel == pc->labels_[idx]){
                            points->push_back(idx);
                            break;
                        }
                    }
                }

                us_->push(new Select(pc, points, false, selection_mask_, true, ll_->getHiddenLabels()));

            }
            us_->endMacro();
        });
        menu.addAction(&select_layer);

        QAction save_layer("Save Layer(s)", 0);

        connect(&save_layer, &QAction::triggered, [&] () {
                QString filename = QFileDialog::getSaveFileName(
                            nullptr, tr("Save layer as PTX"), QDir::home().absolutePath(), tr("PTX Files (*.ptx)"), 0, QFileDialog::DontUseNativeDialog);
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
        });

        menu.addAction(&save_layer);

        menu.exec(ui_->tableView->mapToGlobal(pos));
    }
}

void LayerListView::setColor() {
    int id = sender()->property("layer_id").toInt();
    bool random = sender()->property("random").toBool();
    Layer & l = *ll_->getLayers()[id];
    if(random)
        l.setRandomColor();
    else {
        QColor new_col = QColorDialog::getColor(l.color_);
        if(new_col.isValid())
            l.setColor(new_col);
    }
    emit update();
}
