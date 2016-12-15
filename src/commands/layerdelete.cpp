#include "layerdelete.h"
#include <QDebug>
#include <model/layerlist.h>
#include <model/layer.h>

LayerDelete::LayerDelete(boost::shared_ptr<Layer> layer, LayerList * ll) {
    const std::set<uint16_t> & source_labels = layer->getLabelSet();
    std::copy(source_labels.begin(), source_labels.end(), std::back_inserter(labels_));
    col_ = layer->getColor();
    name_ = layer->getName();
    ll_ = ll;
    id_ = layer->getId();
}

QString LayerDelete::actionText(){
    return "New Layer";
}

void LayerDelete::undo(){
    // Add layer back
    boost::shared_ptr<Layer> layer = ll_->addLayerWithId(id_);
    layer->setName(name_);
    layer->setColor(col_);
    for(uint16_t label : labels_){
        layer->addLabel(label);
    }

    qDebug() << "Undeleted layer: " << layer->getId() << " that had label" << id_;
}

void LayerDelete::redo(){
    // Delete layer
    boost::shared_ptr<Layer> layer = ll_->getLayer(id_);
    ll_->deleteLayer(layer);

    qDebug() << "Deleted layer: " << id_;
}

bool LayerDelete::mergeWith(const QUndoCommand *other){
    return false;
}

int LayerDelete::id() const{
    return 5;
}
