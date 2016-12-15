#include "layerguiselect.h"
#include <QDebug>
#include <model/layerlist.h>
#include <model/pointcloud.h>
#include <model/layer.h>

LayerFromSelection::LayerFromSelection(boost::shared_ptr<std::vector<uint16_t> > labels,
                                 LayerList * ll, bool subtractive) {
    subtractive_ = subtractive;
    labels_ = labels;
    ll_ = ll;
    new_layer_id_ = -1;
    applied_once_ = false;
}

QString LayerFromSelection::actionText(){
    return "New Layer";
}

void LayerFromSelection::undo(){

    boost::shared_ptr<Layer> layer = ll_->getLayer(new_layer_id_);

    // Delete layer
    if(layer == nullptr){
        qDebug() << "This should not happen";
        return;
    }

    ll_->deleteLayer(layer);

    // TODO: Freelist is not updated

    // Undo subtractive
    if(subtractive_) {
        for(auto it : removed_from_){
            for(uint16_t label : it.second)
                it.first->addLabel(label);
        }
    }

}

void LayerFromSelection::redo(){
    boost::shared_ptr<Layer> layer;
    if(applied_once_)
        layer = ll_->addLayerWithId(new_layer_id_);
    else
        layer = ll_->addLayer();

    new_layer_id_ = layer->getId();

    if(subtractive_){
        // Subtractive remove labels from layers
        for(uint16_t label : *labels_) {
            const LayerSet & ls = ll_->getLayersForLabel(label);
            for(Layer * l : ls){
                if(l != ll_->getDefaultLayer().get()){
                    removed_from_[l].push_back(label);
                    l->removeLabel(label);
                }
            }
        }

    }

    for(uint16_t label : *labels_){
        layer->addLabel(label);
    }

    applied_once_ = true;
}

bool LayerFromSelection::mergeWith(const QUndoCommand *other){
    return false;
}

int LayerFromSelection::id() const{
    return 3;
}
