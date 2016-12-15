#include "layerfromlabels.h"
#include <QDebug>
#include <model/layerlist.h>
#include <model/pointcloud.h>
#include <model/layer.h>

LayerFromLabels::LayerFromLabels(boost::shared_ptr<std::vector<uint16_t> > labels,
                                 LayerList * ll, QString layer_name, bool subtractive) {
    subtractive_ = subtractive;
    labels_ = labels;
    ll_ = ll;
    layer_name_ = layer_name;
    new_layer_id_ = -1;
    applied_once_ = false;
}

QString LayerFromLabels::actionText(){
    return "New Layer";
}

void LayerFromLabels::undo(){

    // Delete layer
    boost::shared_ptr<Layer> new_layer = ll_->getLayer(new_layer_id_);
    if(new_layer == nullptr){
        qDebug() << "Layer went missing, can't undo new layer";
    } else {
        ll_->deleteLayer(new_layer);
    }

    // TODO: Freelist is not updated. Feelist is bs! For now..

    // Undo subtractive
    if(subtractive_) {
        for(auto it : removed_from_){
            for(uint16_t label : it.second) {
                ll_->getLayer(it.first)->addLabel(label);
            }
        }
    }

}

void LayerFromLabels::redo(){
    boost::shared_ptr<Layer> layer;

    if(!applied_once_){
        layer = ll_->addLayer();
    }
    else {
        layer = ll_->addLayerWithId(new_layer_id_);
    }

    layer->setName(layer_name_);
    new_layer_id_ = layer->getId();

    if(subtractive_){
        // Subtractive remove labels from layers
        for(uint16_t label : *labels_) {
            const LayerSet & ls = ll_->getLayersForLabel(label);
            for(Layer * l : ls){
                if(l != ll_->getDefaultLayer().get()){
                    removed_from_[l->getId()].push_back(label);
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

bool LayerFromLabels::mergeWith(const QUndoCommand *other){
    return false;
}

int LayerFromLabels::id() const{
    return 4;
}
