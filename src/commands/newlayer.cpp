#include "newlayer.h"

#include <QDebug>

#include <model/layerlist.h>
#include <model/pointcloud.h>
#include <model/layer.h>

NewLayer::NewLayer(boost::shared_ptr<PointCloud> pc,
                                 boost::shared_ptr<std::vector<int> > idxs,
                                 LayerList * ll) {
    pc_ = pc;
    idxs_ = idxs;
    ll_ = ll;
    applied_once_ = false;
    new_layer_id_ = -1;
}

QString NewLayer::actionText(){
    return "New Layer";
}

void NewLayer::undo(){
    // Change labels back
    for(int idx : *idxs_){
        pc_->labels_[idx] = new_to_old[pc_->labels_[idx]];
    }

    boost::shared_ptr<Layer> new_layer = ll_->getLayer(new_layer_id_);

    if(new_layer == nullptr){
        qDebug() << "Could not uncreate layer " << new_layer_id_;
        return;
    }

    // Delete layer
    layer_color_ = new_layer->getColor();
    ll_->deleteLayer(new_layer);
    qDebug() << "Un created layer: " << new_layer_id_;

}

uint16_t NewLayer::getNewLabel(uint16_t old, boost::shared_ptr<Layer> layer) {
    auto new_label_it = old_to_new.find(old);

    bool unknown_mapping = (new_label_it == old_to_new.cend());

    if (unknown_mapping) {
        // Create a new label
        uint16_t new_label = ll_->createLabelId();
        layer->addLabel(new_label);

        // Add all layers assocatied with the old label
        // to the new label
        ll_->copyLayerSet(old, new_label);

        // Keep track of what happened
        old_to_new[old] = new_label;
        new_to_old[new_label] = old;

        return new_label;
    } else {
        // this will happen in a redo
        //layer->addLabel(old_to_new[old]);
        //ll_->copyLayerSet(old, old_to_new[old]);
    }
    return old_to_new[old];
}

void NewLayer::redo(){
    // When redoing we need to make sure that the same labling is achieved
    // every time, as subsequent command might reference the labled it their redo
    // lables can be freed when the command is deleted, assuming the stack is infinite
    // also, disregarding deletions for merges

    boost::shared_ptr<Layer> layer;
    if(applied_once_)
        layer = ll_->addLayerWithId(new_layer_id_);
    else
        layer = ll_->addLayer();

    new_layer_id_ = layer->getId();
    //new_layer_ = layer;
    if(new_to_old.size() != 0)
        layer->setColor(layer_color_);


    // Assign new labels to points in the new layer
    for(int idx : *idxs_){
        pc_->labels_[idx] = getNewLabel(pc_->labels_[idx], layer);
    }

    // Need to do work that getNewLabel doesnt do on a redo
    if(applied_once_) {
        //Add labels to layers
        for(auto it: new_to_old) {
            layer->addLabel(it.first);
            ll_->copyLayerSet(it.second, it.first);
        }
    }
    applied_once_ = true;

    emit pc_->labelUpdate();
    emit pc_->flagUpdate();

    // TODO(Rickert) : Update color lookup buffer
    qDebug() << "Created layer: " << new_layer_id_;
}

bool NewLayer::mergeWith(const QUndoCommand *other){
    return false;
}

int NewLayer::id() const{
    return 2;
}
