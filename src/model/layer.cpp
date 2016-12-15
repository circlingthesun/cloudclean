#include "model/layer.h"
#include <cstdlib>
#include <QDebug>

Layer::Layer(std::map<uint16_t, LayerSet> & layer_lookup_table, uint id)
    : layer_lookup_table_(layer_lookup_table) {
    name_ = QString("New Layer ") + QString::number(last_id_);
    color_.setHsl(rand()%359, rand()%55 + 200, 127);
    visible_ = true;
    id_ = id;
}

Layer::~Layer(){
    // Delete all references in layer_lookup_table_
    for(int16_t label : labels_){
        // Why would this fail?... think about it
        auto it = layer_lookup_table_[label].find(this);
        if(it != layer_lookup_table_[label].end())
            layer_lookup_table_[label].erase(it);
    }
}

QColor Layer::getColor() const {
    return color_;
}

void Layer::setColor(QColor color){
    color_ = color;
    emit colorChanged();
}

void Layer::setRandomColor(){
    color_.setHsl(rand()%359, rand()%55 + 200, 127);
    emit colorChanged();
}

QString Layer::getName() const {
    return name_;
}

void Layer::setName(QString name) {
    name_ = name;
}

void Layer::addLabel(uint16_t id) {
    layer_lookup_table_[id].insert(this);
    labels_.insert(id);
}

void Layer::removeLabel(uint16_t id) {
    auto it = layer_lookup_table_[id].find(this);
    if(it != layer_lookup_table_[id].end())
        layer_lookup_table_[id].erase(it);
}

const std::set<uint16_t> &Layer::getLabelSet() const{
    return labels_;
}

bool Layer::isVisible() const {
    return visible_;
}

void Layer::toggleVisible() {
    visible_ = !visible_;
}

uint Layer::getId() const {
    return id_;
}

uint Layer::last_id_ = 0;
