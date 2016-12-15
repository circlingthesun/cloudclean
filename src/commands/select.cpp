#include "select.h"

#include <QDebug>
#include <boost/make_shared.hpp>

Select::Select(boost::shared_ptr<PointCloud> pc,
        boost::shared_ptr<std::vector<int> > indices,
        bool deselect_action,
        uint8_t selection_mask,
        bool destructive,
        boost::shared_ptr<std::vector<uint16_t> > exclude_labels,
        QUndoCommand *parent)
        : QUndoCommand(parent) {
    indices_.reset(new std::vector<int>());
    pc_ = pc;
    deselect_action_ = deselect_action;
    destructive_ = destructive;

    // Check that select and deselect are not already in desired state

    selectmask_ = selection_mask;

    auto is_selected = [this, &pc] (int idx) {
        return bool(selectmask_ & uint8_t(pc->flags_[idx]));
    };

    auto is_excluded = [&exclude_labels, &pc] (int idx) {
        for(uint16_t elabel : *exclude_labels) {
            if(elabel == pc->labels_[idx])
                return true;
        }
        return false;
    };

    if(indices != nullptr){
        for(int idx : *indices) {
            if(is_excluded(idx))
                continue;
            if(!is_selected(idx) && deselect_action_)
                continue;
            if(is_selected(idx) && !deselect_action_)
                continue;

            indices_->push_back(idx);
        }
    }
}

QString Select::actionText(){
    return "Selection";
}

void Select::undo(){
    if(pc_.expired())
        return;

    boost::shared_ptr<PointCloud> pc = pc_.lock();

    boost::shared_ptr<std::vector<int> > update;
    update.reset(new std::vector<int>());

    for(int idx : *indices_) {
        PointFlags &pf = pc->flags_[idx];

        if(destructive_) {
            pf = PointFlags(old_selection_[idx]);
        }
        else if(deselect_action_) {
            pf = PointFlags(selectmask_ | uint8_t(pf));
        }
        else {
            pf = PointFlags(~(selectmask_) & uint8_t(pf));
        }
        update->push_back(idx);
    }

    emit pc->flagUpdate(update);
    //pc->flagsUpdated(update);

}

void Select::redo(){
    if(pc_.expired())
        return;

    boost::shared_ptr<PointCloud> pc = pc_.lock();

    boost::shared_ptr<std::vector<int> > update;
    update.reset(new std::vector<int>());

    for(int idx : *indices_) {
        PointFlags &pf = pc->flags_[idx];

        if(destructive_) {
            old_selection_[idx] = (uint8_t)pf;
            if(deselect_action_)
                pf = PointFlags(0);
            else
                pf = PointFlags(selectmask_);

        }
        else if(deselect_action_)
            pf = PointFlags(~(selectmask_) & uint8_t(pf));
        else
            pf = PointFlags((selectmask_) | uint8_t(pf));
        update->push_back(idx);
    }


    emit pc->flagUpdate(update);

}

boost::shared_ptr<std::vector<int> > mergeUnique(
        boost::shared_ptr<std::vector<int> > a,
        boost::shared_ptr<std::vector<int> > b){

    boost::shared_ptr<std::vector<int> > c;
    c.reset(new std::vector<int>(a->size() + b->size(), 0));

    std::copy(a->begin(), a->end(), c->begin());
    std::copy(b->begin(), b->end(), c->begin() + a->size());
    std::sort(c->begin(), c->end());
    auto it = std::unique(c->begin(), c->end());
    c->resize(std::distance( c->begin(), it));

    return c;
}

bool Select::mergeWith(const QUndoCommand *other){
    if (other->id() != id())
        return false;

    const Select * o = static_cast<const Select *>(other);

    if(destructive_ != o->destructive_)
        return false;

    if(this->selectmask_ != o->selectmask_)
        return false;

    if (o->pc_.lock() != pc_.lock())
        return false;

    // transfer new mappings
    if(destructive_) {
        for(int idx : *o->indices_){
            if(old_selection_.find(idx) == old_selection_.end())
                old_selection_[idx] = (*o->old_selection_.find(idx)).second;
        }
    }

    indices_ = mergeUnique(indices_, o->indices_);

    return true;
}

int Select::id() const {
    return 1;
}

