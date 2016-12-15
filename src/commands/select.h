#ifndef SELECTCOMMAND_H
#define SELECTCOMMAND_H

#include <vector>
#include <map>
#include <memory>
#include <QUndoCommand>
#include <boost/weak_ptr.hpp>
#include "model/pointcloud.h"
#include "commands/export.h"

class COMMAND_API Select : public QUndoCommand
{
public:
    Select(boost::shared_ptr<PointCloud> pc,
                           boost::shared_ptr<std::vector<int> > selected,
                           bool deselect = false,
                           uint8_t selection_type = 1,
                           bool destructive = true,
                           boost::shared_ptr<std::vector<uint16_t> > exclude_labels = boost::shared_ptr<std::vector<uint16_t> >(new std::vector<uint16_t>()),
                           QUndoCommand *parent = 0);
    QString actionText();
    virtual void undo();
    virtual void redo();
    virtual bool mergeWith(const QUndoCommand *other);
    virtual int id() const;
    
private:
    boost::weak_ptr<PointCloud> pc_;
    boost::shared_ptr<std::vector<int> > indices_;
    std::map<uint, uint8_t> old_selection_;
    uint8_t selectmask_;
    bool deselect_action_;
    bool destructive_;
};

#endif // SELECTCOMMAND_H
