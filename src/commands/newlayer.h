#ifndef NEWLAYERCOMMAND_H
#define NEWLAYERCOMMAND_H

#include <vector>
#include <memory>
#include <map>
#include <QUndoCommand>
#include <QColor>
#include <QString>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include "commands/export.h"

class LayerList;
class PointCloud;
class Layer;

class COMMAND_API NewLayer : public QUndoCommand
{
 public:
    NewLayer(boost::shared_ptr<PointCloud> pc,
                    boost::shared_ptr<std::vector<int> > idxs,
                    LayerList * ll);
    QString actionText();
    virtual void undo();
    virtual void redo();
    virtual bool mergeWith(const QUndoCommand *other);
    virtual int id() const;

 private:
    uint16_t getNewLabel(uint16_t old, boost::shared_ptr<Layer> layer);

 private:
    std::map<uint16_t, uint16_t> old_to_new;
    std::map<uint16_t, uint16_t> new_to_old;

    boost::shared_ptr<PointCloud> pc_;
    boost::shared_ptr<std::vector<int> > idxs_;
    LayerList * ll_;

    uint new_layer_id_;
    QColor layer_color_;

    bool applied_once_;

};

#endif // NEWLAYERCOMMAND_H
