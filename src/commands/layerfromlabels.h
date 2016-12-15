#ifndef LayerFromLabels_H
#define LayerFromLabels_H

#include <vector>
#include <map>
#include <memory>
#include <QUndoCommand>
#include <QColor>
#include <QString>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include "commands/export.h"

class LayerList;
class PointCloud;
class Layer;

class COMMAND_API LayerFromLabels : public QUndoCommand
{
 public:
    LayerFromLabels(boost::shared_ptr<std::vector<uint16_t> > labels,
                    LayerList * ll, QString layer_name = "New Layer", bool subtractive = true);
    QString actionText();
    virtual void undo();
    virtual void redo();
    virtual bool mergeWith(const QUndoCommand *other);
    virtual int id() const;

 private:
    bool subtractive_;
    boost::shared_ptr<std::vector<uint16_t> > labels_;
    LayerList * ll_;
    uint new_layer_id_;
    std::map<uint, std::vector<uint16_t> > removed_from_;
    QString layer_name_;
    bool applied_once_;
};

#endif // LayerFromLabels_H
