#ifndef LayerGuiSelect_H
#define LayerGuiSelect_H

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

class COMMAND_API LayerFromSelection : public QUndoCommand
{
 public:
    LayerFromSelection(boost::shared_ptr<std::vector<uint16_t> > labels,
                    LayerList * ll, bool subtractive = true);
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
    std::map<Layer *, std::vector<uint16_t> > removed_from_;
    bool applied_once_;
};

#endif // LayerGuiSelect_H
