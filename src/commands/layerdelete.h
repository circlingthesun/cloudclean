#ifndef LayerDelete_H
#define LayerDelete_H

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

class COMMAND_API LayerDelete : public QUndoCommand {
 public:
    LayerDelete(boost::shared_ptr<Layer> layer, LayerList * ll);
    QString actionText();
    virtual void undo();
    virtual void redo();
    virtual bool mergeWith(const QUndoCommand *other);
    virtual int id() const;

 private:
    std::vector<uint16_t> labels_;
    QColor col_;
    QString name_;
    LayerList * ll_;
    uint id_;
};

#endif // LayerFromLabels_H
