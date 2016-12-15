#ifndef LAYERLISTVIEW_H
#define LAYERLISTVIEW_H

#include <memory>
#include <QDockWidget>
#include <QItemSelection>
#include "model/layerlist.h"
#include "model/cloudlist.h"

namespace Ui {
class LayerListView;
}

class LayerListView : public QDockWidget
{
    Q_OBJECT
    
 public:
    LayerListView(QUndoStack *us, LayerList * ll,
                           CloudList * cl, uint8_t & selection_mask, QWidget *parent = 0);
    ~LayerListView();

 public slots:
    void selectLayer(boost::shared_ptr<Layer> layer);
    void selectionToLayer();

 private slots:
    void intersectSelectedLayers(bool subtractive=false);
    void intersectSelectedLayersSubtract();
    void mergeSelectedLayers(bool copy=false);
    void mergeSelectedLayersCopy();

    void contextMenu(const QPoint &pos);
    void setColor();

 private:
    LayerList * ll_;
    CloudList * cl_;
    QUndoStack * us_;
    Ui::LayerListView *ui_;
    uint8_t & selection_mask_;
};

#endif // LAYERLISTVIEW_H
