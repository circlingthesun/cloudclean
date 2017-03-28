#ifndef CLOUDLISTVIEW_H
#define CLOUDLISTVIEW_H

#include <memory>
#include <QDockWidget>
#include <QItemSelection>
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/mainwindow.h"

namespace Ui {
class CloudListView;
}

class CloudListView : public QDockWidget
{
    Q_OBJECT
    
 public:
    CloudListView(QUndoStack *us, LayerList * ll,
                           CloudList * cl, GLWidget *glwidget,
                           QWidget *parent = 0);
    ~CloudListView();

 private slots:
   void dataChanged();
   void contextMenu(const QPoint &pos);

 public slots:
   void deselectAllPoints();
   void selectAllPoints();
   void countVisible();
   void invertSelection();

 private:
    LayerList * ll_;
    CloudList * cl_;
    QUndoStack *us_;
    GLWidget * glwidget_;
    Ui::CloudListView *ui_;
    MainWindow * mw_;
};

#endif // CLOUDLISTVIEW_H
