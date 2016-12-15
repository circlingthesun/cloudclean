#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QHash>
#include <memory>
#include "gui/export.h"

class QUndoStack;
class CloudList;
class LayerList;
class QProgressBar;
class FlatView;
class GLWidget;
class CloudListView;
class LayerListView;
class GLData;
class ActionManager;
class QUndoStack;
class QMenuBar;
class QStackedWidget;
class QToolBox;
class QButtonGroup;
class QPushButton;

#ifdef LEAP_SUPPORT
    #include "gui/leaplistener.h"
    class LeapListener;
    namespace Leap {
        class Controller;
    }
#endif

class GUI_API MainWindow : public QMainWindow {
    Q_OBJECT
    
 public:
    explicit MainWindow(QUndoStack * us, CloudList *cl, LayerList *ll, QWidget *parent = 0);
    virtual ~MainWindow();

    void setSelectMask(uint8_t mask);

    void addMenu(QAction * action, QString menu_name);
    void removeMenu(QAction * action, QString menu_name);
    inline int addTab(QWidget * widget, QString label) {
        return tabs_->addTab(widget, label);
    }

    inline void removeTab(int idx) {
        tabs_->removeTab(idx);
    }

 public slots:
    void loadFile();
    void saveLayer();

    void startBgAction(QString name, bool deterministic = false);
    void stopBgAction(QString name);
    void progBgAction(QString name, int prog);

 private slots:
    void contextMenu(const QPoint &pos);


 public:
   FlatView * flatview_;
   GLWidget * glwidget_;
   QToolBar * toolbar_;
   QStackedWidget * tooloptions_;

   QMenu * file_menu_;
   QMenu * edit_menu_;
   QMenu * view_menu_;
   QMenu * window_menu_;

   QDockWidget * options_dock_;
   QDockWidget * selection_dock_;

   uint8_t select_mask_;
   bool deselect_;
   bool edit_mode_;

   CloudListView * clv_;
   LayerListView * llv_;

 protected:
   void closeEvent(QCloseEvent *event);

 private:
    QStatusBar * statusbar_;
    QProgressBar *progressbar_;
    QTabWidget * tabs_;

    GLData * gld_;
    CloudList * cl_;
    LayerList * ll_;

    QUndoStack * us_;

    QMenuBar * mb_;
    QHash<QString, QMenu *> menus_;

    std::vector<QPushButton *> buttons_;
    QButtonGroup * button_group_;

#ifdef LEAP_SUPPORT
    LeapListener * listener;
    Leap::Controller * controller;
#endif

};

#endif // MAINWINDOW_H
