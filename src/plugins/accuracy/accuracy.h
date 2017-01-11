#ifndef ACCURACY_H
#define ACCURACY_H

#include <set>
#include <boost/weak_ptr.hpp>
#include <QTimer>
#include <QTime>
#include <tuple>

#include "plugins/accuracy/export.h"
#include "pluginsystem/iplugin.h"

class QAction;
class QWidget;
class Core;
class CloudList;
class LayerList;
class FlatView;
class GLWidget;
class MainWindow;
class Layer;
class QLineEdit;
class QLabel;
class QDoubleSpinBox;
class QDockWidget;
class QPushButton;

class ACCURACY_API Accuracy : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.accuracy" FILE "accuracy.json")
 public:
    QString getName();
    void initialize(Core * core);
    void cleanup();
    ~Accuracy();

 signals:
   void enabling();

 public slots:
   std::tuple<float, float, float> sample();

 private slots:
    void enable();
    void disable();


    void start_stop();
    void reset();
    void save();

 private:
    Core * core_;
    CloudList * cl_;
    LayerList * ll_;
    GLWidget * glwidget_;
    FlatView * flatview_;
    MainWindow * mw_;

    QAction * enable_;
    QWidget * dock_widget_;
    QWidget * dock_widget2_;
    bool is_enabled_;

    float target_accuracy_;

    QDockWidget * dock_;
    QDockWidget * dock2_;

    QDoubleSpinBox * target_accuracy_input_;

    std::vector<boost::weak_ptr<Layer> >  target_;
    QTimer timer_;
    QTime time_;

    QLineEdit * elapsed_time_text_;
    QLineEdit * accuracy_text_;
    QLineEdit * filename_text_;
    QLineEdit * test_key_text_;

    QPushButton * start_button_;

    bool started_ = false;

    std::vector<std::tuple<int, float, float, float>> time_accuracy_precision_recall_;
};

#endif  // ACCURACY_H
