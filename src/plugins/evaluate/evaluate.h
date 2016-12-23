#ifndef EVALUATE_H
#define EVALUATE_H

#include "pluginsystem/iplugin.h"
#include <set>
#include <boost/weak_ptr.hpp>
#include "pluginsystem/iplugin.h"
#include "utilities/lasso.h"
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

class Evaluate : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.evaluate" FILE "evaluate.json")
 public:
    QString getName();
    void initialize(Core * core);
    void cleanup();
    ~Evaluate();

 signals:
   void enabling();

 public slots:
   void paint2d();

 private slots:
    void enable();
    void disable();
    void eval();

 private:
    std::tuple<std::vector<int>, std::vector<int>> get_false_selections(std::vector<int> & world_idxs, std::vector<bool> & target_mask);
    std::vector<std::vector<int> > cluster(std::vector<int> & idxs);
    std::vector<int> concaveHull(std::vector<int> & idxs, float simplify);
    boost::shared_ptr<std::vector<int>> lassoPoints(std::vector<int> & idxs, float expand);
    std::vector<Eigen::Vector2f> makePolygon(std::vector<int> & idxs, float expand = 0);

    Eigen::Vector2f getPoint(int idx);
    int dpR(std::vector<int> & idxs, std::vector<bool> & keep, int start_idx, int end_idx, float e);
    std::vector<int> dp(std::vector<int> & idxs, float e);

 private:
    Core * core_;
    CloudList * cl_;
    LayerList * ll_;
    GLWidget * glwidget_;
    FlatView * flatview_;
    MainWindow * mw_;

    QAction * enable_;
    QWidget * settings_;
    bool is_enabled_;

    QLineEdit * precision_;
    QLineEdit * recall_;

    std::vector<boost::weak_ptr<Layer> >  world_layers_;
    std::vector<boost::weak_ptr<Layer> >  target_layers_;
    std::vector<boost::weak_ptr<Layer> >  selection_layers_;
    Lasso * lasso_;
};

#endif  // EVALUATE_H
