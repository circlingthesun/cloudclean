#ifndef LAYERLIST_H
#define LAYERLIST_H

#include <vector>
#include <memory>
#include <mutex>
#include <set>
#include <unordered_map>
#include <QAbstractListModel>
#include <QItemSelection>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include "model/layer.h"
#include "model/export.h"

class MODEL_API LayerList : public QAbstractListModel {
    Q_OBJECT
 public:
    explicit LayerList(QObject *parent = 0);
    ~LayerList();
    Qt::ItemFlags flags(const QModelIndex & index) const;
    int columnCount(const QModelIndex &) const;
    int rowCount(const QModelIndex & parent = QModelIndex()) const;
    QVariant data(const QModelIndex & index, int role = Qt::DisplayRole) const;
    bool setData(const QModelIndex & index, const QVariant & value, int role);
    boost::shared_ptr<Layer> addLayer(boost::shared_ptr<Layer> layer);
    boost::shared_ptr<Layer> addLayerWithId(uint id);
    boost::shared_ptr<Layer> addLayer();
    boost::shared_ptr<Layer> addLayer(QString name);
    int16_t createLabelId();

    const LayerSet &getLayersForLabel(int i);
    void copyLayerSet(uint8_t source_label, uint8_t dest_label);

    int getLayerIndex(boost::shared_ptr<Layer> layer) const;
    boost::shared_ptr<Layer> getLayer(uint id);

    uint getLastLabel() const;
    const std::vector<boost::weak_ptr<Layer>> & getSelection() const;
    boost::shared_ptr<Layer> getDefaultLayer() const;
    const std::vector<boost::shared_ptr<Layer>> & getLayers();

    boost::shared_ptr<std::vector<uint16_t> > getHiddenLabels();

    int getLayerIdxByName(QString name);

    void reset();

 signals:
    void layerUpdate(boost::shared_ptr<Layer> layer);
    void lookupTableUpdate();
    void changedSelection(std::vector<boost::weak_ptr<Layer> > selection);

 public slots:
    void selectionChanged(const std::vector<int> & selection);
    void deleteLayer(boost::shared_ptr<Layer> layer);
    void deleteLayer();
    void deleteLayer(int idx);

 private:
    std::mutex * mtx_;
    std::map<uint16_t, LayerSet> layer_lookup_table_; // label associated with layer
    std::vector<uint16_t> free_labels_;

    uint last_label_;
    std::vector<boost::weak_ptr<Layer>> selection_;
    boost::shared_ptr<Layer> default_layer_;
    std::vector<boost::shared_ptr<Layer>> layers_; // a layer is a group of labels
    std::map<uint, boost::shared_ptr<Layer>> layer_id_map_;
};

#endif // LAYERLIST_H
