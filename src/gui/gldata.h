#ifndef GLDATA_H
#define GLDATA_H

#include "glheaders.h"
#include <memory>
#include <mutex>
#include <QObject>
#include <QGLBuffer>
#include "model/pointcloud.h"
#include "gui/cloudgldata.h"
#include "model/cloudlist.h"
#include "model/layerlist.h"
#include "gui/export.h"

class GUI_API GLData : public QObject {
    Q_OBJECT
 public:
    GLData(QGLContext * glcontext,
                    CloudList * cl,
                    LayerList *ll,
                    QObject *parent = 0);
    ~GLData();
 signals:
    void update();
    
 public slots:
    void reloadCloud(boost::shared_ptr<PointCloud> cloud);
    void reloadColorLookupBuffer();
    void deleteCloud(boost::shared_ptr<PointCloud> cloud);
    
 public:
    boost::shared_ptr<QGLBuffer> color_lookup_buffer_;
    std::map<boost::shared_ptr<PointCloud>, boost::shared_ptr<CloudGLData> > cloudgldata_;
    float selection_color_[4];


 private:
    CloudList * cl_;
    LayerList * ll_;
    QGLContext * glcontext_;
    std::mutex * clb_mutex_;
};

#endif // GLDATA_H
