#ifndef MODEL_CPOINTCLOUD_H_
#define MODEL_CPOINTCLOUD_H_

#include <mutex>
#include <memory>
#include <future>
#include <thread>
#include <QObject>
#include <QString>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/octree.h>
#include <Eigen/Dense>
#include "model/export.h"

typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> Octree;

enum class MODEL_API PointFlags : int8_t {
    selected = 0x001,
    selected2 = 0x002,
    reserved2 = 0x006,
    reserved3 = 0x008,
    reserved4 = 0x010,
    reserved5 = 0x020,
    reserved6 = 0x040
};

enum class MODEL_API CoordinateFrame: bool {
    Camera,
    Laser
};

class MODEL_API PointCloud : public QObject, public pcl::PointCloud<pcl::PointXYZRGB> {
    Q_OBJECT
 public:
    explicit PointCloud();
    ~PointCloud();
    bool point_matches_label(int idx, std::vector<uint16_t> & labels);
    bool save_ptx(const char* filepath, std::vector<uint16_t> labels);
    bool load_ptx(const char* filepath, int decimation_factor = 1);

    void translate(const Eigen::Vector3f& pos);
    void translate(float x, float y, float z)  {
        translate(Eigen::Vector3f(x, y, z));
    }
    void rotate2D(float x, float y);

    Eigen::Affine3f modelview();
    const Octree::Ptr octree();
    boost::shared_ptr<const std::vector<int>> gridToCloudMap() const;
    const std::vector<int> & cloudToGridMap() const;

    bool isVisible();
    void toggleVisible();
    const CoordinateFrame coordinateFrame();
    int scan_width() const;
    int scan_height() const;

    std::vector<boost::shared_ptr<std::vector<int> > > getSelections();

    QString filepath() { return filepath_; }

    bool isMoving() {
        Eigen::Vector4f diff = sensor_origin_ - sensor_origin_future_;
        diff[3] = 0;
        if(diff.norm() > 1e-4)
            return true;
    }

 signals:
    void transformed();
    void progress(int percentage);
    void flagUpdate(boost::shared_ptr<std::vector<int> > idxs = nullptr);
    void labelUpdate(boost::shared_ptr<std::vector<int> > idxs = nullptr);

 public slots:
    void resetOrientation();

 private:
    std::future<Octree::Ptr> fut_octree_;
    Octree::Ptr octree_;
    bool visible_;
    mutable boost::shared_ptr<std::vector<int>> grid_to_cloud_map_;
    std::vector<int> cloud_to_grid_map_;
    boost::shared_ptr<std::mutex> pc_mutex;
    QString filepath_;

    float translation_speed_;
    Eigen::Vector4f sensor_origin_future_;

 public:
    int scan_width_;
    int scan_height_;

    std::vector<uint16_t> labels_;
    std::vector<PointFlags> flags_;
    CoordinateFrame frame_;

    // yeah... i think this is in the octree
    Eigen::Vector3f min_bounding_box_;
    Eigen::Vector3f max_bounding_box_;

    bool deleting_;

};

#endif // MODEL_CPOINTCLOUD_H_
