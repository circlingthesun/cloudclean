
#include "plugins/featureeval/utils.h"
#include "model/pointcloud.h"
#include "Eigen/Dense"
#include <QDebug>
#include <QTime>
#include <vector>
#include <memory>
#include <pcl/common/pca.h>
#include <pcl/search/flann_search.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "plugins/featureeval/gridsearch.h"
#include "utilities/cv.h"

boost::shared_ptr<std::vector<float> > test_feature(boost::shared_ptr<PointCloud> cloud,
                            const double radius, int max_nn, bool use_depth) {

    boost::shared_ptr<std::vector<float>> stdevs
                        = boost::make_shared<std::vector<float>>(cloud->size());

    std::vector<int> idxs(0);
    std::vector<float> dists(0);

    // 1m radius
    //const double radius = 1.0;

    GridSearch gs(*cloud);

    // center
    Eigen::Map<Eigen::Vector3f> center(cloud->sensor_origin_.data());

    const Octree::Ptr ot = cloud->octree();
    for(uint i = 0; i < cloud->size(); i++){
        idxs.clear();
        dists.clear();

        //ot->radiusSearch(cloud->points[i], radius, idxs, dists);
        //grid_nn_op(i, *cloud, idxs, 1, 50);
        gs.radiusSearch((*cloud)[i], radius, idxs, dists, max_nn);

        // calculate stdev of the distances?
        // bad idea because you have a fixed radius
        // Calculate distance from center of scan

        Eigen::Map<const Eigen::Vector3f> query_point(&(cloud->points[i].x));

        float sum = 0.0f;
        float sum_sq = 0.0f;

        for(int idx : idxs) {
            const float * data = &(cloud->points[idx].x);
            Eigen::Map<const Eigen::Vector3f> point(data);

            float dist;

            if(use_depth)
                dist = (point-center).norm();
            else
                dist = (point-query_point).norm();

            sum += dist;
            sum_sq += dist*dist;
        }

        if(idxs.size() > 2)
            (*stdevs)[i] = (sum_sq - (sum*sum)/(idxs.size()))/((idxs.size())-1);
        else
            (*stdevs)[i] = 0;

    }

    return stdevs;
}

