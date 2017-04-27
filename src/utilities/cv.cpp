#include "utilities/cv.h"
#include "model/pointcloud.h"
#include <cfloat>
#include <Eigen/Dense>
#include <QDebug>
#include <vector>
#include <memory>
//#include "plugins/visualisedepth/gridsearch.h"

boost::shared_ptr<std::vector<int>> makeLookup(boost::shared_ptr<PointCloud> cloud) {
    int size = cloud->scan_width() * cloud->scan_height();
    auto grid_to_cloud = boost::make_shared<std::vector<int>>(size, -1);
    for(uint i = 0; i < cloud->size(); i++) {
        int grid_idx = cloud->cloudToGridMap()[i];
        (*grid_to_cloud)[grid_idx] = i;
    }
    return grid_to_cloud;
}

boost::shared_ptr<std::vector<float>> makeDistmap(
        boost::shared_ptr<PointCloud> cloud,
        boost::shared_ptr<std::vector<float>> distmap) {
    uint size = cloud->scan_width() * cloud->scan_height();

    if(distmap == nullptr || distmap->size() != size)
        distmap = boost::make_shared<std::vector<float>>(size, 0.0f);

    float max_dist = 0.0;

    for(uint i = 0; i < cloud->size(); i++) {
        int grid_idx = cloud->cloudToGridMap()[i];
        pcl::PointXYZRGB & p = (*cloud)[i];
        (*distmap)[grid_idx] = sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2));

        if((*distmap)[i] > max_dist)
            max_dist = (*distmap)[i];
    }

    return distmap;
}

static const double sobel_x[9] = {
    1, 0, -1,
    2, 0, -2,
    1, 0, -1,
};

static const double sobel_y[9] = {
    1, 2, 1,
    0, 0, 0,
    -1, -2, -1,
};

boost::shared_ptr<std::vector<float> > gradientImage(boost::shared_ptr<std::vector<float>> image,
        int w, int h,
        boost::shared_ptr<std::vector<float>> out_image) {

    uint size = w*h;

    if(out_image == nullptr || out_image->size() != size) {
        out_image = boost::make_shared<std::vector<float>>(size, 0);
    }

    float * grad_mag = &out_image->at(0);

    // Calculate the gradient magnitude
    for(int x = 0; x < w; x++){
        for(int y = 0; y < h; y++){
            float gx = convolve_op(w, h, &(*image)[0], x, y, sobel_x, 3);
            float gy = convolve_op(w, h, &(*image)[0], x, y, sobel_y, 3);
            grad_mag[x+y*w] = sqrt(gx*gx + gy*gy);
        }
    }

    return out_image;
}

boost::shared_ptr<std::vector<float> > convolve(
        boost::shared_ptr<std::vector<float>> image,
        int w, int h, const double * filter, const int filter_size,
        boost::shared_ptr<std::vector<float>> out_image) {

    uint size = w*h;
    if(out_image == nullptr || out_image->size() != size) {
        out_image = boost::make_shared<std::vector<float>>(size, 0);
    }

    float * img = &out_image->at(0);

    for(int x = 0; x < w; x++){
        for(int y = 0; y < h; y++){
            img[x+y*w] = convolve_op(w, h, &(*image)[0], x, y, filter, filter_size);
        }
    }

    return out_image;
}


boost::shared_ptr<std::vector<float> > morphology(boost::shared_ptr<std::vector<float>> image,
        int w, int h, const int *strct, int strct_size,
        Morphology type,
        boost::shared_ptr<std::vector<float>> out_image) {

    uint size = w*h;
    if(out_image == nullptr || out_image->size() != size) {
        out_image = boost::make_shared<std::vector<float>>(size, 0);
    }

    //float * img = &out_image->at(0);

    // Calculate the gradient magnitude
    for(int x = 0; x < w; x++){
        for(int y = 0; y < h; y++){
            morph_op(&(*image)[0], w, h, &(*out_image)[0], x, y, strct, strct_size, type);
        }
    }

    return out_image;
}

// stdev on depth image
boost::shared_ptr<std::vector<float> > stdev(
        boost::shared_ptr<std::vector<float>> image,
        int w, int h, const int local_size,
        boost::shared_ptr<std::vector<float>> out_image) {

    uint size = w*h;
    if(out_image == nullptr || out_image->size() != size) {
        out_image = boost::make_shared<std::vector<float>>(size, 0);
    }

    float * img = &out_image->at(0);

    for(int x = 0; x < w; x++){
        for(int y = 0; y < h; y++){
            img[x+y*w] = stdev_op(w, h, &(*image)[0], x, y, local_size);
        }
    }

    return out_image;
}

boost::shared_ptr<std::vector<float> > interpolate(
        boost::shared_ptr<std::vector<float>> image,
        int w, int h, const int nsize,
        boost::shared_ptr<std::vector<float>> out_image) {

    uint size = w*h;
    if(out_image == nullptr || out_image->size() != size) {
        out_image = boost::make_shared<std::vector<float>>(size, 0);
    }

    //float * img = &out_image->at(0);

    // Calculate the gradient magnitude
    for(int x = 0; x < w; x++){
        for(int y = 0; y < h; y++){
            interp_op(&(*image)[0], w, h, &(*out_image)[0], x, y, nsize);
        }
    }

    return out_image;
}

boost::shared_ptr<std::vector<float> > stdev_dist(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                            const double radius, int max_nn, bool use_depth) {

    boost::shared_ptr<std::vector<float>> stdevs
                        = boost::make_shared<std::vector<float>>(cloud->size());

    std::vector<int> idxs(0);
    std::vector<float> dists(0);

    // 1m radius
    //const double radius = 1.0;

    //GridSearch gs(*cloud);

    // center
    Eigen::Map<Eigen::Vector3f> center(cloud->sensor_origin_.data());

    double resolution = 0.02;
    Octree::Ptr octree = Octree::Ptr(new Octree(resolution));
    octree->setInputCloud(cloud);
    octree->defineBoundingBox();
    octree->addPointsFromInputCloud();

    //const Octree::Ptr ot = cloud->octree();
    for(uint i = 0; i < cloud->size(); i++){
        idxs.clear();
        dists.clear();

        octree->radiusSearch(cloud->points[i], radius, idxs, dists, max_nn);
        //grid_nn_op(i, *cloud, idxs, 1, 50);
        //gs.radiusSearch((*cloud)[i], radius, idxs, dists, max_nn);

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


boost::shared_ptr<std::vector<float>> cloudToGrid(const std::vector<int> &map,
        uint img_size,
        boost::shared_ptr<std::vector<float>> input,
        boost::shared_ptr<std::vector<float>> img) {

    if(img == nullptr || img->size() != img_size)
        img = boost::make_shared<std::vector<float>>(img_size, 0.0f);

    for(uint i = 0; i < map.size(); i++) {
        int grid_idx = map[i];
        (*img)[grid_idx] = (*input)[i];
    }

    return img;
}

boost::shared_ptr<std::vector<Eigen::Vector3f> > getHist(boost::shared_ptr<PointCloud> cloud, double radius, uint max_nn) {

    boost::shared_ptr<std::vector<Eigen::Vector3f> > eigen_vals =
            boost::make_shared<std::vector<Eigen::Vector3f>>(cloud->size());

    pcl::KdTreeFLANN<pcl::PointXYZRGB> search;
    search.setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr(cloud.get(), boost::serialization::null_deleter()));


    QTime total;
    total.start();

    int less_than_three_points_count = 0;

    boost::shared_ptr <std::vector<int> > kIdxs;
    kIdxs = boost::shared_ptr <std::vector<int> >(new std::vector<int>);
    std::vector<float> kDist;

    float min = FLT_MAX;
    float max = FLT_MIN;

    // For every point
    for(unsigned int i = 0; i < cloud->size(); i++){

        search.radiusSearch(i, radius, *kIdxs, kDist, max_nn);

        if(kIdxs->size() > max_nn){
            qDebug() << "Whoops! Too many";
            continue;
        }

        if(kIdxs->size() < 3) {
            less_than_three_points_count++;
            (*eigen_vals)[i] = Eigen::Vector3f(0, 0, 1.0f); // Assume isolated point
            continue;
        }

        pcl::PCA<pcl::PointXYZRGB> pcEstimator(true);
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr const_cloud(cloud.get(), boost::serialization::null_deleter());
        pcEstimator.setInputCloud (const_cloud);
        pcEstimator.setIndices(kIdxs);
        (*eigen_vals)[i] = pcEstimator.getEigenValues();


        (*eigen_vals)[i].normalize(); // SHOULD THIS BE NORMALISED?

    }

    /*
    for(unsigned int i = 0; i < cloud->size(); i++){
        (*eigen_vals)[i] = ((*eigen_vals)[i] - Eigen::Vector3f(min, min, min) ) / (max - min);
    }
    */

    qDebug() << "Radius: " << radius << " Max_nn: " << max_nn << " Time: " << total.elapsed()/1000.0f << "Sec";
    qDebug("Points with less than %d neighbours: %d", max_nn, less_than_three_points_count);
    qDebug("Max: %f, Min: %f", max, min);

    return eigen_vals;
}
