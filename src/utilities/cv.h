#ifndef UTILITIES_CV_H
#define UTILITIES_CV_H

#include <memory>
#include <vector>
#include <cassert>
#include <limits>
#include <QDebug>
#include <QTime>
#include <boost/make_shared.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include <pcl/common/pca.h>
#include <pcl/search/flann_search.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "model/pointcloud.h"
#include "utilities/export.h"

enum class Morphology{ERODE, DILATE};

UTIL_API boost::shared_ptr<std::vector<float>> makeDistmap(
        boost::shared_ptr<PointCloud> cloud,
        boost::shared_ptr<std::vector<float>> distmap = nullptr
);

UTIL_API boost::shared_ptr<std::vector<float> > gradientImage(boost::shared_ptr<std::vector<float>> image,
        int w, int h,
        boost::shared_ptr<std::vector<float>> out_image = nullptr);

UTIL_API boost::shared_ptr<std::vector<float> > convolve(
        boost::shared_ptr<std::vector<float>> image,
        int w, int h, const double * filter, const int filter_size,
        boost::shared_ptr<std::vector<float>> out_image = nullptr);

UTIL_API boost::shared_ptr<std::vector<float> > morphology(
        boost::shared_ptr<std::vector<float>> image,
        int w, int h, const int * strct, int strct_size,
        Morphology type,
        boost::shared_ptr<std::vector<float>> out_image = nullptr);

UTIL_API boost::shared_ptr<std::vector<float> > stdev(
        boost::shared_ptr<std::vector<float>> image,
        int w, int h, const int local_size,
        boost::shared_ptr<std::vector<float>> out_image = nullptr);

template <typename T>
void min_max(const std::vector<T> & v, T & min, T & max){
    min = std::numeric_limits<T>::max();
    max = std::numeric_limits<T>::min();
    for(auto val : v){
        if(val > max)
            max = val;
        else if(val < min)
            min = val;
    }

}

UTIL_API boost::shared_ptr<std::vector<float> > interpolate(
        boost::shared_ptr<std::vector<float>> image,
        int w, int h, const int nsize,
        boost::shared_ptr<std::vector<float>> out_image = nullptr);

UTIL_API boost::shared_ptr<std::vector<float> > stdev_dist(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                 const double radius, int max_nn = 0, bool use_depth = false);

UTIL_API boost::shared_ptr<std::vector<float>> cloudToGrid(const std::vector<int> & map, uint img_size,
        boost::shared_ptr<std::vector<float>> input,
        boost::shared_ptr<std::vector<float>> img = nullptr);

UTIL_API boost::shared_ptr<std::vector<Eigen::Vector3f> > getHist(boost::shared_ptr<PointCloud> cloud, double radius, uint max_nn = 0);

template <typename PointT>
boost::shared_ptr<std::vector<Eigen::Vector3f> > getPCA(pcl::PointCloud<PointT> * cloud, double radius, uint max_nn = 0) {

    boost::shared_ptr<std::vector<Eigen::Vector3f> > eigen_vals =
            boost::make_shared<std::vector<Eigen::Vector3f>>(cloud->size());

    typename pcl::PointCloud<PointT>::ConstPtr cptr(cloud, boost::serialization::null_deleter());
    pcl::KdTreeFLANN<PointT> search;
    search.setInputCloud(cptr);

    int less_than_three_points_count = 0;

    boost::shared_ptr <std::vector<int> > kIdxs;
    kIdxs = boost::shared_ptr <std::vector<int> >(new std::vector<int>);
    std::vector<float> kDist;

    // For every point
    for(uint i = 0; i < cloud->size(); i++){

        search.radiusSearch(i, radius, *kIdxs, kDist, max_nn);

        if(kIdxs->size() > max_nn && max_nn > 0){
            qDebug() << "Whoops! Too many";
            continue;
        }

        if(kIdxs->size() < 3) {
            less_than_three_points_count++;
            (*eigen_vals)[i] = Eigen::Vector3f(0, 0, 1.0f); // Assume isolated point
            continue;
        }

        pcl::PCA<PointT> pcEstimator(true);
        pcEstimator.setInputCloud (cptr);
        pcEstimator.setIndices(kIdxs);
        (*eigen_vals)[i] = pcEstimator.getEigenValues();
    }

    return eigen_vals;
}

inline float clamp(float x, float a, float b)
{
    return x < a ? a : (x > b ? b : x);
}

template<class PointT, class NormalT>
boost::shared_ptr<std::vector<float> > normal_stdev(
        typename pcl::PointCloud<PointT>::Ptr cloud,
        typename pcl::PointCloud<NormalT>::Ptr normals,
        double radius,
        int max_nn) {

    boost::shared_ptr<std::vector<float> > std_devs =
            boost::make_shared<std::vector<float>>(cloud->size(), 0);

    pcl::KdTreeFLANN<PointT> search;
    search.setInputCloud(cloud);


    QTime t;
    t.start();

    int less_than_three_points_count = 0;

    boost::shared_ptr <std::vector<int> > kIdxs;
    kIdxs = boost::shared_ptr <std::vector<int> >(new std::vector<int>(cloud->size(), 0));
    std::vector<float> kDist;

    // For every point
    for(unsigned int i = 0; i < cloud->size(); i++){

        if(i % 20000 == 0) {
            int ms = t.restart();
            qDebug() << "so " << less_than_three_points_count << "out of " << i << "points have less than 2 neighbours";
            qDebug() << "Radius: " << radius << "Max nn: " << max_nn;
            qDebug() << "% done: " << float(i) / cloud->size();
            qDebug() << "MS per loop: " << float(ms)/20000.0f;
        }

        // TODO(Rickert): Sort out Nan in formal estimation
        // Skip NAN's
        if((*normals)[i].data_n[0] != (*normals)[i].data_n[0])
            continue;

        search.radiusSearch(i, radius, *kIdxs, kDist, max_nn);

        if(kDist.size() < 3) {
            less_than_three_points_count++;
            continue;
        }

        std::vector<float> angles;
        angles.resize(kDist.size());

        Eigen::Map<Eigen::Vector3f> current((*normals)[i].data_n);

        float sumOfSquares = 0.0f;
        float sum = 0.0f;

        for(int idx : *kIdxs) {

            Eigen::Map<Eigen::Vector3f> neighbour((*normals)[idx].data_n);

            float cosine = neighbour.dot(current) /
                    neighbour.norm()*current.norm();

            cosine = clamp(cosine, 0.0f, 1.0f);

            // Normalised angle
            float angle = acos(cosine)/M_PI;

            sum += angle;
            sumOfSquares += angle*angle;
        }

        float std_dev = sqrt( (sumOfSquares/angles.size()) - pow(sum/angles.size(), 2));

        if(std_dev != std_dev) {
            //qDebug() << "Bugger! NAN";
            continue;
        }


        (*std_devs)[i] = std_dev;

    }

    return std_devs;
}

/// Inline functions:

inline float convolve_op(
        int w, int h, float * source, int x, int y,
        const double * filter, int filter_size) {
    assert(filter_size%2 != 0);

    int start = -filter_size/2;
    int end = filter_size/2;

    float sum = 0.0f;

    for(int iy = start; iy <= end; iy++){
        for(int ix = start; ix <= end; ix++){
            // map pos
            int _x = ix + x;
            int _y = iy + y;

            // wraps around on edges
            if(_x < 0)
                _x = w+_x;
            else if(_x > w-1)
                _x = _x-w;

            if(_y < 0)
                _y = h+_y;
            else if(_y > h-1)
                _y = _y-h;

            // map index
            int i = _x + w * _y;
            // filter index
            int f = ix+end + filter_size*(iy+end);

            float val = source[i] * filter[f];

            sum += val;
        }
    }
    return sum;
}

inline void morph_op(float * source, int w, int h, float * dest, int x, int y,
               const int * strct, int strct_size, const Morphology type) {
    assert(strct_size%2 != 0);

    int start = -strct_size/2;
    int end = strct_size/2;

    // Center of stuct is center

    float min_or_max;
    if(type == Morphology::DILATE)
        min_or_max = 0;
    else
        min_or_max = std::numeric_limits<float>::max();

    for(int iy = start; iy <= end; iy++){
        for(int ix = start; ix <= end; ix++){
            // map pos
            int _x = ix + x;
            int _y = iy + y;

            // wraps around on edges
            if(_x < 0)
                _x = w+_x;
            else if(_x > w-1)
                _x = _x-w;

            if(_y < 0)
                _y = h+_y;
            else if(_y > h-1)
                _y = _y-h;

            // strct index
            int s = ix+1 + 3*(iy+1);

            // Skip elements not in the structure
            if(strct[s] != 1)
                continue;

            // map index
            int i = _x + w * _y;

            if(type == Morphology::DILATE){
                if(source[i] > min_or_max)
                    min_or_max = source[i];
            }
            else {
                 if(source[i] < min_or_max)
                    min_or_max = source[i];
            }
        }
    }


    dest[x+y*w] = min_or_max;
}

inline float stdev_op(
        int w, int h, float * source, int x, int y, int size) {
    assert(size%2 != 0);

    int start = -size/2;
    int end = size/2;

    float sum = 0.0f;
    float sum_sq = 0.0f;

    for(int iy = start; iy <= end; iy++){
        for(int ix = start; ix <= end; ix++){
            // map pos
            int _x = ix + x;
            int _y = iy + y;

            // wraps around on edges
            if(_x < 0)
                _x = w+_x;
            else if(_x > w-1)
                _x = _x-w;

            if(_y < 0)
                _y = h+_y;
            else if(_y > h-1)
                _y = _y-h;

            // map index
            int i = _x + w * _y;

            sum += source[i];
            sum_sq += source[i] * source[i];;
        }
    }
    return (sum_sq - (sum*sum)/(w*h))/((w*h)-1);
}

inline void interp_op(float * source, int w, int h, float * dest, int x, int y,
               int nsize) {
    assert(nsize%2 != 0);

    int start = -nsize/2;
    int end = nsize/2;

    // Dont need to interopolate
    if(source[x+y*w] > 1e-6){
        dest[x+y*w] = source[x+y*w];
        return;
    }

    float sum = 0.0f;
    int n = 0;

    for(int iy = start; iy <= end; iy++){
        for(int ix = start; ix <= end; ix++){
            // map pos
            int _x = ix + x;
            int _y = iy + y;

            // wraps around on edges
            if(_x < 0)
                _x = w+_x;
            else if(_x > w-1)
                _x = _x-w;

            if(_y < 0)
                _y = h+_y;
            else if(_y > h-1)
                _y = _y-h;

            // source index
            int i = _x + w * _y;

            // Skip null values
            if(source[i]  < 1e-6)
                continue;

            sum += source[i];
            n++;
        }
    }

    if(n == 0){
        dest[x+y*w] = source[x+y*w];
        return;
    }

    dest[x+y*w] = sum/n;
}


inline void grid_nn_op(int idx,
                       PointCloud & cloud,
                       std::vector<int> & idxs,
                       double radius,
                       uint max_nn) {


    int h = cloud.scan_height();
    int w = cloud.scan_width();

    boost::shared_ptr<const std::vector<int>> grid_to_cloud = cloud.gridToCloudMap();

    int grid_idx = cloud.cloudToGridMap()[idx];
    int x = grid_idx / h;
    int y = grid_idx % h;

    Eigen::Map<Eigen::Vector3f> query_point(&cloud.points[idx].x, 3);

    const double rad_sq = radius * radius;
    const int max_ring_size = (w>h?w:h)/2;
    int outside_radius = 0;

    for(int ring = 1; ring <= max_ring_size ; ring++){
        //qDebug("Ring %d", ring);
        // Iterator over edge of square
        for(int iy = -ring; iy <= ring; iy++){
            for(int ix = -ring; ix <=ring; ix++){

                // map pos
                int _x = ix + x;
                int _y = iy + y;

                // wraps around on edges
                if(_x < 0)
                    _x = w+_x;
                else if(_x > w-1)
                    _x = _x-w;

                if(_y < 0)
                    _y = h+_y;
                else if(_y > h-1)
                    _y = _y-h;


                // source index
                int i = _y + h * _x;
                int idx = (*grid_to_cloud)[i];

                // Only look at valid indexes
                if(idx  != -1){
                    float * data = &(cloud.points[idx].x);
                    Eigen::Map<Eigen::Vector3f> neighbour(data, 3);
                    float sqdist = (neighbour-query_point).squaredNorm();

                    if(sqdist <= rad_sq) {
                        idxs.push_back(idx);
                        if(idxs.size() > max_nn)
                            return;
                    } else {
                        outside_radius++;
                    }

                    int side_len = (ring*2+1);
                    int max_err = side_len*2 + 2*(side_len-2);

                    // If error is more than the indexes in a ring
                    if(outside_radius > max_err){
                        //qDebug() << "Exeeded" << max_err;
                        return;
                    }
                }


                // Skip the inner values
                if(iy != -ring && iy != ring && ix == -ring) {
                    ix = ring-1;
                }
            }
        }
    }

}


static const double gaussian[25] = {
    0.00296901674395065, 0.013306209891014005, 0.02193823127971504, 0.013306209891014005, 0.00296901674395065,
    0.013306209891014005, 0.05963429543618023, 0.09832033134884507, 0.05963429543618023, 0.013306209891014005,
    0.02193823127971504, 0.09832033134884507, 0.16210282163712417, 0.09832033134884507, 0.02193823127971504,
    0.013306209891014005, 0.05963429543618023, 0.09832033134884507, 0.05963429543618023, 0.013306209891014005,
    0.00296901674395065, 0.013306209891014005, 0.02193823127971504, 0.013306209891014005, 0.00296901674395065,
};

inline float angle(Eigen::Vector3f &a, Eigen::Vector3f &b){
    float cosine = a.dot(b) / (a.norm()*b.norm());
    cosine = clamp(cosine, 0.0f, 1.0f);
    float angle = acos(cosine);
    return angle;
    //return angle/M_PI;
}

inline float cosine(Eigen::Vector3f &a, Eigen::Vector3f &b){
    float cosine = a.dot(b) / (a.norm()*b.norm());
    return clamp(cosine, 0.0f, 1.0f);
}

#endif  // UTILITIES_CV_H
