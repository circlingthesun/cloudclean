#ifndef VISIUALISE_DEPTH_UTIL
#define VISIUALISE_DEPTH_UTIL

#include <memory>
#include <vector>
#include <cassert>
#include <cfloat>
#include <QTime>
#include <QDebug>
#include <pcl/kdtree/kdtree_flann.h>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "model/pointcloud.h"

boost::shared_ptr<std::vector<float> > test_feature(boost::shared_ptr<PointCloud> cloud,
                                 const double radius, int max_nn = 0, bool use_depth = false);

template <typename PointT>
boost::shared_ptr<std::vector<std::vector<float> > > calcDistHist(pcl::PointCloud<PointT> & cloud, int bins, double radius, int max_nn) {

    //boost::shared_ptr<std::vector<float>> distmap = makeDistmap(cloud);
    //const std::vector<int> & cloud_to_grid = cloud->cloudToGridMap();

    std::vector<float> blank(bins, 0.0f);

    boost::shared_ptr<std::vector<std::vector<float>> > histograms =
            boost::make_shared<std::vector<std::vector<float> > >(cloud.size(), blank);

    pcl::KdTreeFLANN<PointT> search;
    search.setInputCloud(typename pcl::PointCloud<PointT>::ConstPtr(&cloud, boost::serialization::null_deleter()));

    QTime total;
    total.start();

    int less_than_three_points_count = 0;

    std::vector<int> kIdxs;
    std::vector<float> kSqDist;

    float min = FLT_MAX;
    float max = FLT_MIN;

    qDebug() << "Here";

    // For every point
    for(unsigned int i = 0; i < cloud.size(); i++){
        kIdxs.clear();
        kSqDist.clear();

        search.radiusSearch(i, radius, kIdxs, kSqDist, max_nn);

        if(kIdxs.size() > max_nn && max_nn != 0){
            qDebug() << "Whoops! Too many";
            continue;
        }

        if(kIdxs.size() < 3) {
            less_than_three_points_count++;
            //(*histograms)[i] = Eigen::Vector3f(0, 0, 1.0f); // Assume isolated point
            continue;
        }

        /*
        // Calculate histogram dist
        float min = FLT_MAX;
        float max = FLT_MIN;
        float mean = 0;
        for(float & d : kDist) {
            if(d > max)
                max = d;
            if(d < min)
                min = d;
            mean+=d;
        }
        mean /= kDist.size();
        */

        float norm_increment = 1.0/kSqDist.size();

        for(float & d : kSqDist) {
            //int bin_idx = (((d-min)/(max-min)) * (bins-1));
            int bin_idx = (sqrt(d)/radius) * (bins-1);
            //qDebug() << "Loop start" << bin_idx << "Bins:" << bins;
            (*histograms)[i][bin_idx] += norm_increment;
            /*
            try{
                histograms->at(i).at(bin_idx) += norm_increment;
            } catch(...) {
                qDebug() << "Indices: " << i << bin_idx;
                qDebug() << "Other: " << d << min << max << ((d-min)/(max-min));
            }
            */

            //qDebug() << "Loop end" << bin_idx;
        }

        /*
        for(int j = 0; j < bins; j++){
            std::cout << (*histograms)[i][j] << " ";
        }
        std::cout << std::endl;
        fflush(stdout);
        */
    }

    qDebug() << "Radius: " << radius << " Max_nn: " << max_nn << " Time: " << total.elapsed()/1000.0f << "Sec";
    qDebug("Points with less than %d neighbours: %d", max_nn, less_than_three_points_count);
    qDebug("Max: %f, Min: %f", max, min);

    return histograms;
}

template <typename PointT>
boost::shared_ptr<std::vector<std::vector<float> > > calcIntensityHist(pcl::PointCloud<PointT> & cloud, int bins, double radius, uint max_nn) {

    //boost::shared_ptr<std::vector<float>> distmap = makeDistmap(cloud);
    //const std::vector<int> & cloud_to_grid = cloud->cloudToGridMap();

    std::vector<float> blank(bins, 0.0f);

    boost::shared_ptr<std::vector<std::vector<float>> > histograms =
            boost::make_shared<std::vector<std::vector<float> > >(cloud.size(), blank);

    pcl::KdTreeFLANN<PointT> search;
    search.setInputCloud(typename pcl::PointCloud<PointT>::ConstPtr(&cloud, boost::serialization::null_deleter()));

    QTime total;
    total.start();

    int less_than_three_points_count = 0;

    std::vector<int> kIdxs;
    std::vector<float> kSqDist;

    float min = FLT_MAX;
    float max = FLT_MIN;

    qDebug() << "Here";

    // For every point
    for(unsigned int i = 0; i < cloud.size(); i++){
        kIdxs.clear();
        kSqDist.clear();

        search.radiusSearch(i, radius, kIdxs, kSqDist, max_nn);

        if(kIdxs.size() > max_nn && max_nn != 0){
            qDebug() << "Whoops! Too many";
            continue;
        }

        if(kIdxs.size() < 3) {
            less_than_three_points_count++;
            //(*histograms)[i] = Eigen::Vector3f(0, 0, 1.0f); // Assume isolated point
            continue;
        }

        /*
        // Calculate histogram dist
        float min = FLT_MAX;
        float max = FLT_MIN;
        float mean = 0;
        for(float & d : kDist) {
            if(d > max)
                max = d;
            if(d < min)
                min = d;
            mean+=d;
        }
        mean /= kDist.size();
        */

        float norm_increment = 1.0/kSqDist.size();

        for(int & idx : kIdxs) {
            float & intensity = cloud[idx].intensity;
            int bin_idx = intensity * (bins-1);
            (*histograms)[i][bin_idx] += norm_increment;
        }

        /*
        for(int j = 0; j < bins; j++){
            std::cout << (*histograms)[i][j] << " ";
        }
        std::cout << std::endl;
        fflush(stdout);
        */
    }

    qDebug() << "Radius: " << radius << " Max_nn: " << max_nn << " Time: " << total.elapsed()/1000.0f << "Sec";
    qDebug("Points with less than %d neighbours: %d", max_nn, less_than_three_points_count);
    qDebug("Max: %f, Min: %f", max, min);

    return histograms;
}


#endif  // VISIUALISE_DEPTH_UTIL
