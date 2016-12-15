#ifndef ULTILITIES_UTILS_H
#define ULTILITIES_UTILS_H

#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/octree/octree_container.h>
#if _WIN32
    #include <pcl/octree/impl/octree_iterator.hpp>
#endif
#include <boost/serialization/shared_ptr.hpp>
#include "utilities/export.h"

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr octreeDownsample(
        pcl::PointCloud<PointT> * input,
        float resolution,
        std::vector<int> & sub_idxs) {

    size_t data_items = sizeof(PointT)/sizeof(float);

    typename pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>());
    sub_idxs.resize(input->size(), 0);

    typename pcl::PointCloud<PointT>::Ptr ptr(input, boost::serialization::null_deleter());

    typename pcl::octree::OctreePointCloud<PointT> octree1(resolution);
    octree1.setInputCloud (ptr);
    octree1.addPointsFromInputCloud();

    typename pcl::octree::OctreePointCloud<PointT>::LeafNodeIterator it1;
    typename pcl::octree::OctreePointCloud<PointT>::LeafNodeIterator it1_end = octree1.leaf_end();

    unsigned int leafNodeCounter = 0;

    for (it1 = octree1.leaf_begin(); it1 != it1_end; ++it1) {
        std::vector<int> & indices = it1.getLeafContainer().getPointIndicesVector();

        PointT p;
        Eigen::Map<Eigen::VectorXf> pmap = Eigen::VectorXf::Map(&p.data[0], data_items);

        for(int idx : indices){


            Eigen::Map<Eigen::VectorXf> pmap1 = Eigen::VectorXf::Map(reinterpret_cast<float *>(&((*input)[idx])), data_items);
            pmap += pmap1;
            sub_idxs[idx] = output->size();

        }

        float size_inv = 1.0/indices.size();

        pmap*=size_inv;
        output->push_back(p);

        leafNodeCounter++;
    }

    return output;
}

#undef small
#ifdef small
#error "small defined"
#endif
#ifdef big
#error "big defined"
#endif
#ifdef map
#error "map defined"
#endif
template <typename T1, typename T2>
void map_small_to_big(
        std::vector<T1, T2> & small_,
                      std::vector<T1, T2>& big_,
                      std::vector<int> & map_){

    big_.resize(map_.size());
    for(unsigned int i = 0; i < map_.size(); i++) {
        int small_idx = map_[i];

        memcpy(&big_[i], &small_[small_idx], sizeof(T1));
    }
}


UTIL_API pcl::PointCloud<pcl::PointXYZINormal>::Ptr zipNormals(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
        pcl::PointCloud<pcl::Normal>::Ptr normals);

#endif  // ULTILITIES_UTILS_H
