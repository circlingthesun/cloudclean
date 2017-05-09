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

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr octreeDownsample(
        pcl::PointCloud<pcl::PointXYZRGBNormal> * input,
        float resolution,
        std::vector<int> & sub_idxs) {

    size_t data_items = sizeof(pcl::PointXYZRGBNormal)/sizeof(float);

    typename pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    sub_idxs.resize(input->size(), 0);

    typename pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ptr(input, boost::serialization::null_deleter());

    typename pcl::octree::OctreePointCloud<pcl::PointXYZRGBNormal> octree1(resolution);
    octree1.setInputCloud (ptr);
    octree1.addPointsFromInputCloud();

    typename pcl::octree::OctreePointCloud<pcl::PointXYZRGBNormal>::LeafNodeIterator it1;
    typename pcl::octree::OctreePointCloud<pcl::PointXYZRGBNormal>::LeafNodeIterator it1_end = octree1.leaf_end();

    unsigned int leafNodeCounter = 0;

    for (it1 = octree1.leaf_begin(); it1 != it1_end; ++it1) {
        std::vector<int> & indices = it1.getLeafContainer().getPointIndicesVector();

        int r = 0, g = 0, b = 0;
        float x = 0.0f, y = 0.0f, z = 0.0f, intensity = 0.0f, nx = 0.0f, ny = 0.0f, nz = 0.0f;

        for(int idx : indices){

            x += (*input)[idx].x;
            y += (*input)[idx].y;
            z += (*input)[idx].z;
            intensity += (*input)[idx].data[3];
            nx += (*input)[idx].normal_x;
            ny += (*input)[idx].normal_y;
            nz += (*input)[idx].normal_z;
            r += (*input)[idx].r;
            g += (*input)[idx].g;
            b += (*input)[idx].b;

            sub_idxs[idx] = output->size();
        }

        float size_inv = 1.0/indices.size();

        pcl::PointXYZRGBNormal p;

        p.x = x*=size_inv;
        p.y = y*=size_inv;
        p.z = z*=size_inv;
        p.data[3] = intensity*=size_inv;
        p.normal_x = nx*=size_inv;
        p.normal_y = ny*=size_inv;
        p.normal_z = nz*=size_inv;
        float t = sqrt(nx*nx + ny*ny + nz*nz);
        p.normal_x /= t;
        p.normal_y /= t;
        p.normal_z /= t;
        p.r = uint8_t(r*=size_inv);
        p.g = uint8_t(g*=size_inv);
        p.b = uint8_t(b*=size_inv);

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


UTIL_API pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr zipNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
        pcl::PointCloud<pcl::Normal>::Ptr normals);

#endif  // ULTILITIES_UTILS_H
