/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef GRID_SEARCH_H_
#define GRID_SEARCH_H_

#include <pcl/point_cloud.h>
#include <pcl/for_each_type.h>
#include <pcl/common/concatenate.h>
#include <pcl/search/search.h>
#include "model/pointcloud.h"
#include <boost/serialization/shared_ptr.hpp>

typedef PointCloud CCPointCloud;

/** \brief Generic search class. All search wrappers must inherit from this.
  *
  * Each search method must implement 2 different types of search:
  *   - \b nearestKSearch - search for K-nearest neighbors.
  *   - \b radiusSearch - search for all nearest neighbors in a sphere of a given radius
  *
  * The input to each search method can be given in 3 different ways:
  *   - as a query point
  *   - as a (cloud, index) pair
  *   - as an index
  *
  * For the latter option, it is assumed that the user specified the input
  * via a \ref setInputCloud () method first.
  *
  * \note In case of an error, all methods are supposed to return 0 as the number of neighbors found.
  *
  * \note libpcl_search deals with three-dimensional search problems. For higher
  * level dimensional search, please refer to the libpcl_kdtree module.
  *
  * \author Radu B. Rusu
  * \ingroup search
  */
class GridSearch : public pcl::search::Search<pcl::PointXYZI>
{
  public:

    /** Constructor. */
    GridSearch (const std::string& name = "", bool sorted = false)
        : Search(name, sorted)
    {
    }

    GridSearch (CCPointCloud & cloud, const std::string& name = "", bool sorted = false)
      : Search(name, sorted)
    {
        input_  = pcl::PointCloud<pcl::PointXYZI>::ConstPtr(&cloud, boost::serialization::null_deleter());
    }

    /** Destructor. */
    virtual
    ~GridSearch ()
    {
    }

    /** \brief Search for the k-nearest neighbors for the given query point.
      * \param[in] point the given query point
      * \param[in] k the number of neighbors to search for
      * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
      * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
      * a priori!)
      * \return number of neighbors found
      */
    virtual int
    nearestKSearch (const pcl::PointXYZI &point, int k, std::vector<int> &k_indices,
                    std::vector<float> &k_sqr_distances) const {

        const CCPointCloud * cloud = dynamic_cast<const CCPointCloud *>(input_.get());
        assert(cloud != nullptr && "Not of right type Pointcloud");

        k_indices.clear();
        k_sqr_distances.clear();

        int idx = point.data_c[3];

        int h = cloud->scan_height();
        int w = cloud->scan_width();

        boost::shared_ptr<const std::vector<int>> grid_to_cloud = cloud->gridToCloudMap();

        int grid_idx = cloud->cloudToGridMap()[idx];
        int x = grid_idx / h;
        int y = grid_idx % h;

        Eigen::Map<const Eigen::Vector3f> query_point(&cloud->points[idx].x, 3);

        const int max_ring_size = (w>h?w:h)/2;

        for(int ring = 1; ring <= max_ring_size ; ring++){
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

                    // Only consider valid indices
                    if(idx  != -1){
                        const float * data = &(cloud->points[idx].x);
                        Eigen::Map<const Eigen::Vector3f> neighbour(data, 3);
                        float sqdist = (neighbour-query_point).squaredNorm();

                        k_indices.push_back(idx);
                        k_sqr_distances.push_back(sqdist);
                        if(k_indices.size() == uint(k))
                            return k_indices.size();
                    }

                    // Skip the inner values
                    if(iy != -ring && iy != ring && ix == -ring) {
                        ix = ring-1;
                    }
                }
            }
        }

        return k_indices.size();
    }


    /** \brief Search for all the nearest neighbors of the query point in a given radius.
      * \param[in] point the given query point
      * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
      * \param[out] k_indices the resultant indices of the neighboring points
      * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
      * \param[in] max_nn if given, bounds the maximum returned neighbors to this value. If \a max_nn is set to
      * 0 or to a number higher than the number of points in the input cloud, all neighbors in \a radius will be
      * returned.
      * \return number of neighbors found in radius
      */
    virtual int
    radiusSearch (const pcl::PointXYZI& point, double radius, std::vector<int>& k_indices,
                  std::vector<float>& k_sqr_distances, unsigned int max_nn = 0) const {

        const CCPointCloud * cloud = dynamic_cast<const CCPointCloud *>(input_.get());
        assert(cloud != nullptr && "Not of right type Pointcloud");

        k_indices.clear();
        k_sqr_distances.clear();

        int h = cloud->scan_height();
        int w = cloud->scan_width();

        boost::shared_ptr<const std::vector<int>> grid_to_cloud = cloud->gridToCloudMap();

        //int idx = point.data_c[3];
        int grid_idx = point.data_c[2];

        int x = grid_idx / h;
        int y = grid_idx % h;

        //Eigen::Map<const Eigen::Vector3f> query_point(&cloud->points[idx].x, 3);
        Eigen::Vector3f qp(point.x, point.y, point.z);

        const double rad_sq = radius * radius;
        const int max_ring_size = (w>h?w:h)/2;
        int points_outside_radius = 0;

        for(int ring = 1; ring <= max_ring_size ; ring++){
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

                    // Only consider valid indices
                    if(idx  != -1){
                        const float * data = &(cloud->points[idx].x);
                        Eigen::Map<const Eigen::Vector3f> neighbour(data, 3);
                        float sqdist = (qp - neighbour).squaredNorm();

                        if(sqdist <= rad_sq) {
                            k_indices.push_back(idx);
                            k_sqr_distances.push_back(sqdist);
                            qDebug() << "Yay";
                            if(max_nn != 0 && k_indices.size() > max_nn)
                                return k_indices.size();

                        } else {
                            points_outside_radius++;
                        }

                        int side_len = (ring*2+1);
                        int max_err = side_len*2 + 2*(side_len-2);

                        // If error is more than the indices in a ring
                        if(points_outside_radius > max_err){
                            return k_indices.size();
                        }
                    }

                    // Skip the inner values
                    if(iy != -ring && iy != ring && ix == -ring) {
                        ix = ring-1;
                    }
                }
            }
        }

        return k_indices.size();
    }


}; // class GridSearch

#endif  // GRID_SEARCH_H_
