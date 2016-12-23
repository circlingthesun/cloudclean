#ifndef NORMAL_ESITIMATION_H
#define NORMAL_ESITIMATION_H

#include "pluginsystem/iplugin.h"

#include <memory>
#include <map>
#include <unordered_map>
#include <future>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/weak_ptr.hpp>
#include <boost/smart_ptr/owner_less.hpp>
#include "plugins/normalestimation/export.h"
#include "pluginsystem/pluginmanager.h"

/*
#include "glheaders.h"
#ifdef __APPLE__
    #include <OpenCL/opencl.h>
#else
    #include <CL/cl.h>
#endif
#include <CL/cl_gl.h>
*/

template<typename T>
struct WeakPtrHash : public std::unary_function<boost::weak_ptr<T>, size_t> {
   size_t operator()(const boost::weak_ptr<T>& wp) const
   {
      auto sp = wp.lock();
      return std::hash<decltype(sp)>()(sp);
   }
};

template<typename T>
struct WeakPtrEqual : public std::unary_function<boost::weak_ptr<T>, bool> {

   bool operator()(const boost::weak_ptr<T>& left, const boost::weak_ptr<T>& right) const
   {
      return !left.owner_before(right) && !right.owner_before(left);
   }
};

class Core;
class CloudList;
class QUndoStack;
class PointCloud;

typedef std::map<boost::weak_ptr<PointCloud>, pcl::PointCloud<pcl::Normal>::Ptr,
    boost::owner_less<boost::weak_ptr<PointCloud>>> NormalMap;

//typedef std::unordered_map<boost::weak_ptr<PointCloud>, pcl::PointCloud<pcl::Normal>::Ptr, WeakPtrHash<PointCloud>, WeakPtrEqual<PointCloud>> NormalMap;

typedef std::map<boost::weak_ptr<PointCloud>,
    std::future<pcl::PointCloud<pcl::Normal>::Ptr>,
    boost::owner_less<boost::weak_ptr<PointCloud>>> FutureNormalMap;

class NE_API NormalEstimator : public IPlugin {
    Q_INTERFACES(IPlugin)
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin" FILE "normalestimation.json")
 public:
    QString getName();
    void initialize(Core * core);
    void cleanup();
    void enablePremptiveEstimation(bool enable);
    pcl::PointCloud<pcl::Normal>::Ptr getNormals(boost::shared_ptr<PointCloud> cloud);
    bool normalsAvailible(boost::shared_ptr<PointCloud> cloud);

 public slots:
    void addedCloud(boost::shared_ptr<PointCloud> cloud);
    void removingCloud(boost::shared_ptr<PointCloud> cloud);

 private:
    pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(boost::shared_ptr<PointCloud> cloud);

 private:
    bool premptive_estimation_;

    Core * core_;
    CloudList * cl_;
    NormalMap normal_map_;
    FutureNormalMap future_normal_map_;

    //cl_context clcontext;
    //cl_command_queue cmd_queue;

};

#endif  // NORMAL_ESITIMATION_H
