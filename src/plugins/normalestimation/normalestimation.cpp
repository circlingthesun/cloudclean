#include "plugins/normalestimation/normalestimation.h"
#include <cmath>
#include <QDebug>
#include <pcl/features/normal_3d.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "commands/select.h"
#include "pluginsystem/core.h"

#ifdef Q_OS_UNIX
    #include <GL/glx.h>
    #undef KeyPress  // Defined in X11/X.h, interferes with QEvent::KeyPress
#endif

#ifdef _MSC_VER
    #define INFINITY (DBL_MAX+DBL_MAX)
    #define NAN (INFINITY-INFINITY)
#endif

QString NormalEstimator::getName(){
    return "Normal estimation";
}

void NormalEstimator::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    premptive_estimation_ = true;

/*
    // Setup OpenCL
    cl_uint num_platforms;
    cl_platform_id clPlatformIDs[20];
    cl_platform_id platform = nullptr;
    char chBuffer[1024];
    cl_device_id device;

    cl_device_id devices[100];
    cl_uint devices_n = 0;

    cl_int result;


    result = clGetPlatformIDs (0, NULL, &num_platforms);

    if(num_platforms == 0) {
        qDebug("No OpenCL platform found!\n");
        return;
    }

    clGetPlatformIDs (num_platforms, clPlatformIDs, NULL);

    cl_uint max_freq = 0;

    qDebug() << "Availible CL Platforms:";
    for(uint i = 0; i < num_platforms && i < 20; ++i){
        result = clGetPlatformInfo(clPlatformIDs[i], CL_PLATFORM_VENDOR, 1024, &chBuffer, NULL);
        qDebug("platform %d: %s", i, chBuffer);

        //if(strstr(chBuffer, "NVIDIA") || strstr(chBuffer, "AMD")) {
        //    platform = clPlatformIDs[i];
        //    qDebug("Using platform: %s", chBuffer);
        //}

        result = clGetDeviceIDs(clPlatformIDs[i], CL_DEVICE_TYPE_GPU, 100, devices, &devices_n);

        qDebug("CL Devices on platform %s", chBuffer);
        for (uint i = 0; i < devices_n; i++) {
            char buffer[10240];
            char devicename[10240];
            cl_uint buf_uint;
            cl_ulong buf_ulong;

            qDebug("  -- %d --", i);
            clGetDeviceInfo(devices[i], CL_DEVICE_NAME, sizeof(devicename), devicename, NULL);
            qDebug("  DEVICE_NAME = %s", devicename);
            clGetDeviceInfo(devices[i], CL_DEVICE_VENDOR, sizeof(buffer), buffer, NULL);
            qDebug("  DEVICE_VENDOR = %s", buffer);
            clGetDeviceInfo(devices[i], CL_DEVICE_VERSION, sizeof(buffer), buffer, NULL);
            qDebug("  DEVICE_VERSION = %s", buffer);
            clGetDeviceInfo(devices[i], CL_DRIVER_VERSION, sizeof(buffer), buffer, NULL);
            qDebug("  DRIVER_VERSION = %s", buffer);
            clGetDeviceInfo(devices[i], CL_DEVICE_GLOBAL_MEM_SIZE, sizeof(buf_ulong), &buf_ulong, NULL);
            qDebug("  DEVICE_GLOBAL_MEM_SIZE = %llu\n", (unsigned long long)buf_ulong);
            clGetDeviceInfo(devices[i], CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(buf_uint), &buf_uint, NULL);
            qDebug("  DEVICE_MAX_COMPUTE_UNITS = %u", (unsigned int)buf_uint);

            clGetDeviceInfo(devices[i], CL_DEVICE_MAX_CLOCK_FREQUENCY, sizeof(buf_uint), &buf_uint, NULL);
            qDebug("  DEVICE_MAX_CLOCK_FREQUENCY = %u\n", (unsigned int)buf_uint);

            if(buf_uint > max_freq) {
                max_freq = buf_uint;
                device = devices[i];
                platform = clPlatformIDs[i];
                qDebug("Using device %s on platform: %s", devicename, chBuffer);
            }
        }


    }

    if(result != CL_SUCCESS){
        qDebug("Failed to obtain an availible device");
        qDebug() << "CL object create failed:" << oclErrorString(result);
        return;
    }

    #ifdef _WIN32
        HGLRC glCtx = wglGetCurrentContext();
    #else
        GLXContext glCtx = glXGetCurrentContext();
    #endif

    // http://www.codeproject.com/Articles/201263/Part-6-Primitive-Restart-and-OpenGL-Interoperabili
    cl_context_properties props[] = {
            CL_CONTEXT_PLATFORM,
            (cl_context_properties) platform,
        #ifdef _WIN32
            CL_WGL_HDC_KHR, (intptr_t) wglGetCurrentDC(),
        #else
            CL_GLX_DISPLAY_KHR, (intptr_t) glXGetCurrentDisplay(),
        #endif
            CL_GL_CONTEXT_KHR, (intptr_t) glCtx, 0
    };

    clcontext = clCreateContext(props, 1, &device, NULL, NULL, NULL);
    cmd_queue = clCreateCommandQueue(clcontext, device, 0, NULL);
*/

    connect(cl_, SIGNAL(cloudUpdate(boost::shared_ptr<PointCloud>)),
            this, SLOT(addedCloud(boost::shared_ptr<PointCloud>)));

    connect(cl_, SIGNAL(deletingCloud(boost::shared_ptr<PointCloud>)),
            this, SLOT(removingCloud(boost::shared_ptr<PointCloud>)));

}

void NormalEstimator::cleanup(){
    disconnect(cl_, SIGNAL(cloudUpdate(boost::shared_ptr<PointCloud>)),
            this, SLOT(addedCloud(boost::shared_ptr<PointCloud>)));
    disconnect(cl_, SIGNAL(deletingCloud(boost::shared_ptr<PointCloud>)),
            this, SLOT(removingCloud(boost::shared_ptr<PointCloud>)));
}

void NormalEstimator::enablePremptiveEstimation(bool enable) {
    premptive_estimation_ = enable;
    if(enable){
        // TODO(Rickert): start estimation
        // for each cloud in not in normals
        //  or in futures
        for(auto it : normal_map_){

        }
    }
}

pcl::PointCloud<pcl::Normal>::Ptr NormalEstimator::getNormals(boost::shared_ptr<PointCloud> cloud) {
    boost::weak_ptr<PointCloud> wcloud = cloud;
    auto it = normal_map_.find(wcloud);
    if (it != normal_map_.end())
        return it->second;

    auto it2 = future_normal_map_.find(wcloud);
    if(it2 != future_normal_map_.end()){
        std::future<pcl::PointCloud<pcl::Normal>::Ptr> & fut = it2->second;
        normal_map_[wcloud] = fut.get();
        future_normal_map_.erase(it2);
        return normal_map_[wcloud];
    }

    normal_map_[wcloud] = estimateNormals(cloud);
    return normal_map_[wcloud];
}

bool NormalEstimator::normalsAvailible(boost::shared_ptr<PointCloud> cloud) {
    boost::weak_ptr<PointCloud> wcloud = cloud;
    auto it = normal_map_.find(wcloud);
    if (it != normal_map_.end())
        return true;

    auto it2 = future_normal_map_.find(wcloud);
    if(it2 != future_normal_map_.end()){
        std::future<pcl::PointCloud<pcl::Normal>::Ptr> & fut = it2->second;
        return fut.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    }
    return false;
}

void NormalEstimator::addedCloud(boost::shared_ptr<PointCloud> cloud) {
    if(!premptive_estimation_)
        return;
    boost::weak_ptr<PointCloud> wpc = cloud;
    future_normal_map_[wpc] = std::async(std::launch::async, &NormalEstimator::estimateNormals, this, cloud);
}

void NormalEstimator::removingCloud(boost::shared_ptr<PointCloud> cloud) {
    // Todo stop future calc
    // delete data
    boost::weak_ptr<PointCloud> wpc = cloud;
    const auto it = normal_map_.find(wpc); // TODO: Bug! Freezes on delete
    if(it != normal_map_.end()) {
        normal_map_.erase(it);
        return;
    }

    auto it2 = future_normal_map_.find(wpc);
    if(it2 != future_normal_map_.end()){
        std::future<pcl::PointCloud<pcl::Normal>::Ptr> & fut = it2->second;

        auto async_del_future = [&fut, &wpc, this] () {
            if(fut.valid())
                fut.get(); // Once ref count drops to 0 it should delete

            //auto it2 = future_normal_map_.find(wpc);
            // TODO: Bad future in map
            //if(it2 != future_normal_map_.end())
            //    future_normal_map_.erase(it2);
        };

        std::thread(async_del_future).detach();
        return;
    }

}

class PointIdx {
 public:
    int x;
    int y;
    PointIdx(int x, int y): x(x), y(y) {}
    PointIdx(): x(0), y(0) {}
};

class PointIdxPair {
 public:
    PointIdx p1;
    PointIdx p2;
    PointIdxPair(PointIdx p1, PointIdx p2): p1(p1), p2(p2) {}
    PointIdxPair(int p1x, int p1y, int p2x, int p2y)
        :p1(PointIdx(p1x, p1y)), p2(PointIdx(p2x, p2y)) {}
};

bool isValidPoint(PointIdx & point, int width, int height,
                  int index, std::vector<int> & grid_to_cloud) {
    // calculate current x and y
    int cur_x = index%width;
    int cur_y = index/width;

    // Bounds check
    int x_pos = cur_x + point.x;
    int y_pos = cur_y + point.y;
    if (x_pos > width-1 || x_pos < 0)
        return false;
    if (y_pos > height-1 || y_pos < 0)
        return false;

    // Index with NaN value is not valid
    int idx = y_pos*width + x_pos;
    if (grid_to_cloud[idx] == -1)
        return false;

    return true;
}

pcl::PointCloud<pcl::Normal>::Ptr
NormalEstimator::estimateNormals(boost::shared_ptr<PointCloud> cloud) {

    qDebug() << "Starting normal estimation";

    // New normals datastructure
    pcl::PointCloud<pcl::Normal>::Ptr normals;
    normals.reset(new pcl::PointCloud<pcl::Normal>());
    normals->resize(cloud->points.size());

    // Temporary data structure to save mapping from grid to cloud
    std::vector<int> grid_to_cloud;
    grid_to_cloud.resize(cloud->scan_width()*cloud->scan_height(), -1);
    for(uint i = 0; i < cloud->cloudToGridMap().size(); i++){
        grid_to_cloud[cloud->cloudToGridMap()[i]] = i;
    }

    // Keeps track of NAN indices
    std::vector<int> missing_normals;

    int height = cloud->scan_width();
    int width = cloud->scan_height();
    int length = width*height;

    // Relative point anticlockwise around (0,0)
    PointIdx pointList[8] = {
        PointIdx(1, 0),
        PointIdx(1, -1),
        PointIdx(0, -1),
        PointIdx(-1, -1),
        PointIdx(-1, 0),
        PointIdx(-1, 1),
        PointIdx(0, 1),
        PointIdx(1, 1)
    };

    for (int i = 0; i < length; ++i) {
        if(cloud->deleting_)
            return normals;

        // Skip values that dont exist
        if (grid_to_cloud[i] == -1) {
            continue;
        }

        // Estimation variables
        Eigen::Vector3f agregate_n(0.0f, 0.0f, 0.0f);
        int face_count = 0;

        // Calculate normal from 8 triangles
        int next = 0;
        int first = -1;
        while (true) {
            // Find valid indices around 0,0
            int p1idx = -1, p2idx = -1;

            while (next < 8) {
                if (isValidPoint(pointList[next], width, height, i, grid_to_cloud)) {
                    p1idx = next++;
                    break;
                }
                next++;
            }

            while (next < 9) {
                // Don't want two of the same indices in different orders
                if (next == 8 && face_count == 1)
                    break;

                // Loop around
                if (next == 8 && face_count > 1) {
                    p2idx = first;
                    break;
                }

                int idx = next % 8;
                if (isValidPoint(pointList[idx], width, height, i, grid_to_cloud)) {
                    p2idx = idx;
                    break;
                }
                next++;
            }

            // Stop if no points in range were found
            if (p1idx == -1 || p2idx == -1)
                break;

            if (first == -1)
                first = p1idx;

            PointIdxPair relativePair(pointList[p1idx], pointList[p2idx]);

            // calculate current x and y
            int cur_x = i%width;
            int cur_y = i/width;

            // Calculate absolute indices
            int idx1 = (cur_y+relativePair.p1.y)*width
                    + (cur_x+relativePair.p1.x);
            int idx2 = (cur_y+relativePair.p2.y)*width
                    + (cur_x+relativePair.p2.x);


            Eigen::Map<Eigen::Vector3f> p0 = cloud->points[grid_to_cloud[i]].getVector3fMap();
            Eigen::Map<Eigen::Vector3f> p1 = cloud->points[grid_to_cloud[idx1]].getVector3fMap();
            Eigen::Map<Eigen::Vector3f> p2 = cloud->points[grid_to_cloud[idx2]].getVector3fMap();

            // Cross product to get normal
            Eigen::Vector3f vec1 = p1 - p0;
            Eigen::Vector3f vec2 = p2 - p0;
            Eigen::Vector3f tmp = vec1.cross(vec2).normalized();

            // Normalized 0 0 0 == NaN
            if (tmp.x() != tmp.x() ||
               tmp.y() != tmp.y() ||
               tmp.z() != tmp.z()
            )
                continue;

            agregate_n += tmp;

            face_count++;
        }

        pcl::Normal & normal_ref = normals->points[grid_to_cloud[i]];

        // TESTING REMOVE THIS:

//        normal_ref.data_n[0] = 1.0f;
//        normal_ref.data_n[1] = 0.0f;
//        normal_ref.data_n[2] = 0.0f;
//        normal_ref.data_n[3] = 0.0f;
//        continue;

        //////////////////////

        // Face count can be 0
        // Should set to point to camera
        if (face_count == 0) {
            // TODO(Rickert) : Set normal to the value of a neighbour
            missing_normals.push_back(grid_to_cloud[i]);
            normal_ref.data_n[0] = NAN;
            normal_ref.data_n[1] = NAN;
            normal_ref.data_n[2] = NAN;
            normal_ref.data_n[3] = NAN;
            continue;
        }

        Eigen::Vector3f normal = agregate_n.normalized();
        //pcl::flipNormalTowardsViewpoint(cloud->points[grid_to_cloud[i]], 0, 0, 0, normal);

        normal_ref.data_n[0] = normal(0);
        normal_ref.data_n[1] = normal(1);
        normal_ref.data_n[2] = normal(2);
        normal_ref.data_n[3] = 1.0f;

    }

    // Deal with missing normals here

    pcl::KdTreeFLANN<pcl::PointXYZI> search;
    search.setInputCloud(cloud);
    int k = 50;

    int missing = missing_normals.size();

    qDebug() << "Normals WAS missing" << missing;

    for(int idx :  missing_normals){
        std::vector<int> neighbours;
        std::vector<float> dists;
        search.nearestKSearch(idx, k, neighbours, dists);

        Eigen::Vector3f sum(0, 0, 0);

        int count = 0;

        for(int n : neighbours) {
            Eigen::Map<Eigen::Vector3f> nb = (*normals)[n].getNormalVector3fMap();
            if(nb[0] != nb[0])
                continue;

            ++count;
            sum += nb;
        }

        if(count == 0)
            continue;

        (*normals)[idx].getNormalVector3fMap() = sum / count;
        --missing;
    }


    // Check if result is still needed

    qDebug() << "Completed normal estimation";
    qDebug() << "Normals missing" << missing;
    qDebug() << " % Normals missing" <<
                missing/(float)normals->size();

    return normals;
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin")
