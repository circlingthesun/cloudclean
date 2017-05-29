#include "utilities/utils.h"

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr zipNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
        pcl::PointCloud<pcl::Normal>::Ptr normals){

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr zipped (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    zipped->resize(cloud->size());

    for(uint i = 0; i < cloud->size(); i ++){
        pcl::PointXYZRGB & p = (*cloud)[i];
        pcl::Normal & n = (*normals)[i];

        pcl::PointXYZRGBNormal & pn = (*zipped)[i];
        pn.getNormalVector4fMap() = n.getNormalVector4fMap();
        pn.getVector4fMap() = p.getVector4fMap();
        pn.r = p.r;
        pn.g = p.g;
        pn.b = p.b;
    }

    return zipped;
}
