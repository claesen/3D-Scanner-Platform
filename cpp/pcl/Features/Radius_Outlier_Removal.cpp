#include "Radius_Outlier_Removal.h"


pcl::PointCloud<pcl::PointXYZRGBA>::Ptr radius_filter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> outrem;
// build the filter
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.8);
    outrem.setMinNeighborsInRadius(2);
// apply filter
    outrem.filter(*cloud_filtered);
    return cloud_filtered;

}