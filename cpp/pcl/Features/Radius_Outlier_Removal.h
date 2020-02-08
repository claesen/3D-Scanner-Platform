#ifndef PCL_RADIUS_OUTLIER_REMOVAL_H
#define PCL_RADIUS_OUTLIER_REMOVAL_H

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>


pcl::PointCloud<pcl::PointXYZRGBA>::Ptr radius_filter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

#endif //PCL_RADIUS_OUTLIER_REMOVAL_H
