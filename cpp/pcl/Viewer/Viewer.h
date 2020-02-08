#ifndef PCL_VIEWER_H
#define PCL_VIEWER_H

#include <iostream>
#include <pcl/visualization/cloud_viewer.h>

extern int user_data;

void viewerPsycho(pcl::visualization::PCLVisualizer &viewer);
void viewerOneOff(pcl::visualization::PCLVisualizer &viewer);

#endif //PCL_VIEWER_H
