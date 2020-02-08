
#include "Viewer.h"

using namespace std;

int user_data;

// This function runs once in the start of the visualization
void viewerOneOff(pcl::visualization::PCLVisualizer &viewer) {

    // Set meta stuff
    viewer.setWindowName("LiDAR 3D-Scanner PCL Visual");

    // Set Point-Cloud Stuff
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

    // Set Camera Stuff
    viewer.initCameraParameters();
    viewer.setCameraPosition(0, 50, 50,    0, 0, 0,   0, 0, 1);
    viewer.setCameraClipDistances(0.00522511, 50);

    std::cout << "i only run once" << std::endl;

}

// This function runs every iteration of the visualization
void viewerPsycho(pcl::visualization::PCLVisualizer &viewer) {

    //FIXME: possible race condition here:
    user_data++;
}


