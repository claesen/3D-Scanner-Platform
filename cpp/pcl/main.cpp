#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include "Viewer/Viewer.h"
#include "IO/Load_From_txt.h"
#include "Features/Radius_Outlier_Removal.h"

using namespace std ;

int
main (int argc, char** argv)
{
    // If 2nd arg is 'l' load some static code which reads from "x.txt", "y.txt" and "z.txt"
    cout << argc << " + " << argv[0] << " + " << argv[1] << endl;
    if (argc > 1 && *argv[1] == 'l') 
    {
        Load_From_TXT();
    }


    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile ("big.pcd", *cloud);

    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    //blocks until the cloud is actually rendered
    viewer.showCloud(cloud);

    //use the following functions to get access to the underlying more advanced/powerful
    //PCLVisualizer

    //This will only get called once
    viewer.runOnVisualizationThreadOnce (viewerOneOff);

    //This will get called once per visualization iteration
    viewer.runOnVisualizationThread (viewerPsycho);

    // MAIN LOOP OF EVERYTHING
    while (!viewer.wasStopped ())
    {
        //you can also do cool processing here
        radius_filter(cloud);
        //FIXME: Note that this is running in a separate thread from viewerPsycho
        //and you should guard against race conditions yourself...
        user_data++;
    }

    return (0);
}