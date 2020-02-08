#include <ros/ros.h>
#include "std_msgs/String.h"
#include <string>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "geometry_msgs/Point32.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/Color.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>


// Neat struct to hold all topic name definitions in one
struct Topics_Config {
    std::string buffered_point_topic;
    std::string processed_pointcloud_topic;
};

class lidar_processor {

public:

    // Constructor & Destructor of node class
    lidar_processor(ros::NodeHandle& nh, Topics_Config topics_config);
    ~lidar_processor();

private:
    // Node handle instance:
    // Exactly what it sounds like. Is used as a communication medium between the node class and ros core
    ros::NodeHandle nh_;

    // Initialization functions
    void initialize_subscribers();
    void initialize_publishers();

    // All functions run when callback is executed, in correct execution order:
    void callback_point_collection(const geometry_msgs::Point32::ConstPtr &point_msg);
    void buffer_point(geometry_msgs::Point32::ConstPtr point_msg);
    bool ready_to_run();
    void execute();
    bool populate_with_buffered_cloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_msg);
    void publish(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_msg);

    pcl::PointCloud<pcl::PointXYZ> *buffered_point_cloud;

    std::string world_frame = "map";

    Topics_Config topics_config_;

    ros::Subscriber point_sub; // Dont actually need to hold on to this reference as it is stored by the nodehandle
    ros::Publisher point_cloud_pub;


};
