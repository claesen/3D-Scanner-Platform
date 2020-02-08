#include "lidar_processing/lidar_processing.h"

// Main function of node:
// Is run when roslaunch is run.
// Initializes the node class
//
//

int main(int argc, char** argv)
{
    // Inits a ros node with name lidar_processor
    ros::init(argc, argv, "lidar_processor");

    // Instantiates a node handle for ros core communication (Subscription, publication etc)
    ros::NodeHandle nh;

    // A struct which holds all the topic names for all subs/pubs
    Topics_Config topics_config;
    topics_config.buffered_point_topic = "/USBCommunicator/Point";
    topics_config.processed_pointcloud_topic = "/LidarProcessor/PointCloud";

    // Instantiates the node class
    lidar_processor lp = lidar_processor(nh, topics_config);

    // Tells roscore to run, iteratively processes all publications etc.
    ros::spin();

    return 0;
}


lidar_processor::lidar_processor(ros::NodeHandle& nh,
                                        Topics_Config topics_config):
                                        nh_(nh), topics_config_(topics_config),
                                        buffered_point_cloud()   // <- magic initalization
{
    // Inits the internal point cloud buffer:
    buffered_point_cloud = new pcl::PointCloud<pcl::PointXYZ>;

    initialize_subscribers();
    initialize_publishers();
}


lidar_processor::~lidar_processor() = default; // Default destructor, not too sure this is a good idea, should probably run nh.destroy here


void lidar_processor::initialize_subscribers(){
    // Sets up a subscriber to
    // - "buffered_point_topic" topic with
    // - "1000" in message buffer size, and
    // - "callback_point_collection" as callback function for each message received, and
    // - "this" as namespace (i think)
    point_sub =  nh_.subscribe(topics_config_.buffered_point_topic,
                                        1000,
                                        &lidar_processor::callback_point_collection,
                                        this) ;
}


void lidar_processor::initialize_publishers(){
    // Sets up a publisher with messages of type
    // - "PointCloud<pcl::PointXYZRGBA>", to the topic
    // - "processed_pointcloud_topic", and
    // - "1000" in message buffer size
    point_cloud_pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGBA>>(topics_config_.processed_pointcloud_topic, 1000);
}


void lidar_processor::buffer_point(geometry_msgs::Point32::ConstPtr point_msg)
{
    buffered_point_cloud->push_back(pcl::PointXYZ(point_msg->x, point_msg->y, point_msg->z));
}


bool lidar_processor::ready_to_run()
{
    // Returns whether the node is ready to start execution/processing
    return true;
}

void lidar_processor::execute()
{

}

bool lidar_processor::populate_with_buffered_cloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_msg)
{
    // Populates the input message pointer with the current buffered cloud

    // Defines the frame in which the point is defined in reference to, i.e. the origo in reference to each points coordinate
    cloud_msg->header.frame_id = world_frame;

    // Converts all points one by one to format which rviz can handle:
    for (auto &point : buffered_point_cloud->points){
        pcl::PointXYZRGBA point_rgba;
        point_rgba.x = point.x;
        point_rgba.y = point.y;
        point_rgba.z = point.z;
        point_rgba.rgba =
        point_rgba.r = 255;
        point_rgba.g = 255;
        point_rgba.b = 255;
        point_rgba.a = 255;
        cloud_msg->points.push_back(point_rgba);
    }
    return true;
}

void lidar_processor::publish(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_msg)
{
    // Publishes the input message with the previously constructed publisher
    point_cloud_pub.publish(*cloud_msg);
}

void lidar_processor::callback_point_collection(const geometry_msgs::Point32::ConstPtr &point_msg)
{
    // Callback function for when a message is published on the "buffered_point_topic" by the usb_communicator node
    // This is the only entry-point for exectution after the node is initialized, i.e. this is where all continous processing has to be executed as well.

    // First add the new point to the internal cloud buffer:
    buffer_point(point_msg);

    // Check if the node is ready for execution/further processing
    // This could for example be when a new full lap is completed
    if (ready_to_run())
    {
        // Excecute any internal processing:
        execute();

        // Publish clouds for visualization, in this case the entire internally buffered cloud:
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_msg(new pcl::PointCloud<pcl::PointXYZRGBA>);

        if (populate_with_buffered_cloud(cloud_msg))
        {
            publish(cloud_msg);
        }
    }
}


