#include <ros/ros.h>
#include "std_msgs/String.h"
#include <string>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "geometry_msgs/Point32.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <visualization_msgs/Marker.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

// Neat struct to hold all topic name definitions in one
struct Topics_Config
{
    std::string processed_pointcloud_topic;
    std::string inliers_pointcloud_topic;
    std::string ransac_plane_topic;
    std::string histogram_inliers_topic;
};

class lidar_processor
{

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
        void callback_point_collection(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &point_msg);
        void buffer_point(geometry_msgs::Point32::ConstPtr point_msg);
        bool ready_to_run();
        void execute();
        void publish_inliers();
        void publish_OGCLOUD();
        void publish_plane();
        void publish_histogram_inliers();
        void populate_histogram_filtering_msg(std::vector<int>* inlier_idxs);
        void populate_inliers_msg(pcl::PointIndices::Ptr inlier_idxs);
        void populate_OGCLOUD_msg(pcl::PointCloud<pcl::PointXYZ>::Ptr OGCLOUD);
        void populate_plane_msg(pcl::ModelCoefficients::Ptr plane_coefs);
        void convert_back();

        void histogram_filtering(pcl::PointCloud<pcl::PointXYZ>* cloud, std::vector<int>* inlier_indexes, int n_bins, double min_dist, int density_threshold, double height_diff_thresh);

        pcl::PointCloud<pcl::PointXYZ> *buffered_point_cloud;
        pcl::PointCloud<pcl::PointXYZRGBA> *cached_message;
        pcl::PointCloud<pcl::PointXYZRGBA> *histogram_inliers_msg;
        pcl::PointCloud<pcl::PointXYZRGBA> *inliers_msg;
        pcl::PointCloud<pcl::PointXYZRGBA> *OGCLOUD_msg;
        visualization_msgs::Marker *plane_msg;

        std::string world_frame = "map";
        bool msg_buffered = false;

        Topics_Config topics_config_;

        ros::Subscriber point_sub; // Dont actually need to hold on to this reference as it is stored by the nodehandle
        ros::Publisher histogram_inliers_pub;
        ros::Publisher inliers_pub;
        ros::Publisher OGCLOUD_pub;
        ros::Publisher plane_pub;
};
