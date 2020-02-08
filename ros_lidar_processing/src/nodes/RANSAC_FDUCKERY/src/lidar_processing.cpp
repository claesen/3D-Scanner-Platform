#include "lidar_processing/lidar_processing.h"

// Main function of the processing node:
// Initializes the node class
//
//
//
//
//
//

int main(int argc, char** argv)
{
    // Inits a ros node with name *name*
    ros::init(argc, argv, "ransac_stuff");

    // Instantiates a node handle for ros core communication (Subscription, advertisement etc.)
    ros::NodeHandle nh;

    // A struct which holds all the topic names for subs/pubs
    Topics_Config topics_config;

    topics_config.processed_pointcloud_topic = "/LidarProcessor/PointCloud";
    topics_config.inliers_pointcloud_topic = "/Ransac/InlierCloud";
    topics_config.ransac_plane_topic = "/Ransac/Plane";
    topics_config.histogram_inliers_topic = "/Histogram/inliers";

    // Instantiates the node class
    lidar_processor lp = lidar_processor(nh, topics_config);

    // Tells roscore to run (iteratively, otherwise spinOnce) processes all publications etc.
    ros::spin();

    return 0;
}


lidar_processor::lidar_processor(ros::NodeHandle& nh,
                                    Topics_Config topics_config):
                                    nh_(nh),
                                    topics_config_(topics_config),
                                    cached_message(),
                                    buffered_point_cloud(),
                                    inliers_msg(),
                                    plane_msg()   // <- Magic initialization
{
    // Initializes the internal point cloud buffer:
    buffered_point_cloud = new pcl::PointCloud<pcl::PointXYZ>;
    cached_message = new pcl::PointCloud<pcl::PointXYZRGBA>;
    inliers_msg = new pcl::PointCloud<pcl::PointXYZRGBA>;
    OGCLOUD_msg = new pcl::PointCloud<pcl::PointXYZRGBA>;
    plane_msg = new visualization_msgs::Marker;
    histogram_inliers_msg = new pcl::PointCloud<pcl::PointXYZRGBA>;

    initialize_subscribers();
    initialize_publishers();
}


lidar_processor::~lidar_processor() = default;


void lidar_processor::initialize_subscribers()
{
    // Sets up a subscriber to \
    - "buffered_point_topic" topic with \
    - "1000" in message buffer size, and \
    - "callback_point_collection" as callback function for each message received, and \
    - "this" as namespace (i think)

    point_sub =  nh_.subscribe(topics_config_.processed_pointcloud_topic,
                                1000,
                                &lidar_processor::callback_point_collection,
                                this );
}


void lidar_processor::initialize_publishers()
{
    // Sets up a publisher with messages of type \
    - "PointCloud<pcl::PointXYZRGBA>", to the topic \
    - "processed_pointcloud_topic", and \
    - "1000" in message buffer size

    inliers_pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGBA>>(topics_config_.inliers_pointcloud_topic, 1000);
    histogram_inliers_pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGBA>>(topics_config_.histogram_inliers_topic, 1000);
    plane_pub = nh_.advertise<visualization_msgs::Marker>(topics_config_.ransac_plane_topic, 1000);
    OGCLOUD_pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGBA>>("OGCLOUD", 1000);
}


bool lidar_processor::ready_to_run()
{
    // Returns whether the node is ready to start execution/processing
    return true;
}


void RANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud,
            pcl::PointIndices::Ptr inliers,
            pcl::ModelCoefficients::Ptr coefficients)
{
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    segmentation.setOptimizeCoefficients(true);

    segmentation.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setInputCloud(inCloud);

    segmentation.setDistanceThreshold(4);
    segmentation.setAxis(Eigen::Vector3f(0,0,1));
    segmentation.setEpsAngle(2 * (M_PI/180.0f));
    segmentation.setMaxIterations(1000);

    segmentation.segment (*inliers, *coefficients);
}


void Extract_indices(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud,
                     pcl::PointIndices::Ptr indices)
{
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (inCloud);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*outCloud);
}


void Voxelgrid_downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud)
{
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud (inCloud);
    vox.setLeafSize (5, 5, 5);
    vox.filter (*outCloud);
}




void lidar_processor::histogram_filtering(pcl::PointCloud<pcl::PointXYZ>* cloud, std::vector<int>* inlier_indexes, int n_bins, double min_dist, int density_threshold, double height_diff_thresh){

    if(density_threshold < 2){
        density_threshold = 2;
    }

    int n_pts = cloud->points.size();

    // VECNORM2
    std::vector<int> high_dist_inds;
    for (int i = 0; i < n_pts; i++){
        if(sqrtf(pow(cloud->points.at(i).x, 2) + pow(cloud->points.at(i).y, 2) + pow(cloud->points.at(i).z, 2)) > min_dist) {
            high_dist_inds.push_back(i);
        }
    }
    std::cout << high_dist_inds.size() << std::endl;


    // Compute edges for the bins:
    double x_max = 0;
    double x_min = FLT_MAX;
    double y_max = 0;
    double y_min = FLT_MAX;

    for (auto & i : high_dist_inds){
        double x = cloud->points.at(i).x;
        double y = cloud->points.at(i).y;
        x_max = x > x_max ? x : x_max;
        x_min = x < x_min ? x : x_min;
        y_max = y > y_max ? y : y_max;
        y_min = y < y_min ? y : y_min;
    }

    std::vector<double> x_edges;
    std::vector<double> y_edges;

    double x_rez = (x_max-x_min)/n_bins;
    double y_rez = (y_max-y_min)/n_bins;

    for (int i = 0; i <= n_bins; i++){
        x_edges.push_back(x_min + i * x_rez);
        y_edges.push_back(y_min + i * y_rez);
    }

    std::vector<int> bins[n_bins][n_bins];

    std::vector<int> inds_tmp;

    for (int y_i = 0; y_i < n_bins; y_i++){

        inds_tmp.clear();

        auto y_edge_i = y_edges.at(y_i);
        auto y_edge_ip1 = y_edges.at(y_i+1);

        for (int y_j = 0; y_j < n_pts; y_j++){

            auto y_pt = cloud->points.at(y_j).y;

            if(y_pt >= y_edge_i && y_pt < y_edge_ip1){
                inds_tmp.push_back(y_j);
            }
        }

        for (int x_i = 0; x_i < n_bins; x_i++){

            auto x_edge_i = x_edges.at(x_i);
            auto x_edge_ip1 = x_edges.at(x_i+1);

            for(auto x_j : inds_tmp){

                auto x_pt = cloud->points.at(x_j).y;

                if(x_pt >= x_edge_i && x_pt < x_edge_ip1){
                    bins[y_i][x_i].push_back(x_j);
                }
            }
        }
    }

    for (auto & bin_i : bins){
        for (auto & bin : bin_i) {
            std::cout << bin.size() << ",/ ";
            if (bin.size() > 0) {
                std::cout << bin.size() << ", ";
            }
        }
    }

    std::cout << std::endl;

/*
    // Distribute point in bins
    std::vector<int> bins[n_bins][n_bins];

    for (int i = 0; i < n_pts; i++){

        int y_edge_idx = n_bins+1;
        int x_edge_idx = n_bins+1;

        auto& y_pt = cloud->points.at(i).y;
        auto& x_pt = cloud->points.at(i).x;

        // Find y-bin index:
        for(int y_i = 0; y_i < n_bins; y_i++){
            if(y_pt >= y_edges.at(y_i) && y_pt < y_edges.at(y_i + 1)){
                y_edge_idx = y_i;
                break;
            }
        }

        if(y_edge_idx == n_bins+1){
            continue;
        }

        // Find y-bin index:
        for(int x_i = 0; x_i < n_bins; x_i++){
            if(x_pt >= x_edges.at(x_i) && x_pt < x_edges.at(x_i + 1)){
                x_edge_idx = x_i;
                break;
            }
        }

        if(x_edge_idx == n_bins+1){
            continue;
        }

        bins[y_edge_idx][x_edge_idx].push_back(i);
    }*/

    // Filter based on density and height diff in individual bins
    int a = 0;
    int b = 0;
    int c = 0;
    for (auto & bin_i : bins){
        for(auto& bin : bin_i) {
            c++;
            if (bin.size() < density_threshold) {
                continue;
            }
            a++;
            double z_max = 0;
            double z_min = FLT_MAX;
            for (auto &bin_idx : bin) {
                double z = cloud->points.at(bin_idx).z;
                z_max = z > z_max ? z : z_max;
                z_min = z < z_min ? z : z_min;
            }
            b++;
            if (z_max - z_min <= height_diff_thresh) {
                continue;
            }

            for (auto &ind : bin) {
                inlier_indexes->push_back(ind);
            }
        }
    }
    std::cout << a << ", " << b << ", " << c << std::endl;
}

void lidar_processor::execute()
{
    // Here is where any processing (e.g. on the buffered cloud) should be preformed
    std::vector<int> inlier_inds;
    histogram_filtering(buffered_point_cloud, &inlier_inds, 100, 120.0, 10, 100.0);

    std::cout << "post: " << inlier_inds.size() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(buffered_point_cloud);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr OGCLOUD((new pcl::PointCloud<pcl::PointXYZ>)); // Testing cloud

    Voxelgrid_downsample(cloud, cloud);

    RANSAC(cloud, inliers, coefficients);

    pcl::PointIndices::Ptr inliers_cpy (inliers);

    std::cout << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;
    std::cout << "N_inliers: " << inliers->indices.size() << std::endl;

    populate_histogram_filtering_msg(&inlier_inds);
    populate_inliers_msg(inliers);
    populate_plane_msg(coefficients);
    populate_OGCLOUD_msg(OGCLOUD);
}


void lidar_processor::publish_inliers()
{
    // Publishes the input message with the previously constructed publisher
    inliers_pub.publish(*inliers_msg);
}

void lidar_processor::publish_OGCLOUD()
{
    // Publishes the input message with the previously constructed publisher
    OGCLOUD_pub.publish(*OGCLOUD_msg);
}


void lidar_processor::publish_plane()
{
    return;
    // Publishes the input message with the previously constructed publisher
    plane_pub.publish(*plane_msg);
}

void lidar_processor::publish_histogram_inliers(){
    histogram_inliers_pub.publish((*histogram_inliers_msg));
}

void lidar_processor::populate_histogram_filtering_msg(std::vector<int>* inlier_idxs)
{
    // Populates the input message pointer with the current buffered cloud

    histogram_inliers_msg->points.clear();

    // Defines the frame in which the point is defined in reference to, i.e. the origo in reference to each points coordinate
    histogram_inliers_msg->header.frame_id = world_frame;

    // Converts all points one by one to format which rviz can handle:
    for (auto &index: *inlier_idxs){
        auto &point = buffered_point_cloud->points[index];
        pcl::PointXYZRGBA point_rgba;
        point_rgba.x = point.x;
        point_rgba.y = point.y;
        point_rgba.z = point.z;
        point_rgba.rgba =
        point_rgba.r = 249;
        point_rgba.g = 105;
        point_rgba.b = 14;
        point_rgba.a = 255;
        histogram_inliers_msg->points.push_back(point_rgba);
    }
}

void lidar_processor::populate_inliers_msg(pcl::PointIndices::Ptr inlier_idxs)
{
    // Populates the input message pointer with the current buffered cloud

    inliers_msg->points.clear();

    // Defines the frame in which the point is defined in reference to, i.e. the origo in reference to each points coordinate
    inliers_msg->header.frame_id = world_frame;

    // Converts all points one by one to format which rviz can handle:
    for (auto &index: inlier_idxs->indices){
        auto &point = buffered_point_cloud->points[index];
        pcl::PointXYZRGBA point_rgba;
        point_rgba.x = point.x;
        point_rgba.y = point.y;
        point_rgba.z = point.z;
        point_rgba.rgba =
        point_rgba.r = 249;
        point_rgba.g = 105;
        point_rgba.b = 14;
        point_rgba.a = 255;
        inliers_msg->points.push_back(point_rgba);
    }
}

void lidar_processor::populate_OGCLOUD_msg(pcl::PointCloud<pcl::PointXYZ>::Ptr OGCLOUD)
{
    OGCLOUD_msg->points.clear();

    // Defines the frame in which the point is defined in reference to, i.e. the origo in reference to each points coordinate
    OGCLOUD_msg->header.frame_id = world_frame;

    // Converts all points one by one to format which rviz can handle:
    //for (auto &index: inlier_idxs->indices){
    for (auto &point: OGCLOUD->points){
      //  auto &point = buffered_point_cloud->points[index];
        pcl::PointXYZRGBA point_rgba;
        point_rgba.x = point.x;
        point_rgba.y = point.y;
        point_rgba.z = point.z;
        point_rgba.rgba =
        point_rgba.r = 100;
        point_rgba.g = 220;
        point_rgba.b = 178;
        point_rgba.a = 255;
        OGCLOUD_msg->points.push_back(point_rgba);
    }
}


void lidar_processor::populate_plane_msg(pcl::ModelCoefficients::Ptr plane_coefs)
{
return;
   // plane_msg->coef[0]=(plane_coefs->values[0]);
    //plane_msg->coef[1]=(plane_coefs->values[1]);
    //plane_msg->coef[2]=(plane_coefs->values[2]);
    //plane_msg->coef[3]=(plane_coefs->values[3]);

    plane_msg->header.frame_id = world_frame;
    plane_msg->header.stamp = ros::Time::now();

    // Set the namespace and id for this plane_msg->  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    plane_msg->ns = "basic_shapes";
    plane_msg->id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    plane_msg->type = visualization_msgs::Marker::ARROW;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    plane_msg->action = visualization_msgs::Marker::MODIFY;

    // Set the pose of the plane_msg->  This is a full 6DOF pose relative to the frame/time specified in the header
    plane_msg->pose.position.x = 0;
    plane_msg->pose.position.y = 0;
    plane_msg->pose.position.z = 0;
    plane_msg->pose.orientation.x = 0.0;
    plane_msg->pose.orientation.y = 0.0;
    plane_msg->pose.orientation.z = 0.0;
    plane_msg->pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    plane_msg->scale.x = 1.0;
    plane_msg->scale.y = 1.0;
    plane_msg->scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    plane_msg->color.r = 0.0f;
    plane_msg->color.g = 1.0f;
    plane_msg->color.b = 0.0f;
    plane_msg->color.a = 1.0;


}

void lidar_processor::convert_back()
{
    buffered_point_cloud = new pcl::PointCloud<pcl::PointXYZ>;
    for (auto &point : cached_message->points){
        pcl::PointXYZ point_rgba;
        point_rgba.x = point.x;
        point_rgba.y = point.y;
        point_rgba.z = point.z;
        buffered_point_cloud->points.push_back(point_rgba);
    }
}

void lidar_processor::callback_point_collection(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &point_msg)
{
    if (msg_buffered)
        return;

    msg_buffered = true;

    // Callback function for when a message is published on the "buffered_point_topic" by the usb_communicator node
    // This is the only entry-point for exectution after the node is initialized, i.e. this is where all continous processing has to be executed as well.

    // First add the new point to the internal cloud buffer:
    *cached_message = *point_msg;

    convert_back();

    execute();

    // Check if the node is ready for execution/further processing
    // This could for example be when a new full lap is completed
    if (ready_to_run()) {
        publish_inliers();
        publish_OGCLOUD();
        publish_plane();
        publish_histogram_inliers();


        // Publish clouds for visualization, in this case the entire internally buffered cloud:
        //publish(point_msg);
    }
}


