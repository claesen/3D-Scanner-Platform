#include <ros/ros.h>
#include "std_msgs/String.h"
#include <string>
#include "lidar_command_message/lidar_command_message.h"


struct Topics_Config {
    std::string command_topic;
};


class lcd_communicator{

public:
    lcd_communicator(ros::NodeHandle& nh, Topics_Config topics_config);
    ~lcd_communicator();

private:
    ros::NodeHandle nh_;
    void initialize_subscribers();
    void initialize_publishers();
    bool populate(lidar_command_message::lidar_command_message *command_msg);
    void publish(lidar_command_message::lidar_command_message command_msg);

    void do_stuff();

    Topics_Config topics_config_;

    ros::Publisher  command_pub;
};
