#include "lcd_communicator/lcd_communicator.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lcd_communicator");

    ros::NodeHandle nh;

    Topics_Config topics_config;

    topics_config.command_topic = "/LCDCommunicator/Command";

    lcd_communicator(nh, topics_config);

    while (ros::ok())
    {
        ros::spinOnce(); // This actually allows publishing
    }

    return 0;
}


lcd_communicator::lcd_communicator(ros::NodeHandle& nh, Topics_Config topics_config): nh_(nh), topics_config_(topics_config)
{
    initialize_subscribers();
    initialize_publishers();

    // Create and start a new thread here:
//    bufferer.start_communication();

}

lcd_communicator::~lcd_communicator() = default;

void lcd_communicator::initialize_subscribers(){
}

void lcd_communicator::initialize_publishers(){
    command_pub = nh_.advertise<lidar_command_message::lidar_command_message>(topics_config_.command_topic, 1000);
}


bool lcd_communicator::populate(lidar_command_message::lidar_command_message *command_msg){
    // pupulate the message
    return true;
}

void lcd_communicator::publish(lidar_command_message::lidar_command_message command_msg){
    command_pub.publish(command_msg);
}



void lcd_communicator::do_stuff(){
    // Main loopyloop in its own thread:

    //Something like diz somewhere when a point is successfully recieved:
    lidar_command_message::lidar_command_message command_msg;
    if(populate(&command_msg)){
        publish(command_msg);
    }
}

