#include "usb_communicator/usb_communicator.h"


// Main function which is run when ruslaunch is run:
int main(int argc, char** argv)
{
    // Instantiates node called usb_communicator in roscore
    ros::init(argc, argv, "usb_communicator");
    ros::NodeHandle nh;

    // Define topics:
    Topics_Config topics_config;
    topics_config.command_topic = "/LCDCommunicator/Command";
    topics_config.buffered_point_topic = "/USBCommunicator/Point";

    // Define USB interface:
    char interface[] = "/dev/ttyACM0";

    // Instantiate usb_com node class:
    usb_communicator usb = usb_communicator(nh, topics_config, interface);

    while (ros::ok())
    {
        ros::spinOnce(); // This actually allows publishing (Process buffered messages once)
        usb.run(); // Runs the usb_com node class once
    }

    return 0;
}

// Pass function handles to serial interface
usb_communicator::usb_communicator(ros::NodeHandle& nh,
                                    Topics_Config topics_config,
                                    char *interface):
                                    nh_(nh),
                                    topics_config_(topics_config),
                                    write_usb_holder(),
                                    si(interface,
                                    this,
                                    this->data_cb,
                                    this->new_lap_cb,
                                    this->done_cb)
{
    initialize_subscribers();
    initialize_publishers();

    // Runs start lidar with argument "number of laps" and sends commands to scanner
    this->si.startLidar(3);

//      Create and start a new thread here:
//      bufferer.start_communication();
}

usb_communicator::~usb_communicator()
{
    // On destroy: Stop lidar
    this->si.stopLidar();
}

void usb_communicator::initialize_subscribers()
{
    command_sub =  nh_.subscribe(
                            topics_config_.command_topic,
                            1000,
                            &usb_communicator::write_usb_callback,
                            this );
}

void usb_communicator::initialize_publishers()
{
    point_pub = nh_.advertise<geometry_msgs::Point32>(topics_config_.buffered_point_topic, 1000);
}


void usb_communicator::publish(point p)
{
    // Populates a message with the input point and publishes that message
    geometry_msgs::Point32 point_msg;
    point_msg.x = p.getX();
    point_msg.y = p.getY();
    point_msg.z = p.getZ();
    point_pub.publish(point_msg);
}


void usb_communicator::write_usb(uint8_t command)
{
  // Should act as a parser for the lcd-com node and send commands forth to the lidar rig over usb

    //int startLidar();
  //int startLidar(int);
  //int stopLidar();
  //int restartLidar();
  //int setSpeed(int);
  //int setResolution(int);
}

void usb_communicator::run()
{
    try {
    this->si.run();
    } catch (std::runtime_error e) {
    /*printf("%s\n", e.what()); */}
}

void usb_communicator::write_usb_callback(const lidar_command_message::lidar_command_message::ConstPtr &msg)
{
    // Nice way to store current commands and so, but is not fully implemented and is not currently utilized
    write_usb_holder.set_msg(msg->command.val);
}

void usb_communicator::data_cb(void *user_data, point p)
{
    // Data callback, is called when a point is recieved/read over usb:
    // publishes the received point

    // No fucking idea what this is:
    usb_communicator *usb = (usb_communicator*) user_data;

    // Publishes the point:
    usb->publish(p);
}

void usb_communicator::new_lap_cb(void *user_data)
{
    // New lap callback, i.e. it is called when a new lap is registred
    usb_communicator *usb = (usb_communicator*) user_data;
}

void usb_communicator::done_cb(void *user_data)
{
    // Done callback, i.e. this is called when the rig sends a flag that the scan is complete
    usb_communicator *usb = (usb_communicator*) user_data;
    usb->si.stopLidar();
}