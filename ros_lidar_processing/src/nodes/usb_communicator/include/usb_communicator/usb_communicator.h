#include <ros/ros.h>
#include "std_msgs/String.h"
#include <string>
#include "geometry_msgs/Point32.h"
#include "lidar_command_message/lidar_command_message.h"
#include "serial_interface.h"
#include <stdio.h>

struct Topics_Config
{
    std::string buffered_point_topic;
    std::string command_topic;
};

// Simply holds the messages recieved from e.g. the lcd_comm node, and the status of recieved messages
class write_usb_holder_class
{
    public:
        bool new_msg_exists = false;

        uint8_t get_msg()
        {
            new_msg_exists = false;
            return msg;
        }
        void set_msg(uint8_t new_msg)
        {
            msg = new_msg;
            new_msg_exists = true;
        }

    private:
        uint8_t msg;
};

// USB_Comm node class:
class usb_communicator
{
    public:
        usb_communicator(ros::NodeHandle& nh, Topics_Config topics_config, char *interface); // Constructor
        ~usb_communicator(); // Destructor
        void run();

    private:
        ros::NodeHandle nh_;
        void initialize_subscribers();
        void initialize_publishers();

        Topics_Config topics_config_;
        ros::Publisher  point_pub;
        void publish(point p);

        // For listening on commands from lcd_com node and sending forth to lidar rig over usb
        void write_usb(uint8_t command);
        void write_usb_callback(const lidar_command_message::lidar_command_message::ConstPtr &msg);
        write_usb_holder_class write_usb_holder;
        ros::Subscriber command_sub;

        serial_interface si;
        // Callback functions for serial interface:
        static void data_cb(void*, point);
        static void new_lap_cb(void*);
        static void done_cb(void*);
};
