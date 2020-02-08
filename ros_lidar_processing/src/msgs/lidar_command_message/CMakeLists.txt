cmake_minimum_required(VERSION 2.8.3)
project(lidar_command_message)

# Get the information about this package's buildtime dependencies
find_package(catkin REQUIRED
        COMPONENTS message_generation std_msgs)


# Declare the message files to be built
add_message_files(FILES
        lidar_command_message.msg
        COMMAND.msg
        )

# Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
        CATKIN_DEPENDS message_runtime std_msgs
)