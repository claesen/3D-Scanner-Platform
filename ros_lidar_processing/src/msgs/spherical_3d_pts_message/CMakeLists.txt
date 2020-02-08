cmake_minimum_required(VERSION 2.8.3)
project(spherical_3d_pts_message)

# Get the information about this package's buildtime dependencies
find_package(catkin REQUIRED
        COMPONENTS message_generation std_msgs)


# Declare the message files to be built
add_message_files(FILES
        spherical_3d_pts_message.msg
        )

# Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
        CATKIN_DEPENDS message_runtime std_msgs
)