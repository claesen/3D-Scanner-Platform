cmake_minimum_required(VERSION 2.8.3)
project(usb_communicator)

# Get the information about this package's buildtime dependencies
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        lidar_command_message)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${catkin_LIBRARIES}
        ../../../../serial_interface/include)

# Declare that this catkin package's runtime dependencies
catkin_package(
        INCLUDE_DIRS
            include
            ../../../../serial_interface/include
        CATKIN_DEPENDS
            roscpp
            std_msgs
            sensor_msgs
            lidar_command_message
)


#add_definitions(${PCL_DEFINITIONS})

# define executable not using any messages/services provided by this package
add_executable(${PROJECT_NAME}_node src/usb_communicator.cpp ../../../../serial_interface/src/serial_interface.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})




install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE)
