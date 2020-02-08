cmake_minimum_required(VERSION 2.8.3)
project(lidar_processing)

# Get the information about this package's buildtime dependencies
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        pcl_conversions
        pcl_ros
        pcl_msgs)

find_package(PCL REQUIRED)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${catkin_LIBRARIES}
        ${PCL_INCLUDE_DIRS})

link_directories(
        ${PCL_LIBRARY_DIRS})

# Declare that this catkin package's runtime dependencies
catkin_package(
        INCLUDE_DIRS
            include
        CATKIN_DEPENDS
            roscpp
            std_msgs
            sensor_msgs
            pcl_conversions
            pcl_msgs
        DEPENDS
            PCL
)


#add_definitions(${PCL_DEFINITIONS})

# define executable not using any messages/services provided by this package
add_executable(${PROJECT_NAME}_node src/lidar_processing.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PCL_LIBRARY_DIRS})




install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE)