cmake_minimum_required(VERSION 3.0.2)
project(localization_309605004)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf2
    tf
    rospy
    std_msgs
    pcl_ros
)


# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.7 REQUIRED)


catkin_package( 
)

###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (lo1 src/localization1.cpp)
target_link_libraries (lo1
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable (lo2 src/localization2.cpp)
target_link_libraries (lo2
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable (lo3 src/localization3.cpp)
target_link_libraries (lo3
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)


