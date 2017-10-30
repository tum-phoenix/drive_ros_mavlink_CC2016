cmake_minimum_required(VERSION 2.8.3)
project(drive_ros_mavlink_cc2016)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  drive_ros_msgs
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs drive_ros_msgs
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(mavlink_phoenix_node src/mavlink_phoenix_node.cpp)
target_link_libraries(mavlink_phoenix_node ${catkin_LIBRARIES})
add_dependencies(mavlink_phoenix_node  drive_ros_msgs_generate_messages_cpp)

install(TARGETS mavlink_phoenix_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    FILES_MATCHING PATTERN "*.launch"
)
