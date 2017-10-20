cmake_minimum_required(VERSION 2.8.3)
project(mavlink_phoenix)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  message_generation

)

add_message_files(
   FILES
   MAV_RAW_DATA.msg
        NOTIFICATION.msg
        HEARTBEAT.msg
        DEBUG.msg
        TELEMETRY.msg
        CONTROL_LIGHTS.msg
        CONTROL_COMMAND.msg
        IMU.msg
        ODOMETER_ABS.msg
        ODOMETER_RAW.msg
        ODOMETER_DELTA.msg
        ODOMETER_DELTA_RAW.msg
        ODOMETER.msg
        PROXIMITY.msg
        PARKING_LOT.msg
        CONFIG_REQUEST_COUNT.msg
        CONFIG_REQUEST.msg
        CONFIG_REQUEST_PARAMS.msg
        CONFIG_COUNT.msg
        CONFIG.msg
        CONFIG_PARAM_INT.msg
        CONFIG_PARAM_BOOL.msg
        CONFIG_PARAM_FLOAT.msg
        CONFIG_PARAM_SET_INT.msg
        CONFIG_PARAM_SET_BOOL.msg
        CONFIG_PARAM_SET_FLOAT.msg
        COMMAND.msg

)
generate_messages(
    DEPENDENCIES
    geometry_msgs
    std_msgs

)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs message_runtime
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

    add_executable(mavlink_phoenix_node src/mavlink_phoenix_node.cpp)
    target_link_libraries(mavlink_phoenix_node ${catkin_LIBRARIES})
    add_dependencies(mavlink_phoenix_node  mavlink_phoenix_generate_messages_cpp)

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
