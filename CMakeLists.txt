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

add_executable(mavlink_phoenix_node
    src/mavlink_phoenix_node.cpp
    src/time_conv.cpp)
target_link_libraries(mavlink_phoenix_node ${catkin_LIBRARIES})
add_dependencies(mavlink_phoenix_node  drive_ros_msgs_generate_messages_cpp)

install(TARGETS mavlink_phoenix_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    # unit tests
    add_rostest_gtest(mavlink_phoenix_unit_test launch/mavlink_CC2016_unit.test
        test/mavlink_phoenix_unit_test.cpp
        src/time_conv.cpp
        )
    target_link_libraries(mavlink_phoenix_unit_test ${catkin_LIBRARIES})

    # regression tests
    add_executable(mavlink_phoenix_debug_time_test
        test/mavlink_phoenix_debug_time_test.cpp)
    target_link_libraries(mavlink_phoenix_debug_time_test ${catkin_LIBRARIES} gtest gtest_main)
    add_dependencies(mavlink_phoenix_debug_time_test  drive_ros_msgs_generate_messages_cpp)


    add_rostest_gtest(mavlink_phoenix_regress_test launch/mavlink_CC2016_regress.test
        src/mavlink_phoenix_node.cpp
        src/time_conv.cpp
        )
    target_link_libraries(mavlink_phoenix_regress_test ${catkin_LIBRARIES})

endif()


install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    FILES_MATCHING PATTERN "*.launch"
)


