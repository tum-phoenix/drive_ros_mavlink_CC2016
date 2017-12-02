#include "ros/ros.h"
#include "drive_ros_msgs/TimeCompare.h"
#include <gtest/gtest.h>

double min_diff_us, max_diff_us, test_duration;

void callback(const drive_ros_msgs::TimeCompareConstPtr& msg)
{
  double diff_us = msg->diff_time.toNSec() * pow(10, -3);

  if(diff_us > max_diff_us)
  {
    ROS_ERROR_STREAM("Diff time exceeds maximum: " << diff_us << " [us]");
    throw std::runtime_error("Diff time exceeds maximum.");
  }

  if(diff_us < min_diff_us)
  {
    ROS_ERROR_STREAM("Diff time exceeds minimum: " << diff_us << " [us]");
    throw std::runtime_error("Diff time exceeds minimum.");
  }

}

TEST(TestDebugTime, diffTime)
{
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");


  pnh.param<double>("min_diff_us", min_diff_us, -0.05);
  pnh.param<double>("max_diff_us", max_diff_us, +0.05);
  pnh.param<double>("test_duration", test_duration, 10);

  ros::Subscriber sub = nh.subscribe("/from_mav/debug_times", 0, callback);

  ros::Duration lifetime(test_duration);
  ros::Time start = ros::Time::now();

  while(ros::Time::now() < start + lifetime)
  {
    ros::spinOnce();
  }
}




int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "mavlink_phoenix_debug_time_test");

  ros::NodeHandle nh;

  return RUN_ALL_TESTS();;
}
