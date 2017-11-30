#include <gtest/gtest.h>
#include <stdio.h>
#include <limits.h>

#define private public
#include "drive_ros_mavlink_cc2016/time_conv.h"

typedef TimeConverter::usec_t us;

TEST(time_convert_test, convert2RosTimeDuration)
{

  // allowed error
  float err = 0.0000001;

  // init class
  TimeConverter time_conv(us(0), us(0), us(0), false, NULL);

  // values to test
  uint32_t values[] = {0,
                       4,
                       std::numeric_limits<uint32_t>::max(),
                       std::numeric_limits<uint32_t>::min()
                      };

  for(int i=0; i<sizeof(values)/sizeof(uint32_t); i++)
  {
    us time_usec(values[i]);
    double time_sec = values[i]* std::pow(10,-6);
    EXPECT_NEAR(ros::Time(time_sec).toSec(), time_conv.convert2RosTime(time_usec).toSec(), err);
    EXPECT_NEAR(ros::Duration(time_sec).toSec(), time_conv.convert2RosDuration(time_usec).toSec(), err);
  }
}


TEST(time_convert_test, ignoreSmallTimeDiffs)
{

  TimeConverter time_conv(us(0), us(0), us(100), false, NULL);

  // new value bigger than offset -> use new time
  {
    time_conv.last_time = us(0);
    EXPECT_EQ(us(200), time_conv.ignoreSmallTimeDiffs(us(200)));
  }

  // new value smaller than offset -> use old time
  {
    time_conv.last_time = us(0);
    EXPECT_EQ(us(0), time_conv.ignoreSmallTimeDiffs(us(50)));
  }

  // new value equals to offset -> use new time
  {
    time_conv.last_time = us(0);
    EXPECT_EQ(us(100), time_conv.ignoreSmallTimeDiffs(us(100)));
  }


}


TEST(time_convert_test, convertTime)
{



//  ignore_times_us = new usec_t(40);
//  comm_offset_us = new usec_t(0);
//  reset_offset_us = new usec_t(1000);

//  enable_time_debug = false;

//  ros::NodeHandle n_test;

//  // zero mavlink time
//  {
//    uint32_t input(0);
//    EXPECT_NEAR(ros::Time::now().toSec(), convert_time(input).toSec(), 0.001);
//  }


//  // mavlink with small tolerable offset
//  {
//    ros::Time ros_time = ros::Time::now();
//    usec_t input(static_cast<uint32_t>(ros_time.toNSec() * pow(10,-3)));
//    float offset = 0.0009;
//    time_offset = ros::Duration(offset);
//    last_time = input;
//    sleep(offset);
//    EXPECT_NEAR(ros_time.toSec(), convert_time(static_cast<uint32_t>(input.count())).toSec(), 0.00001);
//  }


}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  ros::init(argc, argv, "drive_ros_mavlink_cc2016_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
