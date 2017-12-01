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

  // diff bigger than offset -> use new time
  {
    time_conv.last_time = us(0);
    EXPECT_EQ(us(200), time_conv.ignoreSmallTimeDiffs(us(200)));
  }

  // diff smaller than offset -> use old time
  {
    time_conv.last_time = us(0);
    EXPECT_EQ(us(0), time_conv.ignoreSmallTimeDiffs(us(50)));
  }

  // diff equals to offset -> use new time
  {
    time_conv.last_time = us(0);
    EXPECT_EQ(us(100), time_conv.ignoreSmallTimeDiffs(us(100)));
  }

  // diff bigger to offset -> use new time
  {
    time_conv.last_time = us(150);
    EXPECT_EQ(us(10), time_conv.ignoreSmallTimeDiffs(us(10)));
  }

  // diff smaller to offset -> use old time
  {
    time_conv.last_time = us(150);
    EXPECT_EQ(us(150), time_conv.ignoreSmallTimeDiffs(us(100)));
  }


}


TEST(time_convert_test, convertTime)
{
  ros::NodeHandle n;

  // null as input
  {
    float err = 0.001;

    TimeConverter time_conv(us(0), us(0), us(100), false, NULL);
    time_conv.last_time = us(0);
    time_conv.time_offset = ros::Duration(0);
    EXPECT_NEAR(ros::Time::now().toSec(),
                time_conv.convert_time(0).toSec(),
                err);
  }

  // current time as input
  {
    float err = 0.001;

    TimeConverter time_conv(us(0), us(0), us(100), false, NULL);
    uint32_t input(ros::Time::now().toNSec() * std::pow(10,-3));
    time_conv.last_time = us(input);
    time_conv.time_offset = ros::Duration(0);
    EXPECT_NEAR(ros::Time::now().toSec(),
                time_conv.convert_time(input).toSec(),
                err);
  }


}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  ros::init(argc, argv, "drive_ros_mavlink_cc2016_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
