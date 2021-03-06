#include <gtest/gtest.h>
#include <stdio.h>
#include <limits.h>
#include <random>
#include <ctime>

#define private public
#include "drive_ros_mavlink_cc2016/time_conv.h"
#include "drive_ros_mavlink_cc2016/calc_cov.h"

typedef TimeConverter::usec_t us;

double fRand(float fMin, float fMax)
{
  double f = (double)rand()/RAND_MAX;
  return fMin + f * (fMax - fMin);
}


TEST(time_convert_test, convert2RosTimeDuration)
{

  // allowed error
  float err = 0.0000001;

  // init class
  TimeConverter time_conv(us(0), us(0), false, NULL);

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



TEST(time_convert_test, convertTime)
{
  ros::NodeHandle n;

  // null as input
  {
    float err = 0.001;

    TimeConverter time_conv(us(0), us(0), false, NULL);
    time_conv.last_time = us(0);
    time_conv.time_offset = ros::Duration(0);
    EXPECT_NEAR(ros::Time::now().toSec(),
                time_conv.convert_time(0).toSec(),
                err);
  }

  // current time as input
  {
    float err = 0.001;

    TimeConverter time_conv(us(0), us(0), false, NULL);
    uint32_t input(ros::Time::now().toNSec() * std::pow(10,-3));
    time_conv.last_time = us(input);
    time_conv.time_offset = ros::Duration(0);
    EXPECT_NEAR(ros::Time::now().toSec(),
                time_conv.convert_time(input).toSec(),
                err);
  }
}


TEST(calculate_covariance_test, calculateCov)
{


  // coefficient is zero
  std::vector<double> a;
  a.push_back(0);
  double values[] = {
                    std::numeric_limits<double>::min(),
                    -1, 0, 1,
                    std::numeric_limits<double>::max()
                  };

  for(int i=0; i<sizeof(values)/sizeof(double); i++)
  {
    EXPECT_FLOAT_EQ(0, calculateCovariance(a, values[i]));
  }

  // one non zero coefficient
  a.pop_back();
  a.push_back(1);

  for(int i=0; i<sizeof(values)/sizeof(double); i++)
  {
    EXPECT_FLOAT_EQ(1, calculateCovariance(a, values[i]));
  }

  // two non zero coefficients
  a.push_back(-1);
  EXPECT_FLOAT_EQ(0, calculateCovariance(a, 1));
  EXPECT_FLOAT_EQ(1, calculateCovariance(a, 2));
  EXPECT_FLOAT_EQ(0, calculateCovariance(a,-2));

  // three non zero coefficients
  a.push_back(0.5);
  EXPECT_FLOAT_EQ(0.5, calculateCovariance(a, 1));
  EXPECT_FLOAT_EQ(2.5, calculateCovariance(a, 2));
  EXPECT_FLOAT_EQ(6.5, calculateCovariance(a,-2));

  // small fuzzy test
  std::srand(std::time(NULL));  // seed

  std::vector<double> b;

  std::cerr << "Using the following coefficients: ";
  for(int i=0; i<5; i++)
  {
    double ran = fRand(-100, 100);
    b.push_back(ran);
    std::cerr << ran << ",";
  }
  std::cerr << std::endl;

  for(int i=0; i<10000; i++)
  {
    ASSERT_NO_THROW(
      double vel = fRand(-100, 100);
      double res = calculateCovariance(a, vel);

      // check if result is toxic
      if(std::isnan(res) || std::isinf(res) || 0 > res){
         std::cerr << "Problem with vel: " << vel << std::endl;
         throw("Something went wrong!");
      }
    );
  }
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  ros::init(argc, argv, "drive_ros_mavlink_cc2016_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
