#ifndef  _TIME_CONV_H_
#define  _TIME_CONV_H_

#include <ros/ros.h>
#include <cstdlib>
#include <chrono>
#include <ratio>
#include "drive_ros_msgs/TimeCompare.h"



class TimeConverter
{
public:

  // time types
  typedef std::chrono::seconds        sec_t;
  typedef std::chrono::microseconds  usec_t;
  typedef std::chrono::nanoseconds   nsec_t;

  TimeConverter(usec_t reset_off_us,
                usec_t comm_off_us,
                bool enable_time_debug,
                ros::Publisher* debug_pub);


  ros::Time convert_time(const uint32_t usec);

private:
  const usec_t reset_offset_us;
  const usec_t comm_offset_us;
  const bool enable_time_debug;
  ros::Publisher* debug_publisher;

  usec_t last_time;
  ros::Duration time_offset;

  ros::Time convert2RosTime(usec_t in) const;
  ros::Duration convert2RosDuration(usec_t in) const;
};



#endif
