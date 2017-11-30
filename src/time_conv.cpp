#include "drive_ros_mavlink_cc2016/time_conv.h"


/**
 * Constructor
 **/
TimeConverter::TimeConverter(usec_t reset_off_us,
                             usec_t comm_off_us,
                             usec_t ign_times_us,
                             bool en_time_debug,
                             ros::Publisher* debug_pub):
   reset_offset_us(reset_off_us),
   comm_offset_us(comm_off_us),
   ignore_times_us(ign_times_us),
   enable_time_debug(en_time_debug),
   debug_publisher(debug_pub)
{



}

/**
 *  Converts microseconds to ROS-Time
 **/
ros::Time TimeConverter::convert2RosTime(usec_t in) const
{
  sec_t  sec = std::chrono::duration_cast<sec_t>(in);
  nsec_t nsec = std::chrono::duration_cast<nsec_t>(in) - std::chrono::duration_cast<nsec_t>(sec);
  return ros::Time(sec.count(), nsec.count());
}


/**
 *  Converts microseconds to ROS-Duration
 **/
ros::Duration TimeConverter::convert2RosDuration(usec_t in) const
{
  return convert2RosTime(in) - ros::Time(0,0);
}


/**
 *  this function ignores small time diffs
 **/
TimeConverter::usec_t TimeConverter::ignoreSmallTimeDiffs(const usec_t in)
{
  usec_t out;

  // check if we have enough difference
  if(ignore_times_us.count() > std::abs((in - last_time).count()))
  {
    // use last time
    out = last_time;
    ROS_DEBUG("Use old time");
  }else{

     // use new time
     last_time = in;
     out = in;
     ROS_DEBUG("Use new time");
  }

  return out;
}



/**
 *  Converts mavlink time into ROS time
 **/
ros::Time TimeConverter::convert_time(const uint32_t usec)
{

  ros::Time mav_time = convert2RosTime(ignoreSmallTimeDiffs(static_cast<usec_t>(usec)));
  ros::Time now_time = ros::Time::now();
  ros::Time ros_time = mav_time + time_offset;

  // calculate difference
  ros::Duration diff_time = now_time - ros_time;

  // check if difference is ok or reset if not
  if(std::abs(diff_time.toNSec()) > std::chrono::duration_cast<nsec_t>(reset_offset_us).count())
  {
    // TODO: maybe use some fancy interpolation algorithm to calculate time
    time_offset = now_time - mav_time ;
    ros_time = now_time;
    diff_time = ros::Duration(0,0);

  }

  // substract communication offset
  ros_time = ros_time - convert2RosDuration(comm_offset_us);

  // publish times for debugging purposes
  if(enable_time_debug)
  {
    drive_ros_msgs::TimeCompare msg;
    msg.diff_time = diff_time;
    msg.time_1 = mav_time;
    msg.time_2 = ros_time;
    msg.header.stamp = now_time;
    debug_publisher->publish(msg);
  }



  ROS_DEBUG_STREAM("mav_in[usec]:" << usec << " ros_time[nsec]:" << ros_time.toNSec() << "  time_offset[sec]:" << time_offset.sec << "  diff_double[sec]:" << diff_time.toSec() );

  return ros_time;
}
