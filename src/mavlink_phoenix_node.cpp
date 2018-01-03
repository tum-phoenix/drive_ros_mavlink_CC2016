
/**
 *
 **/
#include <drive_ros_mavlink_cc2016/mavlink2ros.h>
#include <drive_ros_mavlink_cc2016/phoenix/mavlink.h>
#include <drive_ros_mavlink_cc2016/time_conv.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <drive_ros_msgs/VehicleEncoder.h>
#include <fstream>
#include <ros/ros.h>

mavlink_message_t mav_msg; //! Global mavlink message
mavlink_status_t status;   //! Global mavlink status

ros::Publisher to_mav_mav_raw_data_publisher;     //! ROS publisher to write to
                                                  //! mavlink interface
ros::Subscriber from_mav_mav_raw_data_subscriber; //! ROS subscriber to read
                                                  //! from mavlink interface

ros::Publisher from_mav_notification_pub;
ros::Publisher from_mav_heartbeat_pub;
ros::Publisher from_mav_debug_pub;
ros::Publisher from_mav_telemetry_pub;
ros::Publisher from_mav_imu_pub;
ros::Publisher from_mav_odometer_abs_pub;
ros::Publisher from_mav_odometer_raw_pub;
ros::Publisher from_mav_odometer_delta_pub;
ros::Publisher from_mav_odometer_delta_raw_pub;
ros::Publisher from_mav_odometer_pub;
ros::Publisher from_mav_proximity_pub;
ros::Publisher from_mav_parking_lot_pub;
ros::Publisher from_mav_config_count_pub;
ros::Publisher from_mav_config_pub;
ros::Publisher from_mav_config_param_int_pub;
ros::Publisher from_mav_config_param_bool_pub;
ros::Publisher from_mav_config_param_float_pub;
ros::Publisher from_mav_command_pub;


// covariance matrix
enum COV{
  XX = 0,
  XY = 1,
  XZ = 2,
  YX = 3,
  YY = 4,
  YZ = 5,
  ZX = 6,
  ZY = 7,
  ZZ = 8
};



// parameter
static bool enable_imu_debug;
static std::ofstream file_log;

static std::vector<double> odo_pos_var;
static std::vector<double> odo_vel_var;
static std::vector<double> imu_acc_cov_xx;
static std::vector<double> imu_acc_cov_yy;
static std::vector<double> imu_acc_cov_zz;
static std::vector<double> imu_gyr_cov_xx;
static std::vector<double> imu_gyr_cov_yy;
static std::vector<double> imu_gyr_cov_zz;


static double current_vel = 0;

// time converter class
static TimeConverter* time_conv;

static const double gravity = 9.81;


// calculates the covariances based on the current velocity
double calculateCovariance(const std::vector<double> coef, const double vel)
{
  double out = 0;
  for(int i=0; i<coef.size(); i++)
  {
    out += coef.at(i) * std::pow(vel, i);
  }

  return out;
}



/**
 *
 **/
int write_to_mav(uint8_t *b, int sz) {

  drive_ros_msgs::mav_RAW_DATA m;

  m.channel = drive_ros_msgs::mav_RAW_DATA::CH_COMM0;
  m.data.assign(b, b + sz);
  int rc = m.data.size();
  to_mav_mav_raw_data_publisher.publish(m);
  ROS_DEBUG("Writen to MAV %d bytes", rc);
  return rc;
}

/**
 *
 */
void to_mav_control_lights_callback(
    const drive_ros_msgs::mav_cc16_CONTROL_LIGHTS::ConstPtr &msg) {
  ROS_DEBUG("[drive_ros_mavlink_cc2016] Received a  'to_mav_CONTROL_LIGHTS request");

  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t m;

  m.sysid = msg->sysid;
  m.compid = msg->compid;

  mavlink_control_lights_t control_lights_out;

  /** ASSIGN FIELDS **/

  memcpy(&(control_lights_out.colors), &(msg->colors[0]),
         sizeof(uint32_t) * (int)(msg->colors.size()));

  mavlink_msg_control_lights_encode(msg->sysid, msg->compid, &m,
                                    &control_lights_out);
  uint16_t len = mavlink_msg_to_send_buffer(data, &m);
  write_to_mav(data, len);
}
/**
 *
 */
void to_mav_control_command_callback(
    const drive_ros_msgs::mav_cc16_CONTROL_COMMAND::ConstPtr &msg) {
  ROS_DEBUG("[drive_ros_mavlink_cc2016] Received a  'to_mav_CONTROL_COMMAND request");

  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t m;

  m.sysid = msg->sysid;
  m.compid = msg->compid;

  mavlink_control_command_t control_command_out;

  /** ASSIGN FIELDS **/

  control_command_out.velocity = msg->velocity;
  control_command_out.steering_front = msg->steering_front;
  control_command_out.steering_rear = msg->steering_rear;
  control_command_out.indicator_left = msg->indicator_left;
  control_command_out.indicator_right = msg->indicator_right;

  mavlink_msg_control_command_encode(msg->sysid, msg->compid, &m,
                                     &control_command_out);
  uint16_t len = mavlink_msg_to_send_buffer(data, &m);
  write_to_mav(data, len);
}
/**
 *
 */
void to_mav_config_request_count_callback(
    const drive_ros_msgs::mav_cc16_CONFIG_REQUEST_COUNT::ConstPtr &msg) {
  ROS_DEBUG("[drive_ros_mavlink_cc2016] Received a  'to_mav_CONFIG_REQUEST_COUNT request");

  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t m;

  m.sysid = msg->sysid;
  m.compid = msg->compid;

  mavlink_config_request_count_t config_request_count_out;

  /** ASSIGN FIELDS **/

  config_request_count_out.dummy = msg->dummy;

  mavlink_msg_config_request_count_encode(msg->sysid, msg->compid, &m,
                                          &config_request_count_out);
  uint16_t len = mavlink_msg_to_send_buffer(data, &m);
  write_to_mav(data, len);
}
/**
 *
 */
void to_mav_config_request_callback(
    const drive_ros_msgs::mav_cc16_CONFIG_REQUEST::ConstPtr &msg) {
  ROS_DEBUG("[drive_ros_mavlink_cc2016] Received a  'to_mav_CONFIG_REQUEST request");

  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t m;

  m.sysid = msg->sysid;
  m.compid = msg->compid;

  mavlink_config_request_t config_request_out;

  /** ASSIGN FIELDS **/

  config_request_out.config_id = msg->config_id;

  mavlink_msg_config_request_encode(msg->sysid, msg->compid, &m,
                                    &config_request_out);
  uint16_t len = mavlink_msg_to_send_buffer(data, &m);
  write_to_mav(data, len);
}
/**
 *
 */
void to_mav_config_request_params_callback(
    const drive_ros_msgs::mav_cc16_CONFIG_REQUEST_PARAMS::ConstPtr &msg) {
  ROS_DEBUG("[drive_ros_mavlink_cc2016] Received a  'to_mav_CONFIG_REQUEST_PARAMS request");

  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t m;

  m.sysid = msg->sysid;
  m.compid = msg->compid;

  mavlink_config_request_params_t config_request_params_out;

  /** ASSIGN FIELDS **/

  config_request_params_out.config_id = msg->config_id;
  config_request_params_out.param_id = msg->param_id;

  mavlink_msg_config_request_params_encode(msg->sysid, msg->compid, &m,
                                           &config_request_params_out);
  uint16_t len = mavlink_msg_to_send_buffer(data, &m);
  write_to_mav(data, len);
}
/**
 *
 */
void to_mav_config_param_set_int_callback(
    const drive_ros_msgs::mav_cc16_CONFIG_PARAM_SET_INT::ConstPtr &msg) {
  ROS_DEBUG("[drive_ros_mavlink_cc2016] Received a  'to_mav_CONFIG_PARAM_SET_INT request");

  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t m;

  m.sysid = msg->sysid;
  m.compid = msg->compid;

  mavlink_config_param_set_int_t config_param_set_int_out;

  /** ASSIGN FIELDS **/

  config_param_set_int_out.config_id = msg->config_id;
  config_param_set_int_out.param_id = msg->param_id;
  config_param_set_int_out.value = msg->value;

  mavlink_msg_config_param_set_int_encode(msg->sysid, msg->compid, &m,
                                          &config_param_set_int_out);
  uint16_t len = mavlink_msg_to_send_buffer(data, &m);
  write_to_mav(data, len);
}
/**
 *
 */
void to_mav_config_param_set_bool_callback(
    const drive_ros_msgs::mav_cc16_CONFIG_PARAM_SET_BOOL::ConstPtr &msg) {
  ROS_DEBUG("[drive_ros_mavlink_cc2016] Received a  'to_mav_CONFIG_PARAM_SET_BOOL request");

  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t m;

  m.sysid = msg->sysid;
  m.compid = msg->compid;

  mavlink_config_param_set_bool_t config_param_set_bool_out;

  /** ASSIGN FIELDS **/

  config_param_set_bool_out.config_id = msg->config_id;
  config_param_set_bool_out.param_id = msg->param_id;
  config_param_set_bool_out.value = msg->value;

  mavlink_msg_config_param_set_bool_encode(msg->sysid, msg->compid, &m,
                                           &config_param_set_bool_out);
  uint16_t len = mavlink_msg_to_send_buffer(data, &m);
  write_to_mav(data, len);
}
/**
 *
 */
void to_mav_config_param_set_float_callback(
    const drive_ros_msgs::mav_cc16_CONFIG_PARAM_SET_FLOAT::ConstPtr &msg) {
  ROS_DEBUG("[drive_ros_mavlink_cc2016] Received a  'to_mav_CONFIG_PARAM_SET_FLOAT request");

  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t m;

  m.sysid = msg->sysid;
  m.compid = msg->compid;

  mavlink_config_param_set_float_t config_param_set_float_out;

  /** ASSIGN FIELDS **/

  config_param_set_float_out.config_id = msg->config_id;
  config_param_set_float_out.param_id = msg->param_id;
  config_param_set_float_out.value = msg->value;

  mavlink_msg_config_param_set_float_encode(msg->sysid, msg->compid, &m,
                                            &config_param_set_float_out);
  uint16_t len = mavlink_msg_to_send_buffer(data, &m);
  write_to_mav(data, len);
}
/**
 *
 */
void to_mav_command_callback(const drive_ros_msgs::mav_cc16_COMMAND::ConstPtr &msg) {
  ROS_DEBUG("[drive_ros_mavlink_cc2016] Received a  'to_mav_COMMAND request");

  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t m;

  m.sysid = msg->sysid;
  m.compid = msg->compid;

  mavlink_command_t command_out;

  /** ASSIGN FIELDS **/

  command_out.command = msg->command;

  mavlink_msg_command_encode(msg->sysid, msg->compid, &m, &command_out);
  uint16_t len = mavlink_msg_to_send_buffer(data, &m);
  write_to_mav(data, len);
}

/**
 *
 */
void from_mav_mav_raw_data_callback(
    const drive_ros_msgs::mav_RAW_DATA::ConstPtr &msg) {

  for (int i = 0; i < msg->data.size(); ++i) {
    // Try to get a new message
    if (mavlink_parse_char(msg->channel, msg->data[i], &mav_msg, &status)) {
      // Handle message
      switch (mav_msg.msgid) {

      case MAVLINK_MSG_ID_NOTIFICATION: {
        drive_ros_msgs::mav_cc16_NOTIFICATION m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_notification_t notification_in;
        memset(&notification_in, 0, sizeof(notification_in));
        mavlink_msg_notification_decode(&mav_msg, &notification_in);

        m.header.stamp = time_conv->convert_time(notification_in.timestamp);
        m.type = notification_in.type;
        memcpy(&(m.description), &(notification_in.description),
               sizeof(char) * 50);
        memcpy(&(m.tag), &(notification_in.tag), sizeof(char) * 30);

        from_mav_notification_pub.publish(m);

        ROS_DEBUG("[drive_ros_mavlink_cc2016] Received a 'NOTIFICATION' from mavlink.");
      } break;
      case MAVLINK_MSG_ID_HEARTBEAT: {
        drive_ros_msgs::mav_cc16_HEARTBEAT m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_heartbeat_t heartbeat_in;
        memset(&heartbeat_in, 0, sizeof(heartbeat_in));
        mavlink_msg_heartbeat_decode(&mav_msg, &heartbeat_in);

        m.header.stamp = time_conv->convert_time(heartbeat_in.timestamp);
        m.header.frame_id = "rear_axis_middle";
        m.battery_voltage = heartbeat_in.battery_voltage;
        m.remote_control = heartbeat_in.remote_control;
        m.drive_mode = heartbeat_in.drive_mode;
        m.rc_velocity = heartbeat_in.rc_velocity;
        m.rc_steering_front = heartbeat_in.rc_steering_front;
        m.rc_steering_rear = heartbeat_in.rc_steering_rear;

        from_mav_heartbeat_pub.publish(m);
        ROS_DEBUG("[drive_ros_mavlink_cc2016] Received a 'HEARTBEAT' from mavlink.");
      } break;
      case MAVLINK_MSG_ID_DEBUG: {
        drive_ros_msgs::mav_cc16_DEBUG m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_debug_t debug_in;
        memset(&debug_in, 0, sizeof(debug_in));
        mavlink_msg_debug_decode(&mav_msg, &debug_in);

        m.header.stamp = time_conv->convert_time(debug_in.timestamp);
        memcpy(&(m.data), &(debug_in.data), sizeof(float) * 12);

        from_mav_debug_pub.publish(m);
        ROS_DEBUG("[drive_ros_mavlink_cc2016] Received a 'DEBUG' from mavlink.");
      } break;
      case MAVLINK_MSG_ID_TELEMETRY: {
        drive_ros_msgs::mav_cc16_TELEMETRY m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_telemetry_t telemetry_in;
        memset(&telemetry_in, 0, sizeof(telemetry_in));
        mavlink_msg_telemetry_decode(&mav_msg, &telemetry_in);

        m.header.stamp = time_conv->convert_time(telemetry_in.timestamp);
        m.xacc = telemetry_in.xacc;
        m.yacc = telemetry_in.yacc;
        m.zacc = telemetry_in.zacc;
        m.xgyro = telemetry_in.xgyro;
        m.ygyro = telemetry_in.ygyro;
        m.zgyro = telemetry_in.zgyro;
        m.dist_front = telemetry_in.dist_front;
        m.dist_rear = telemetry_in.dist_rear;
        m.dist_side = telemetry_in.dist_side;
        m.odom = telemetry_in.odom;
        m.odom_accumulated = telemetry_in.odom_accumulated;
        m.xmotion_front = telemetry_in.xmotion_front;
        m.ymotion_front = telemetry_in.ymotion_front;
        m.xmotion_rear = telemetry_in.xmotion_rear;
        m.ymotion_rear = telemetry_in.ymotion_rear;
        m.motion_front_quality = telemetry_in.motion_front_quality;
        m.motion_rear_quality = telemetry_in.motion_rear_quality;
        m.current_motor = telemetry_in.current_motor;
        m.current_servo_front = telemetry_in.current_servo_front;
        m.current_servo_rear = telemetry_in.current_servo_rear;
        m.current_total = telemetry_in.current_total;
        m.pwm_servo_front = telemetry_in.pwm_servo_front;
        m.pwm_servo_rear = telemetry_in.pwm_servo_rear;
        m.battery_voltage = telemetry_in.battery_voltage;
        m.remote_control = telemetry_in.remote_control;
        m.drive_mode = telemetry_in.drive_mode;

        from_mav_telemetry_pub.publish(m);
        ROS_DEBUG("[drive_ros_mavlink_cc2016] Received a 'TELEMETRY' from mavlink.");
      } break;
      case MAVLINK_MSG_ID_IMU: {
        sensor_msgs::Imu m;

        mavlink_imu_t imu_in;
        memset(&imu_in, 0, sizeof(imu_in));
        mavlink_msg_imu_decode(&mav_msg, &imu_in);

        m.header.stamp = time_conv->convert_time(imu_in.timestamp);
        m.header.frame_id = "imu";

        m.linear_acceleration.x = imu_in.xacc * gravity; // convert g's to m/s2
        m.linear_acceleration.y = imu_in.yacc * gravity;
        m.linear_acceleration.z = -imu_in.zacc * gravity; // wrong orientation
        m.linear_acceleration_covariance.elems[COV::XX] = calculateCovariance(imu_acc_cov_xx, current_vel);
        m.linear_acceleration_covariance.elems[COV::YY] = calculateCovariance(imu_acc_cov_yy, current_vel);
        m.linear_acceleration_covariance.elems[COV::ZZ] = calculateCovariance(imu_acc_cov_zz, current_vel);

        m.angular_velocity.x = imu_in.xgyro;
        m.angular_velocity.y = imu_in.ygyro;
        m.angular_velocity.z = imu_in.zgyro;
        m.angular_velocity_covariance.elems[COV::XX] = calculateCovariance(imu_gyr_cov_xx, current_vel);
        m.angular_velocity_covariance.elems[COV::YY] = calculateCovariance(imu_gyr_cov_yy, current_vel);
        m.angular_velocity_covariance.elems[COV::ZZ] = calculateCovariance(imu_gyr_cov_zz, current_vel);

        // magnetometer doesn't work

        if(enable_imu_debug)
        {
          file_log << time_conv->convert_time(imu_in.timestamp) << ",";
          file_log << imu_in.xacc*gravity  << ",";
          file_log << imu_in.yacc*gravity  << ",";
          file_log << imu_in.zacc*gravity  << ",";
          file_log << imu_in.xgyro << ",";
          file_log << imu_in.ygyro << ",";
          file_log << imu_in.zgyro << ",";
          file_log << imu_in.xmag  << ",";
          file_log << imu_in.ymag  << ",";
          file_log << imu_in.zmag  << std::endl;
        }

        from_mav_imu_pub.publish(m);
        ROS_DEBUG("[drive_ros_mavlink_cc2016] Received a 'IMU' from mavlink.");
      } break;
      case MAVLINK_MSG_ID_ODOMETER_ABS: {
        drive_ros_msgs::mav_cc16_ODOMETER_ABS m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_odometer_abs_t odometer_abs_in;
        memset(&odometer_abs_in, 0, sizeof(odometer_abs_in));
        mavlink_msg_odometer_abs_decode(&mav_msg, &odometer_abs_in);

        m.header.stamp = time_conv->convert_time(odometer_abs_in.timestamp);
        m.header.frame_id = "rear_axis_middle";
        m.dist.x = odometer_abs_in.xdist;
        m.dist.y = odometer_abs_in.ydist;
        m.dist.z = odometer_abs_in.zdist;
        m.velocity.x = odometer_abs_in.xvelocity;
        m.velocity.y = odometer_abs_in.yvelocity;
        m.velocity.z = odometer_abs_in.zvelocity;
        m.quality = odometer_abs_in.quality;

        from_mav_odometer_abs_pub.publish(m);
        ROS_DEBUG("[drive_ros_mavlink_cc2016] Received a 'ODOMETER_ABS' from mavlink.");
      } break;
      case MAVLINK_MSG_ID_ODOMETER_RAW: {
        drive_ros_msgs::mav_cc16_ODOMETER_RAW m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_odometer_raw_t odometer_raw_in;
        memset(&odometer_raw_in, 0, sizeof(odometer_raw_in));
        mavlink_msg_odometer_raw_decode(&mav_msg, &odometer_raw_in);

        m.header.stamp = time_conv->convert_time(odometer_raw_in.timestamp);
        m.header.frame_id = "rear_axis_middle";
        m.dist.x = odometer_raw_in.xdist;
        m.dist.y = odometer_raw_in.ydist;
        m.dist.z = odometer_raw_in.zdist;
        m.quality = odometer_raw_in.quality;

        from_mav_odometer_raw_pub.publish(m);
        ROS_DEBUG("[drive_ros_mavlink_cc2016] Received a 'ODOMETER_RAW' from mavlink.");
      } break;
      case MAVLINK_MSG_ID_ODOMETER_DELTA: {
        drive_ros_msgs::VehicleEncoder m;
        drive_ros_msgs::EncoderLinear enc;


        mavlink_odometer_delta_t odometer_delta_in;
        memset(&odometer_delta_in, 0, sizeof(odometer_delta_in));
        mavlink_msg_odometer_delta_decode(&mav_msg, &odometer_delta_in);

        m.header.stamp = time_conv->convert_time(odometer_delta_in.timestamp);
        m.header.frame_id = "rear_axis_middle";

        m.encoder[drive_ros_msgs::VehicleEncoder::MOTOR].pos_rel = odometer_delta_in.xdist;
        m.encoder[drive_ros_msgs::VehicleEncoder::MOTOR].pos_rel_var = calculateCovariance(odo_pos_var, current_vel);

        m.encoder[drive_ros_msgs::VehicleEncoder::MOTOR].vel = odometer_delta_in.xvelocity;
        m.encoder[drive_ros_msgs::VehicleEncoder::MOTOR].vel_var = calculateCovariance(odo_vel_var, current_vel);

        // save current velocity for covariance calculation
        current_vel = odometer_delta_in.xvelocity;

        from_mav_odometer_delta_pub.publish(m);
        ROS_DEBUG("[drive_ros_mavlink_cc2016] Received a 'ODOMETER_DELTA' from mavlink.");
      } break;
      case MAVLINK_MSG_ID_ODOMETER_DELTA_RAW: {
        drive_ros_msgs::mav_cc16_ODOMETER_DELTA_RAW m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_odometer_delta_raw_t odometer_delta_raw_in;
        memset(&odometer_delta_raw_in, 0, sizeof(odometer_delta_raw_in));
        mavlink_msg_odometer_delta_raw_decode(&mav_msg, &odometer_delta_raw_in);

        m.header.stamp = time_conv->convert_time(odometer_delta_raw_in.timestamp);
        m.header.frame_id = "rear_axis_middle";
        m.delta = odometer_delta_raw_in.delta;
        m.dist.x = odometer_delta_raw_in.xdist;
        m.dist.y = odometer_delta_raw_in.ydist;
        m.dist.z = odometer_delta_raw_in.zdist;
        m.quality = odometer_delta_raw_in.quality;

        from_mav_odometer_delta_raw_pub.publish(m);
        ROS_DEBUG("[drive_ros_mavlink_cc2016] Received a 'ODOMETER_DELTA_RAW' from mavlink.");
      } break;
      case MAVLINK_MSG_ID_ODOMETER: {
        drive_ros_msgs::mav_cc16_ODOMETER m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_odometer_t odometer_in;
        memset(&odometer_in, 0, sizeof(odometer_in));
        mavlink_msg_odometer_decode(&mav_msg, &odometer_in);

        m.header.stamp = time_conv->convert_time(odometer_in.timestamp);
        m.header.frame_id = "rear_axis_middle";
        m.time_delta = odometer_in.time_delta;
        m.dist_delta.x = odometer_in.xdist_delta;
        m.dist_delta.y = odometer_in.ydist_delta;
        m.dist_delta.z = odometer_in.zdist_delta;
        m.dist_abs.x = odometer_in.xdist_abs;
        m.dist_abs.y = odometer_in.ydist_abs;
        m.dist_abs.z = odometer_in.zdist_abs;
        m.velocity.x = odometer_in.xvelocity;
        m.velocity.y = odometer_in.yvelocity;
        m.velocity.z = odometer_in.zvelocity;
        m.quality = odometer_in.quality;

        from_mav_odometer_pub.publish(m);
        ROS_DEBUG("[drive_ros_mavlink_cc2016] Received a 'ODOMETER' from mavlink.");
      } break;
      case MAVLINK_MSG_ID_PROXIMITY: {
        drive_ros_msgs::mav_cc16_PROXIMITY m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_proximity_t proximity_in;
        memset(&proximity_in, 0, sizeof(proximity_in));
        mavlink_msg_proximity_decode(&mav_msg, &proximity_in);

        m.header.stamp = time_conv->convert_time(proximity_in.timestamp);
        m.distance = proximity_in.distance;

        from_mav_proximity_pub.publish(m);
        ROS_DEBUG("[drive_ros_mavlink_cc2016] Received a 'PROXIMITY' from mavlink.");
      } break;
      case MAVLINK_MSG_ID_PARKING_LOT: {
        drive_ros_msgs::mav_cc16_PARKING_LOT m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_parking_lot_t parking_lot_in;
        memset(&parking_lot_in, 0, sizeof(parking_lot_in));
        mavlink_msg_parking_lot_decode(&mav_msg, &parking_lot_in);

        m.header.stamp = time_conv->convert_time(parking_lot_in.timestamp);
        m.header.frame_id = "side_lightswitch";
        m.parking_lot_size = parking_lot_in.parking_lot_size;
        m.parking_lot_position = parking_lot_in.parking_lot_position;

        from_mav_parking_lot_pub.publish(m);
        ROS_DEBUG("[drive_ros_mavlink_cc2016] Received a 'PARKING_LOT' from mavlink.");
      } break;
      case MAVLINK_MSG_ID_CONFIG_COUNT: {
        drive_ros_msgs::mav_cc16_CONFIG_COUNT m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_config_count_t config_count_in;
        memset(&config_count_in, 0, sizeof(config_count_in));
        mavlink_msg_config_count_decode(&mav_msg, &config_count_in);

        m.config_id_mask = config_count_in.config_id_mask;

        from_mav_config_count_pub.publish(m);
        ROS_DEBUG("[drive_ros_mavlink_cc2016] Received a 'CONFIG_COUNT' from mavlink.");
      } break;
      case MAVLINK_MSG_ID_CONFIG: {
        drive_ros_msgs::mav_cc16_CONFIG m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_config_t config_in;
        memset(&config_in, 0, sizeof(config_in));
        mavlink_msg_config_decode(&mav_msg, &config_in);

        m.config_id = config_in.config_id;
        memcpy(&(m.name), &(config_in.name), sizeof(char) * 30);
        m.param_id_mask = config_in.param_id_mask;

        from_mav_config_pub.publish(m);
        ROS_DEBUG("[drive_ros_mavlink_cc2016] Received a 'CONFIG' from mavlink.");
      } break;
      case MAVLINK_MSG_ID_CONFIG_PARAM_INT: {
        drive_ros_msgs::mav_cc16_CONFIG_PARAM_INT m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_config_param_int_t config_param_int_in;
        memset(&config_param_int_in, 0, sizeof(config_param_int_in));
        mavlink_msg_config_param_int_decode(&mav_msg, &config_param_int_in);

        m.config_id = config_param_int_in.config_id;
        m.param_id = config_param_int_in.param_id;
        memcpy(&(m.name), &(config_param_int_in.name), sizeof(char) * 30);
        m.value = config_param_int_in.value;
        m.default_value = config_param_int_in.default_value;
        m.min = config_param_int_in.min;
        m.max = config_param_int_in.max;

        from_mav_config_param_int_pub.publish(m);
        ROS_DEBUG("[drive_ros_mavlink_cc2016] Received a 'CONFIG_PARAM_INT' from mavlink.");
      } break;
      case MAVLINK_MSG_ID_CONFIG_PARAM_BOOL: {
        drive_ros_msgs::mav_cc16_CONFIG_PARAM_BOOL m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_config_param_bool_t config_param_bool_in;
        memset(&config_param_bool_in, 0, sizeof(config_param_bool_in));
        mavlink_msg_config_param_bool_decode(&mav_msg, &config_param_bool_in);

        m.config_id = config_param_bool_in.config_id;
        m.param_id = config_param_bool_in.param_id;
        memcpy(&(m.name), &(config_param_bool_in.name), sizeof(char) * 30);
        m.value = config_param_bool_in.value;
        m.default_value = config_param_bool_in.default_value;

        from_mav_config_param_bool_pub.publish(m);
        ROS_DEBUG("[drive_ros_mavlink_cc2016] Received a 'CONFIG_PARAM_BOOL' from mavlink.");
      } break;
      case MAVLINK_MSG_ID_CONFIG_PARAM_FLOAT: {
        drive_ros_msgs::mav_cc16_CONFIG_PARAM_FLOAT m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_config_param_float_t config_param_float_in;
        memset(&config_param_float_in, 0, sizeof(config_param_float_in));
        mavlink_msg_config_param_float_decode(&mav_msg, &config_param_float_in);

        m.config_id = config_param_float_in.config_id;
        m.param_id = config_param_float_in.param_id;
        memcpy(&(m.name), &(config_param_float_in.name), sizeof(char) * 30);
        m.value = config_param_float_in.value;
        m.default_value = config_param_float_in.default_value;
        m.min = config_param_float_in.min;
        m.max = config_param_float_in.max;

        from_mav_config_param_float_pub.publish(m);
        ROS_DEBUG("[drive_ros_mavlink_cc2016] Received a 'CONFIG_PARAM_FLOAT' from mavlink.");
      } break;
      case MAVLINK_MSG_ID_COMMAND: {
        drive_ros_msgs::mav_cc16_COMMAND m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_command_t command_in;
        memset(&command_in, 0, sizeof(command_in));
        mavlink_msg_command_decode(&mav_msg, &command_in);

        m.command = command_in.command;

        from_mav_command_pub.publish(m);
        ROS_DEBUG("[drive_ros_mavlink_cc2016] Received a 'COMMAND' from mavlink.");
      } break;
      default:
        // Do nothing
        break;
      }
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "drive_ros_mavlink_cc2016_node");
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");

  // debug publisher
  ros::Publisher debug_pub;

  // time conversion parameters
  {
    int reset_offset_us_int, comm_offset_us_int;
    pnh.param<int>("reset_offset", reset_offset_us_int, 5000);
    pnh.param<int>("comm_offset", comm_offset_us_int, 0);

    bool enable_time_debug_bool;
    pnh.param<bool>("enable_time_debug", enable_time_debug_bool, false);

    ROS_INFO_STREAM("set 'reset_offset' to: " << reset_offset_us_int);
    ROS_INFO_STREAM("set 'comm_offset' to: " << comm_offset_us_int);
    ROS_INFO_STREAM("set 'enable_time_debug' to: " << enable_time_debug_bool);

    if(enable_time_debug_bool)
    {
      debug_pub = n.advertise<drive_ros_msgs::TimeCompare>("/from_mav/debug_times", 10);
    }

    time_conv = new TimeConverter(static_cast<TimeConverter::usec_t>(reset_offset_us_int),
                                  static_cast<TimeConverter::usec_t>(comm_offset_us_int),
                                  enable_time_debug_bool,
                                  &debug_pub);
  }


  // imu debug
  {
    pnh.param<bool>("enable_imu_debug", enable_imu_debug, false);

    if(enable_imu_debug)
    {
      file_log.open("/tmp/out_imu.csv");

      file_log << "timestamp"  << ",";
      file_log << "xacc"  << ",";
      file_log << "yacc"  << ",";
      file_log << "zacc"  << ",";
      file_log << "xgyro" << ",";
      file_log << "ygyro" << ",";
      file_log << "zgyro" << ",";
      file_log << "xmag"  << ",";
      file_log << "ymag"  << ",";
      file_log << "zmag"  << std::endl;
    }
  }


  // sensor covariances
  {
    if( pnh.getParam("sensor_cov/odo_vel_var", odo_vel_var) &&
        pnh.getParam("sensor_cov/odo_pos_var", odo_pos_var) &&
        pnh.getParam("sensor_cov/imu_acc_cov_xx", imu_acc_cov_xx) &&
        pnh.getParam("sensor_cov/imu_acc_cov_yy", imu_acc_cov_yy) &&
        pnh.getParam("sensor_cov/imu_acc_cov_zz", imu_acc_cov_zz) &&
        pnh.getParam("sensor_cov/imu_gyr_cov_zz", imu_gyr_cov_xx) &&
        pnh.getParam("sensor_cov/imu_gyr_cov_zz", imu_gyr_cov_yy) &&
        pnh.getParam("sensor_cov/imu_gyr_cov_zz", imu_gyr_cov_zz))
    {
      ROS_INFO("Sensor covariances loaded successfully");
    }else{
      ROS_ERROR("Error loading sensor covariances!");
      throw std::runtime_error("Error loading parameters");
    }
  }


  to_mav_mav_raw_data_publisher =     n.advertise<drive_ros_msgs::mav_RAW_DATA>("/to_mav/mav_raw_data", 10);
  from_mav_mav_raw_data_subscriber =  n.subscribe("/from_mav/mav_raw_data", 10, from_mav_mav_raw_data_callback);

  /**
   * Messages Publishers Initialization
   */
  from_mav_notification_pub =       n.advertise<drive_ros_msgs::mav_cc16_NOTIFICATION>("/from_mav/notification", 10);
  from_mav_heartbeat_pub =          n.advertise<drive_ros_msgs::mav_cc16_HEARTBEAT>("/from_mav/heartbeat", 10);
  from_mav_debug_pub =              n.advertise<drive_ros_msgs::mav_cc16_DEBUG>("/from_mav/debug", 10);
  from_mav_telemetry_pub =          n.advertise<drive_ros_msgs::mav_cc16_TELEMETRY>("/from_mav/telemetry", 10);
  from_mav_odometer_abs_pub =       n.advertise<drive_ros_msgs::mav_cc16_ODOMETER_ABS>("/from_mav/odometer_abs", 10);
  from_mav_odometer_raw_pub =       n.advertise<drive_ros_msgs::mav_cc16_ODOMETER_RAW>("/from_mav/odometer_raw", 10);
  from_mav_odometer_delta_raw_pub = n.advertise<drive_ros_msgs::mav_cc16_ODOMETER_DELTA_RAW>("/from_mav/odometer_delta_raw", 10);
  from_mav_odometer_pub =           n.advertise<drive_ros_msgs::mav_cc16_ODOMETER>("/from_mav/odometer", 10);
  from_mav_proximity_pub =          n.advertise<drive_ros_msgs::mav_cc16_PROXIMITY>("/from_mav/proximity", 10);
  from_mav_parking_lot_pub =        n.advertise<drive_ros_msgs::mav_cc16_PARKING_LOT>("/from_mav/parking_lot", 10);
  from_mav_config_count_pub =       n.advertise<drive_ros_msgs::mav_cc16_CONFIG_COUNT>("/from_mav/config_count", 10);
  from_mav_config_pub =             n.advertise<drive_ros_msgs::mav_cc16_CONFIG>("/from_mav/config", 10);
  from_mav_config_param_int_pub =   n.advertise<drive_ros_msgs::mav_cc16_CONFIG_PARAM_INT>("/from_mav/config_param_int", 10);
  from_mav_config_param_bool_pub =  n.advertise<drive_ros_msgs::mav_cc16_CONFIG_PARAM_BOOL>("/from_mav/config_param_bool", 10);
  from_mav_config_param_float_pub = n.advertise<drive_ros_msgs::mav_cc16_CONFIG_PARAM_FLOAT>("/from_mav/config_param_float", 10);
  from_mav_command_pub =            n.advertise<drive_ros_msgs::mav_cc16_COMMAND>("/from_mav/command", 10);

  // special message types
  from_mav_imu_pub =                n.advertise<sensor_msgs::Imu>("/from_mav/imu", 10);
  from_mav_odometer_delta_pub =     n.advertise<drive_ros_msgs::VehicleEncoder>("/from_mav/odometer_delta", 10);


  /**
   * Messages Subscribers Declaration
   */
  ros::Subscriber to_mav_control_lights_sub =         n.subscribe("/to_mav/control_lights", 10, to_mav_control_lights_callback);
  ros::Subscriber to_mav_control_command_sub =        n.subscribe("/to_mav/control_command", 10, to_mav_control_command_callback);
  ros::Subscriber to_mav_config_request_count_sub =   n.subscribe("/to_mav/config_request_count", 10, to_mav_config_request_count_callback);
  ros::Subscriber to_mav_config_request_sub =         n.subscribe("/to_mav/config_request", 10, to_mav_config_request_callback);
  ros::Subscriber to_mav_config_request_params_sub =  n.subscribe("/to_mav/config_request_params", 10, to_mav_config_request_params_callback);
  ros::Subscriber to_mav_config_param_set_int_sub =   n.subscribe("/to_mav/config_param_set_int", 10, to_mav_config_param_set_int_callback);
  ros::Subscriber to_mav_config_param_set_bool_sub =  n.subscribe("/to_mav/config_param_set_bool", 10, to_mav_config_param_set_bool_callback);
  ros::Subscriber to_mav_config_param_set_float_sub = n.subscribe("/to_mav/config_param_set_float", 10, to_mav_config_param_set_float_callback);
  ros::Subscriber to_mav_command_sub =                n.subscribe("/to_mav/command", 10, to_mav_command_callback);

  ros::spin();
  return 0;
}
