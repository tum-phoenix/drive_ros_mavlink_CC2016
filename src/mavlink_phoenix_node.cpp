
/**
 *
 **/
#include <mavlink_phoenix/mavlink2ros.h>
#include <mavlink_phoenix/phoenix/mavlink.h>
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
ros::Publisher from_mav_control_lights_pub;
ros::Publisher from_mav_control_command_pub;
ros::Publisher from_mav_imu_pub;
ros::Publisher from_mav_odometer_abs_pub;
ros::Publisher from_mav_odometer_raw_pub;
ros::Publisher from_mav_odometer_delta_pub;
ros::Publisher from_mav_odometer_delta_raw_pub;
ros::Publisher from_mav_odometer_pub;
ros::Publisher from_mav_proximity_pub;
ros::Publisher from_mav_parking_lot_pub;
ros::Publisher from_mav_config_request_count_pub;
ros::Publisher from_mav_config_request_pub;
ros::Publisher from_mav_config_request_params_pub;
ros::Publisher from_mav_config_count_pub;
ros::Publisher from_mav_config_pub;
ros::Publisher from_mav_config_param_int_pub;
ros::Publisher from_mav_config_param_bool_pub;
ros::Publisher from_mav_config_param_float_pub;
ros::Publisher from_mav_config_param_set_int_pub;
ros::Publisher from_mav_config_param_set_bool_pub;
ros::Publisher from_mav_config_param_set_float_pub;
ros::Publisher from_mav_command_pub;

/**
 *
 **/
int write_to_mav(uint8_t *b, int sz) {

  mavlink_phoenix::MAV_RAW_DATA m;

  m.channel = mavlink_phoenix::MAV_RAW_DATA::CH_COMM0;
  m.data.assign(b, b + sz);
  int rc = m.data.size();
  to_mav_mav_raw_data_publisher.publish(m);
  ROS_INFO("Writen to MAV %d bytes", rc);
  return rc;
}

/**
 *
 */
void to_mav_notification_callback(
    const mavlink_phoenix::NOTIFICATION::ConstPtr &msg) {
  ROS_INFO("[mavlink_phoenix] Received a  'to_mav_NOTIFICATION request");

  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t m;

  m.sysid = msg->sysid;
  m.compid = msg->compid;

  mavlink_notification_t notification_out;

  /** ASSIGN FIELDS **/

  notification_out.timestamp = msg->timestamp;
  notification_out.type = msg->type;
  memcpy(&(notification_out.description), &(msg->description[0]),
         sizeof(char) * (int)(msg->description.size()));
  memcpy(&(notification_out.tag), &(msg->tag[0]),
         sizeof(char) * (int)(msg->tag.size()));

  mavlink_msg_notification_encode(msg->sysid, msg->compid, &m,
                                  &notification_out);
  uint16_t len = mavlink_msg_to_send_buffer(data, &m);
  write_to_mav(data, len);
}
/**
 *
 */
void to_mav_heartbeat_callback(
    const mavlink_phoenix::HEARTBEAT::ConstPtr &msg) {
  ROS_INFO("[mavlink_phoenix] Received a  'to_mav_HEARTBEAT request");

  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t m;

  m.sysid = msg->sysid;
  m.compid = msg->compid;

  mavlink_heartbeat_t heartbeat_out;

  /** ASSIGN FIELDS **/

  heartbeat_out.timestamp = msg->timestamp;
  heartbeat_out.battery_voltage = msg->battery_voltage;
  heartbeat_out.remote_control = msg->remote_control;
  heartbeat_out.drive_mode = msg->drive_mode;
  heartbeat_out.rc_velocity = msg->rc_velocity;
  heartbeat_out.rc_steering_front = msg->rc_steering_front;
  heartbeat_out.rc_steering_rear = msg->rc_steering_rear;

  mavlink_msg_heartbeat_encode(msg->sysid, msg->compid, &m, &heartbeat_out);
  uint16_t len = mavlink_msg_to_send_buffer(data, &m);
  write_to_mav(data, len);
}
/**
 *
 */
void to_mav_debug_callback(const mavlink_phoenix::DEBUG::ConstPtr &msg) {
  ROS_INFO("[mavlink_phoenix] Received a  'to_mav_DEBUG request");

  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t m;

  m.sysid = msg->sysid;
  m.compid = msg->compid;

  mavlink_debug_t debug_out;

  /** ASSIGN FIELDS **/

  debug_out.timestamp = msg->timestamp;
  memcpy(&(debug_out.data), &(msg->data[0]),
         sizeof(float) * (int)(msg->data.size()));

  mavlink_msg_debug_encode(msg->sysid, msg->compid, &m, &debug_out);
  uint16_t len = mavlink_msg_to_send_buffer(data, &m);
  write_to_mav(data, len);
}
/**
 *
 */
void to_mav_telemetry_callback(
    const mavlink_phoenix::TELEMETRY::ConstPtr &msg) {
  ROS_INFO("[mavlink_phoenix] Received a  'to_mav_TELEMETRY request");

  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t m;

  m.sysid = msg->sysid;
  m.compid = msg->compid;

  mavlink_telemetry_t telemetry_out;

  /** ASSIGN FIELDS **/

  telemetry_out.timestamp = msg->timestamp;
  telemetry_out.xacc = msg->xacc;
  telemetry_out.yacc = msg->yacc;
  telemetry_out.zacc = msg->zacc;
  telemetry_out.xgyro = msg->xgyro;
  telemetry_out.ygyro = msg->ygyro;
  telemetry_out.zgyro = msg->zgyro;
  telemetry_out.dist_front = msg->dist_front;
  telemetry_out.dist_rear = msg->dist_rear;
  telemetry_out.dist_side = msg->dist_side;
  telemetry_out.odom = msg->odom;
  telemetry_out.odom_accumulated = msg->odom_accumulated;
  telemetry_out.xmotion_front = msg->xmotion_front;
  telemetry_out.ymotion_front = msg->ymotion_front;
  telemetry_out.xmotion_rear = msg->xmotion_rear;
  telemetry_out.ymotion_rear = msg->ymotion_rear;
  telemetry_out.motion_front_quality = msg->motion_front_quality;
  telemetry_out.motion_rear_quality = msg->motion_rear_quality;
  telemetry_out.current_motor = msg->current_motor;
  telemetry_out.current_servo_front = msg->current_servo_front;
  telemetry_out.current_servo_rear = msg->current_servo_rear;
  telemetry_out.current_total = msg->current_total;
  telemetry_out.pwm_servo_front = msg->pwm_servo_front;
  telemetry_out.pwm_servo_rear = msg->pwm_servo_rear;
  telemetry_out.battery_voltage = msg->battery_voltage;
  telemetry_out.remote_control = msg->remote_control;
  telemetry_out.drive_mode = msg->drive_mode;

  mavlink_msg_telemetry_encode(msg->sysid, msg->compid, &m, &telemetry_out);
  uint16_t len = mavlink_msg_to_send_buffer(data, &m);
  write_to_mav(data, len);
}
/**
 *
 */
void to_mav_control_lights_callback(
    const mavlink_phoenix::CONTROL_LIGHTS::ConstPtr &msg) {
  ROS_INFO("[mavlink_phoenix] Received a  'to_mav_CONTROL_LIGHTS request");

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
    const mavlink_phoenix::CONTROL_COMMAND::ConstPtr &msg) {
  ROS_INFO("[mavlink_phoenix] Received a  'to_mav_CONTROL_COMMAND request");

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
void to_mav_imu_callback(const mavlink_phoenix::IMU::ConstPtr &msg) {
  ROS_INFO("[mavlink_phoenix] Received a  'to_mav_IMU request");

  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t m;

  m.sysid = msg->sysid;
  m.compid = msg->compid;

  mavlink_imu_t imu_out;

  /** ASSIGN FIELDS **/

  imu_out.timestamp = msg->timestamp;
  imu_out.xacc = msg->xacc;
  imu_out.yacc = msg->yacc;
  imu_out.zacc = msg->zacc;
  imu_out.xgyro = msg->xgyro;
  imu_out.ygyro = msg->ygyro;
  imu_out.zgyro = msg->zgyro;
  imu_out.xmag = msg->xmag;
  imu_out.ymag = msg->ymag;
  imu_out.zmag = msg->zmag;

  mavlink_msg_imu_encode(msg->sysid, msg->compid, &m, &imu_out);
  uint16_t len = mavlink_msg_to_send_buffer(data, &m);
  write_to_mav(data, len);
}
/**
 *
 */
void to_mav_odometer_abs_callback(
    const mavlink_phoenix::ODOMETER_ABS::ConstPtr &msg) {
  ROS_INFO("[mavlink_phoenix] Received a  'to_mav_ODOMETER_ABS request");

  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t m;

  m.sysid = msg->sysid;
  m.compid = msg->compid;

  mavlink_odometer_abs_t odometer_abs_out;

  /** ASSIGN FIELDS **/

  odometer_abs_out.timestamp = msg->timestamp;
  odometer_abs_out.xdist = msg->xdist;
  odometer_abs_out.ydist = msg->ydist;
  odometer_abs_out.zdist = msg->zdist;
  odometer_abs_out.xvelocity = msg->xvelocity;
  odometer_abs_out.yvelocity = msg->yvelocity;
  odometer_abs_out.zvelocity = msg->zvelocity;
  odometer_abs_out.quality = msg->quality;

  mavlink_msg_odometer_abs_encode(msg->sysid, msg->compid, &m,
                                  &odometer_abs_out);
  uint16_t len = mavlink_msg_to_send_buffer(data, &m);
  write_to_mav(data, len);
}
/**
 *
 */
void to_mav_odometer_raw_callback(
    const mavlink_phoenix::ODOMETER_RAW::ConstPtr &msg) {
  ROS_INFO("[mavlink_phoenix] Received a  'to_mav_ODOMETER_RAW request");

  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t m;

  m.sysid = msg->sysid;
  m.compid = msg->compid;

  mavlink_odometer_raw_t odometer_raw_out;

  /** ASSIGN FIELDS **/

  odometer_raw_out.timestamp = msg->timestamp;
  odometer_raw_out.xdist = msg->xdist;
  odometer_raw_out.ydist = msg->ydist;
  odometer_raw_out.zdist = msg->zdist;
  odometer_raw_out.quality = msg->quality;

  mavlink_msg_odometer_raw_encode(msg->sysid, msg->compid, &m,
                                  &odometer_raw_out);
  uint16_t len = mavlink_msg_to_send_buffer(data, &m);
  write_to_mav(data, len);
}
/**
 *
 */
void to_mav_odometer_delta_callback(
    const mavlink_phoenix::ODOMETER_DELTA::ConstPtr &msg) {
  ROS_INFO("[mavlink_phoenix] Received a  'to_mav_ODOMETER_DELTA request");

  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t m;

  m.sysid = msg->sysid;
  m.compid = msg->compid;

  mavlink_odometer_delta_t odometer_delta_out;

  /** ASSIGN FIELDS **/

  odometer_delta_out.timestamp = msg->timestamp;
  odometer_delta_out.delta = msg->delta;
  odometer_delta_out.xdist = msg->xdist;
  odometer_delta_out.ydist = msg->ydist;
  odometer_delta_out.zdist = msg->zdist;
  odometer_delta_out.xvelocity = msg->xvelocity;
  odometer_delta_out.yvelocity = msg->yvelocity;
  odometer_delta_out.zvelocity = msg->zvelocity;
  odometer_delta_out.quality = msg->quality;

  mavlink_msg_odometer_delta_encode(msg->sysid, msg->compid, &m,
                                    &odometer_delta_out);
  uint16_t len = mavlink_msg_to_send_buffer(data, &m);
  write_to_mav(data, len);
}
/**
 *
 */
void to_mav_odometer_delta_raw_callback(
    const mavlink_phoenix::ODOMETER_DELTA_RAW::ConstPtr &msg) {
  ROS_INFO("[mavlink_phoenix] Received a  'to_mav_ODOMETER_DELTA_RAW request");

  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t m;

  m.sysid = msg->sysid;
  m.compid = msg->compid;

  mavlink_odometer_delta_raw_t odometer_delta_raw_out;

  /** ASSIGN FIELDS **/

  odometer_delta_raw_out.timestamp = msg->timestamp;
  odometer_delta_raw_out.delta = msg->delta;
  odometer_delta_raw_out.xdist = msg->xdist;
  odometer_delta_raw_out.ydist = msg->ydist;
  odometer_delta_raw_out.zdist = msg->zdist;
  odometer_delta_raw_out.quality = msg->quality;

  mavlink_msg_odometer_delta_raw_encode(msg->sysid, msg->compid, &m,
                                        &odometer_delta_raw_out);
  uint16_t len = mavlink_msg_to_send_buffer(data, &m);
  write_to_mav(data, len);
}
/**
 *
 */
void to_mav_odometer_callback(const mavlink_phoenix::ODOMETER::ConstPtr &msg) {
  ROS_INFO("[mavlink_phoenix] Received a  'to_mav_ODOMETER request");

  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t m;

  m.sysid = msg->sysid;
  m.compid = msg->compid;

  mavlink_odometer_t odometer_out;

  /** ASSIGN FIELDS **/

  odometer_out.timestamp = msg->timestamp;
  odometer_out.time_delta = msg->time_delta;
  odometer_out.xdist_delta = msg->xdist_delta;
  odometer_out.ydist_delta = msg->ydist_delta;
  odometer_out.zdist_delta = msg->zdist_delta;
  odometer_out.xdist_abs = msg->xdist_abs;
  odometer_out.ydist_abs = msg->ydist_abs;
  odometer_out.zdist_abs = msg->zdist_abs;
  odometer_out.xvelocity = msg->xvelocity;
  odometer_out.yvelocity = msg->yvelocity;
  odometer_out.zvelocity = msg->zvelocity;
  odometer_out.quality = msg->quality;

  mavlink_msg_odometer_encode(msg->sysid, msg->compid, &m, &odometer_out);
  uint16_t len = mavlink_msg_to_send_buffer(data, &m);
  write_to_mav(data, len);
}
/**
 *
 */
void to_mav_proximity_callback(
    const mavlink_phoenix::PROXIMITY::ConstPtr &msg) {
  ROS_INFO("[mavlink_phoenix] Received a  'to_mav_PROXIMITY request");

  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t m;

  m.sysid = msg->sysid;
  m.compid = msg->compid;

  mavlink_proximity_t proximity_out;

  /** ASSIGN FIELDS **/

  proximity_out.timestamp = msg->timestamp;
  proximity_out.distance = msg->distance;

  mavlink_msg_proximity_encode(msg->sysid, msg->compid, &m, &proximity_out);
  uint16_t len = mavlink_msg_to_send_buffer(data, &m);
  write_to_mav(data, len);
}
/**
 *
 */
void to_mav_parking_lot_callback(
    const mavlink_phoenix::PARKING_LOT::ConstPtr &msg) {
  ROS_INFO("[mavlink_phoenix] Received a  'to_mav_PARKING_LOT request");

  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t m;

  m.sysid = msg->sysid;
  m.compid = msg->compid;

  mavlink_parking_lot_t parking_lot_out;

  /** ASSIGN FIELDS **/

  parking_lot_out.timestamp = msg->timestamp;
  parking_lot_out.parking_lot_size = msg->parking_lot_size;
  parking_lot_out.parking_lot_position = msg->parking_lot_position;

  mavlink_msg_parking_lot_encode(msg->sysid, msg->compid, &m, &parking_lot_out);
  uint16_t len = mavlink_msg_to_send_buffer(data, &m);
  write_to_mav(data, len);
}
/**
 *
 */
void to_mav_config_request_count_callback(
    const mavlink_phoenix::CONFIG_REQUEST_COUNT::ConstPtr &msg) {
  ROS_INFO(
      "[mavlink_phoenix] Received a  'to_mav_CONFIG_REQUEST_COUNT request");

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
    const mavlink_phoenix::CONFIG_REQUEST::ConstPtr &msg) {
  ROS_INFO("[mavlink_phoenix] Received a  'to_mav_CONFIG_REQUEST request");

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
    const mavlink_phoenix::CONFIG_REQUEST_PARAMS::ConstPtr &msg) {
  ROS_INFO(
      "[mavlink_phoenix] Received a  'to_mav_CONFIG_REQUEST_PARAMS request");

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
void to_mav_config_count_callback(
    const mavlink_phoenix::CONFIG_COUNT::ConstPtr &msg) {
  ROS_INFO("[mavlink_phoenix] Received a  'to_mav_CONFIG_COUNT request");

  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t m;

  m.sysid = msg->sysid;
  m.compid = msg->compid;

  mavlink_config_count_t config_count_out;

  /** ASSIGN FIELDS **/

  config_count_out.config_id_mask = msg->config_id_mask;

  mavlink_msg_config_count_encode(msg->sysid, msg->compid, &m,
                                  &config_count_out);
  uint16_t len = mavlink_msg_to_send_buffer(data, &m);
  write_to_mav(data, len);
}
/**
 *
 */
void to_mav_config_callback(const mavlink_phoenix::CONFIG::ConstPtr &msg) {
  ROS_INFO("[mavlink_phoenix] Received a  'to_mav_CONFIG request");

  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t m;

  m.sysid = msg->sysid;
  m.compid = msg->compid;

  mavlink_config_t config_out;

  /** ASSIGN FIELDS **/

  config_out.config_id = msg->config_id;
  memcpy(&(config_out.name), &(msg->name[0]),
         sizeof(char) * (int)(msg->name.size()));
  config_out.param_id_mask = msg->param_id_mask;

  mavlink_msg_config_encode(msg->sysid, msg->compid, &m, &config_out);
  uint16_t len = mavlink_msg_to_send_buffer(data, &m);
  write_to_mav(data, len);
}
/**
 *
 */
void to_mav_config_param_int_callback(
    const mavlink_phoenix::CONFIG_PARAM_INT::ConstPtr &msg) {
  ROS_INFO("[mavlink_phoenix] Received a  'to_mav_CONFIG_PARAM_INT request");

  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t m;

  m.sysid = msg->sysid;
  m.compid = msg->compid;

  mavlink_config_param_int_t config_param_int_out;

  /** ASSIGN FIELDS **/

  config_param_int_out.config_id = msg->config_id;
  config_param_int_out.param_id = msg->param_id;
  memcpy(&(config_param_int_out.name), &(msg->name[0]),
         sizeof(char) * (int)(msg->name.size()));
  config_param_int_out.value = msg->value;
  config_param_int_out.default_value = msg->default_value;
  config_param_int_out.min = msg->min;
  config_param_int_out.max = msg->max;

  mavlink_msg_config_param_int_encode(msg->sysid, msg->compid, &m,
                                      &config_param_int_out);
  uint16_t len = mavlink_msg_to_send_buffer(data, &m);
  write_to_mav(data, len);
}
/**
 *
 */
void to_mav_config_param_bool_callback(
    const mavlink_phoenix::CONFIG_PARAM_BOOL::ConstPtr &msg) {
  ROS_INFO("[mavlink_phoenix] Received a  'to_mav_CONFIG_PARAM_BOOL request");

  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t m;

  m.sysid = msg->sysid;
  m.compid = msg->compid;

  mavlink_config_param_bool_t config_param_bool_out;

  /** ASSIGN FIELDS **/

  config_param_bool_out.config_id = msg->config_id;
  config_param_bool_out.param_id = msg->param_id;
  memcpy(&(config_param_bool_out.name), &(msg->name[0]),
         sizeof(char) * (int)(msg->name.size()));
  config_param_bool_out.value = msg->value;
  config_param_bool_out.default_value = msg->default_value;

  mavlink_msg_config_param_bool_encode(msg->sysid, msg->compid, &m,
                                       &config_param_bool_out);
  uint16_t len = mavlink_msg_to_send_buffer(data, &m);
  write_to_mav(data, len);
}
/**
 *
 */
void to_mav_config_param_float_callback(
    const mavlink_phoenix::CONFIG_PARAM_FLOAT::ConstPtr &msg) {
  ROS_INFO("[mavlink_phoenix] Received a  'to_mav_CONFIG_PARAM_FLOAT request");

  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t m;

  m.sysid = msg->sysid;
  m.compid = msg->compid;

  mavlink_config_param_float_t config_param_float_out;

  /** ASSIGN FIELDS **/

  config_param_float_out.config_id = msg->config_id;
  config_param_float_out.param_id = msg->param_id;
  memcpy(&(config_param_float_out.name), &(msg->name[0]),
         sizeof(char) * (int)(msg->name.size()));
  config_param_float_out.value = msg->value;
  config_param_float_out.default_value = msg->default_value;
  config_param_float_out.min = msg->min;
  config_param_float_out.max = msg->max;

  mavlink_msg_config_param_float_encode(msg->sysid, msg->compid, &m,
                                        &config_param_float_out);
  uint16_t len = mavlink_msg_to_send_buffer(data, &m);
  write_to_mav(data, len);
}
/**
 *
 */
void to_mav_config_param_set_int_callback(
    const mavlink_phoenix::CONFIG_PARAM_SET_INT::ConstPtr &msg) {
  ROS_INFO(
      "[mavlink_phoenix] Received a  'to_mav_CONFIG_PARAM_SET_INT request");

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
    const mavlink_phoenix::CONFIG_PARAM_SET_BOOL::ConstPtr &msg) {
  ROS_INFO(
      "[mavlink_phoenix] Received a  'to_mav_CONFIG_PARAM_SET_BOOL request");

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
    const mavlink_phoenix::CONFIG_PARAM_SET_FLOAT::ConstPtr &msg) {
  ROS_INFO(
      "[mavlink_phoenix] Received a  'to_mav_CONFIG_PARAM_SET_FLOAT request");

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
void to_mav_command_callback(const mavlink_phoenix::COMMAND::ConstPtr &msg) {
  ROS_INFO("[mavlink_phoenix] Received a  'to_mav_COMMAND request");

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
    const mavlink_phoenix::MAV_RAW_DATA::ConstPtr &msg) {
  ROS_INFO(
      "[mavlink_phoenix] Received a  'from_mav_mav_raw_data_callback request");

  for (int i = 0; i < msg->data.size(); ++i) {
    // Try to get a new message
    if (mavlink_parse_char(msg->channel, msg->data[i], &mav_msg, &status)) {
      // Handle message
      switch (mav_msg.msgid) {

      case MAVLINK_MSG_ID_NOTIFICATION: {
        mavlink_phoenix::NOTIFICATION m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_notification_t notification_in;
        memset(&notification_in, 0, sizeof(notification_in));
        mavlink_msg_notification_decode(&mav_msg, &notification_in);

        m.timestamp = notification_in.timestamp;
        m.type = notification_in.type;
        memcpy(&(m.description), &(notification_in.description),
               sizeof(char) * 50);
        memcpy(&(m.tag), &(notification_in.tag), sizeof(char) * 30);

        from_mav_notification_pub.publish(m);
      } break;
      case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_phoenix::HEARTBEAT m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_heartbeat_t heartbeat_in;
        memset(&heartbeat_in, 0, sizeof(heartbeat_in));
        mavlink_msg_heartbeat_decode(&mav_msg, &heartbeat_in);

        m.timestamp = heartbeat_in.timestamp;
        m.battery_voltage = heartbeat_in.battery_voltage;
        m.remote_control = heartbeat_in.remote_control;
        m.drive_mode = heartbeat_in.drive_mode;
        m.rc_velocity = heartbeat_in.rc_velocity;
        m.rc_steering_front = heartbeat_in.rc_steering_front;
        m.rc_steering_rear = heartbeat_in.rc_steering_rear;

        from_mav_heartbeat_pub.publish(m);
      } break;
      case MAVLINK_MSG_ID_DEBUG: {
        mavlink_phoenix::DEBUG m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_debug_t debug_in;
        memset(&debug_in, 0, sizeof(debug_in));
        mavlink_msg_debug_decode(&mav_msg, &debug_in);

        m.timestamp = debug_in.timestamp;
        memcpy(&(m.data), &(debug_in.data), sizeof(float) * 12);

        from_mav_debug_pub.publish(m);
      } break;
      case MAVLINK_MSG_ID_TELEMETRY: {
        mavlink_phoenix::TELEMETRY m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_telemetry_t telemetry_in;
        memset(&telemetry_in, 0, sizeof(telemetry_in));
        mavlink_msg_telemetry_decode(&mav_msg, &telemetry_in);

        m.timestamp = telemetry_in.timestamp;
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
      } break;
      case MAVLINK_MSG_ID_CONTROL_LIGHTS: {
        mavlink_phoenix::CONTROL_LIGHTS m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_control_lights_t control_lights_in;
        memset(&control_lights_in, 0, sizeof(control_lights_in));
        mavlink_msg_control_lights_decode(&mav_msg, &control_lights_in);

        memcpy(&(m.colors), &(control_lights_in.colors), sizeof(uint32_t) * 15);

        from_mav_control_lights_pub.publish(m);
      } break;
      case MAVLINK_MSG_ID_CONTROL_COMMAND: {
        mavlink_phoenix::CONTROL_COMMAND m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_control_command_t control_command_in;
        memset(&control_command_in, 0, sizeof(control_command_in));
        mavlink_msg_control_command_decode(&mav_msg, &control_command_in);

        m.velocity = control_command_in.velocity;
        m.steering_front = control_command_in.steering_front;
        m.steering_rear = control_command_in.steering_rear;
        m.indicator_left = control_command_in.indicator_left;
        m.indicator_right = control_command_in.indicator_right;

        from_mav_control_command_pub.publish(m);
      } break;
      case MAVLINK_MSG_ID_IMU: {
        mavlink_phoenix::IMU m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_imu_t imu_in;
        memset(&imu_in, 0, sizeof(imu_in));
        mavlink_msg_imu_decode(&mav_msg, &imu_in);

        m.timestamp = imu_in.timestamp;
        m.xacc = imu_in.xacc;
        m.yacc = imu_in.yacc;
        m.zacc = imu_in.zacc;
        m.xgyro = imu_in.xgyro;
        m.ygyro = imu_in.ygyro;
        m.zgyro = imu_in.zgyro;
        m.xmag = imu_in.xmag;
        m.ymag = imu_in.ymag;
        m.zmag = imu_in.zmag;

        from_mav_imu_pub.publish(m);
      } break;
      case MAVLINK_MSG_ID_ODOMETER_ABS: {
        mavlink_phoenix::ODOMETER_ABS m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_odometer_abs_t odometer_abs_in;
        memset(&odometer_abs_in, 0, sizeof(odometer_abs_in));
        mavlink_msg_odometer_abs_decode(&mav_msg, &odometer_abs_in);

        m.timestamp = odometer_abs_in.timestamp;
        m.xdist = odometer_abs_in.xdist;
        m.ydist = odometer_abs_in.ydist;
        m.zdist = odometer_abs_in.zdist;
        m.xvelocity = odometer_abs_in.xvelocity;
        m.yvelocity = odometer_abs_in.yvelocity;
        m.zvelocity = odometer_abs_in.zvelocity;
        m.quality = odometer_abs_in.quality;

        from_mav_odometer_abs_pub.publish(m);
      } break;
      case MAVLINK_MSG_ID_ODOMETER_RAW: {
        mavlink_phoenix::ODOMETER_RAW m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_odometer_raw_t odometer_raw_in;
        memset(&odometer_raw_in, 0, sizeof(odometer_raw_in));
        mavlink_msg_odometer_raw_decode(&mav_msg, &odometer_raw_in);

        m.timestamp = odometer_raw_in.timestamp;
        m.xdist = odometer_raw_in.xdist;
        m.ydist = odometer_raw_in.ydist;
        m.zdist = odometer_raw_in.zdist;
        m.quality = odometer_raw_in.quality;

        from_mav_odometer_raw_pub.publish(m);
      } break;
      case MAVLINK_MSG_ID_ODOMETER_DELTA: {
        mavlink_phoenix::ODOMETER_DELTA m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_odometer_delta_t odometer_delta_in;
        memset(&odometer_delta_in, 0, sizeof(odometer_delta_in));
        mavlink_msg_odometer_delta_decode(&mav_msg, &odometer_delta_in);

        m.timestamp = odometer_delta_in.timestamp;
        m.delta = odometer_delta_in.delta;
        m.xdist = odometer_delta_in.xdist;
        m.ydist = odometer_delta_in.ydist;
        m.zdist = odometer_delta_in.zdist;
        m.xvelocity = odometer_delta_in.xvelocity;
        m.yvelocity = odometer_delta_in.yvelocity;
        m.zvelocity = odometer_delta_in.zvelocity;
        m.quality = odometer_delta_in.quality;

        from_mav_odometer_delta_pub.publish(m);
      } break;
      case MAVLINK_MSG_ID_ODOMETER_DELTA_RAW: {
        mavlink_phoenix::ODOMETER_DELTA_RAW m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_odometer_delta_raw_t odometer_delta_raw_in;
        memset(&odometer_delta_raw_in, 0, sizeof(odometer_delta_raw_in));
        mavlink_msg_odometer_delta_raw_decode(&mav_msg, &odometer_delta_raw_in);

        m.timestamp = odometer_delta_raw_in.timestamp;
        m.delta = odometer_delta_raw_in.delta;
        m.xdist = odometer_delta_raw_in.xdist;
        m.ydist = odometer_delta_raw_in.ydist;
        m.zdist = odometer_delta_raw_in.zdist;
        m.quality = odometer_delta_raw_in.quality;

        from_mav_odometer_delta_raw_pub.publish(m);
      } break;
      case MAVLINK_MSG_ID_ODOMETER: {
        mavlink_phoenix::ODOMETER m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_odometer_t odometer_in;
        memset(&odometer_in, 0, sizeof(odometer_in));
        mavlink_msg_odometer_decode(&mav_msg, &odometer_in);

        m.timestamp = odometer_in.timestamp;
        m.time_delta = odometer_in.time_delta;
        m.xdist_delta = odometer_in.xdist_delta;
        m.ydist_delta = odometer_in.ydist_delta;
        m.zdist_delta = odometer_in.zdist_delta;
        m.xdist_abs = odometer_in.xdist_abs;
        m.ydist_abs = odometer_in.ydist_abs;
        m.zdist_abs = odometer_in.zdist_abs;
        m.xvelocity = odometer_in.xvelocity;
        m.yvelocity = odometer_in.yvelocity;
        m.zvelocity = odometer_in.zvelocity;
        m.quality = odometer_in.quality;

        from_mav_odometer_pub.publish(m);
      } break;
      case MAVLINK_MSG_ID_PROXIMITY: {
        mavlink_phoenix::PROXIMITY m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_proximity_t proximity_in;
        memset(&proximity_in, 0, sizeof(proximity_in));
        mavlink_msg_proximity_decode(&mav_msg, &proximity_in);

        m.timestamp = proximity_in.timestamp;
        m.distance = proximity_in.distance;

        from_mav_proximity_pub.publish(m);
      } break;
      case MAVLINK_MSG_ID_PARKING_LOT: {
        mavlink_phoenix::PARKING_LOT m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_parking_lot_t parking_lot_in;
        memset(&parking_lot_in, 0, sizeof(parking_lot_in));
        mavlink_msg_parking_lot_decode(&mav_msg, &parking_lot_in);

        m.timestamp = parking_lot_in.timestamp;
        m.parking_lot_size = parking_lot_in.parking_lot_size;
        m.parking_lot_position = parking_lot_in.parking_lot_position;

        from_mav_parking_lot_pub.publish(m);
      } break;
      case MAVLINK_MSG_ID_CONFIG_REQUEST_COUNT: {
        mavlink_phoenix::CONFIG_REQUEST_COUNT m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_config_request_count_t config_request_count_in;
        memset(&config_request_count_in, 0, sizeof(config_request_count_in));
        mavlink_msg_config_request_count_decode(&mav_msg,
                                                &config_request_count_in);

        m.dummy = config_request_count_in.dummy;

        from_mav_config_request_count_pub.publish(m);
      } break;
      case MAVLINK_MSG_ID_CONFIG_REQUEST: {
        mavlink_phoenix::CONFIG_REQUEST m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_config_request_t config_request_in;
        memset(&config_request_in, 0, sizeof(config_request_in));
        mavlink_msg_config_request_decode(&mav_msg, &config_request_in);

        m.config_id = config_request_in.config_id;

        from_mav_config_request_pub.publish(m);
      } break;
      case MAVLINK_MSG_ID_CONFIG_REQUEST_PARAMS: {
        mavlink_phoenix::CONFIG_REQUEST_PARAMS m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_config_request_params_t config_request_params_in;
        memset(&config_request_params_in, 0, sizeof(config_request_params_in));
        mavlink_msg_config_request_params_decode(&mav_msg,
                                                 &config_request_params_in);

        m.config_id = config_request_params_in.config_id;
        m.param_id = config_request_params_in.param_id;

        from_mav_config_request_params_pub.publish(m);
      } break;
      case MAVLINK_MSG_ID_CONFIG_COUNT: {
        mavlink_phoenix::CONFIG_COUNT m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_config_count_t config_count_in;
        memset(&config_count_in, 0, sizeof(config_count_in));
        mavlink_msg_config_count_decode(&mav_msg, &config_count_in);

        m.config_id_mask = config_count_in.config_id_mask;

        from_mav_config_count_pub.publish(m);
      } break;
      case MAVLINK_MSG_ID_CONFIG: {
        mavlink_phoenix::CONFIG m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_config_t config_in;
        memset(&config_in, 0, sizeof(config_in));
        mavlink_msg_config_decode(&mav_msg, &config_in);

        m.config_id = config_in.config_id;
        memcpy(&(m.name), &(config_in.name), sizeof(char) * 30);
        m.param_id_mask = config_in.param_id_mask;

        from_mav_config_pub.publish(m);
      } break;
      case MAVLINK_MSG_ID_CONFIG_PARAM_INT: {
        mavlink_phoenix::CONFIG_PARAM_INT m;

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
      } break;
      case MAVLINK_MSG_ID_CONFIG_PARAM_BOOL: {
        mavlink_phoenix::CONFIG_PARAM_BOOL m;

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
      } break;
      case MAVLINK_MSG_ID_CONFIG_PARAM_FLOAT: {
        mavlink_phoenix::CONFIG_PARAM_FLOAT m;

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
      } break;
      case MAVLINK_MSG_ID_CONFIG_PARAM_SET_INT: {
        mavlink_phoenix::CONFIG_PARAM_SET_INT m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_config_param_set_int_t config_param_set_int_in;
        memset(&config_param_set_int_in, 0, sizeof(config_param_set_int_in));
        mavlink_msg_config_param_set_int_decode(&mav_msg,
                                                &config_param_set_int_in);

        m.config_id = config_param_set_int_in.config_id;
        m.param_id = config_param_set_int_in.param_id;
        m.value = config_param_set_int_in.value;

        from_mav_config_param_set_int_pub.publish(m);
      } break;
      case MAVLINK_MSG_ID_CONFIG_PARAM_SET_BOOL: {
        mavlink_phoenix::CONFIG_PARAM_SET_BOOL m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_config_param_set_bool_t config_param_set_bool_in;
        memset(&config_param_set_bool_in, 0, sizeof(config_param_set_bool_in));
        mavlink_msg_config_param_set_bool_decode(&mav_msg,
                                                 &config_param_set_bool_in);

        m.config_id = config_param_set_bool_in.config_id;
        m.param_id = config_param_set_bool_in.param_id;
        m.value = config_param_set_bool_in.value;

        from_mav_config_param_set_bool_pub.publish(m);
      } break;
      case MAVLINK_MSG_ID_CONFIG_PARAM_SET_FLOAT: {
        mavlink_phoenix::CONFIG_PARAM_SET_FLOAT m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_config_param_set_float_t config_param_set_float_in;
        memset(&config_param_set_float_in, 0,
               sizeof(config_param_set_float_in));
        mavlink_msg_config_param_set_float_decode(&mav_msg,
                                                  &config_param_set_float_in);

        m.config_id = config_param_set_float_in.config_id;
        m.param_id = config_param_set_float_in.param_id;
        m.value = config_param_set_float_in.value;

        from_mav_config_param_set_float_pub.publish(m);
      } break;
      case MAVLINK_MSG_ID_COMMAND: {
        mavlink_phoenix::COMMAND m;

        m.sysid = mav_msg.sysid;
        m.compid = mav_msg.compid;

        mavlink_command_t command_in;
        memset(&command_in, 0, sizeof(command_in));
        mavlink_msg_command_decode(&mav_msg, &command_in);

        m.command = command_in.command;

        from_mav_command_pub.publish(m);
      } break;
      default:
        // Do nothing
        break;
      }
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mavlink_phoenix_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);

  to_mav_mav_raw_data_publisher =
      n.advertise<mavlink_phoenix::MAV_RAW_DATA>("/to_mav_mav_raw_data", 10);
  from_mav_mav_raw_data_subscriber =
      n.subscribe("/from_mav_mav_raw_data", 10, from_mav_mav_raw_data_callback);

  /**
   * Messages Publishers Initialization
   */
  from_mav_notification_pub =
      n.advertise<mavlink_phoenix::NOTIFICATION>("/from_mav_notification", 10);
  from_mav_heartbeat_pub =
      n.advertise<mavlink_phoenix::HEARTBEAT>("/from_mav_heartbeat", 10);
  from_mav_debug_pub =
      n.advertise<mavlink_phoenix::DEBUG>("/from_mav_debug", 10);
  from_mav_telemetry_pub =
      n.advertise<mavlink_phoenix::TELEMETRY>("/from_mav_telemetry", 10);
  from_mav_control_lights_pub = n.advertise<mavlink_phoenix::CONTROL_LIGHTS>(
      "/from_mav_control_lights", 10);
  from_mav_control_command_pub = n.advertise<mavlink_phoenix::CONTROL_COMMAND>(
      "/from_mav_control_command", 10);
  from_mav_imu_pub = n.advertise<mavlink_phoenix::IMU>("/from_mav_imu", 10);
  from_mav_odometer_abs_pub =
      n.advertise<mavlink_phoenix::ODOMETER_ABS>("/from_mav_odometer_abs", 10);
  from_mav_odometer_raw_pub =
      n.advertise<mavlink_phoenix::ODOMETER_RAW>("/from_mav_odometer_raw", 10);
  from_mav_odometer_delta_pub = n.advertise<mavlink_phoenix::ODOMETER_DELTA>(
      "/from_mav_odometer_delta", 10);
  from_mav_odometer_delta_raw_pub =
      n.advertise<mavlink_phoenix::ODOMETER_DELTA_RAW>(
          "/from_mav_odometer_delta_raw", 10);
  from_mav_odometer_pub =
      n.advertise<mavlink_phoenix::ODOMETER>("/from_mav_odometer", 10);
  from_mav_proximity_pub =
      n.advertise<mavlink_phoenix::PROXIMITY>("/from_mav_proximity", 10);
  from_mav_parking_lot_pub =
      n.advertise<mavlink_phoenix::PARKING_LOT>("/from_mav_parking_lot", 10);
  from_mav_config_request_count_pub =
      n.advertise<mavlink_phoenix::CONFIG_REQUEST_COUNT>(
          "/from_mav_config_request_count", 10);
  from_mav_config_request_pub = n.advertise<mavlink_phoenix::CONFIG_REQUEST>(
      "/from_mav_config_request", 10);
  from_mav_config_request_params_pub =
      n.advertise<mavlink_phoenix::CONFIG_REQUEST_PARAMS>(
          "/from_mav_config_request_params", 10);
  from_mav_config_count_pub =
      n.advertise<mavlink_phoenix::CONFIG_COUNT>("/from_mav_config_count", 10);
  from_mav_config_pub =
      n.advertise<mavlink_phoenix::CONFIG>("/from_mav_config", 10);
  from_mav_config_param_int_pub =
      n.advertise<mavlink_phoenix::CONFIG_PARAM_INT>(
          "/from_mav_config_param_int", 10);
  from_mav_config_param_bool_pub =
      n.advertise<mavlink_phoenix::CONFIG_PARAM_BOOL>(
          "/from_mav_config_param_bool", 10);
  from_mav_config_param_float_pub =
      n.advertise<mavlink_phoenix::CONFIG_PARAM_FLOAT>(
          "/from_mav_config_param_float", 10);
  from_mav_config_param_set_int_pub =
      n.advertise<mavlink_phoenix::CONFIG_PARAM_SET_INT>(
          "/from_mav_config_param_set_int", 10);
  from_mav_config_param_set_bool_pub =
      n.advertise<mavlink_phoenix::CONFIG_PARAM_SET_BOOL>(
          "/from_mav_config_param_set_bool", 10);
  from_mav_config_param_set_float_pub =
      n.advertise<mavlink_phoenix::CONFIG_PARAM_SET_FLOAT>(
          "/from_mav_config_param_set_float", 10);
  from_mav_command_pub =
      n.advertise<mavlink_phoenix::COMMAND>("/from_mav_command", 10);

  /**
   * Messages Subscribers Declaration
   */
  ros::Subscriber to_mav_notification_sub =
      n.subscribe("/to_mav_notification", 10, to_mav_notification_callback);
  ros::Subscriber to_mav_heartbeat_sub =
      n.subscribe("/to_mav_heartbeat", 10, to_mav_heartbeat_callback);
  ros::Subscriber to_mav_debug_sub =
      n.subscribe("/to_mav_debug", 10, to_mav_debug_callback);
  ros::Subscriber to_mav_telemetry_sub =
      n.subscribe("/to_mav_telemetry", 10, to_mav_telemetry_callback);
  ros::Subscriber to_mav_control_lights_sub =
      n.subscribe("/to_mav_control_lights", 10, to_mav_control_lights_callback);
  ros::Subscriber to_mav_control_command_sub = n.subscribe(
      "/to_mav_control_command", 10, to_mav_control_command_callback);
  ros::Subscriber to_mav_imu_sub =
      n.subscribe("/to_mav_imu", 10, to_mav_imu_callback);
  ros::Subscriber to_mav_odometer_abs_sub =
      n.subscribe("/to_mav_odometer_abs", 10, to_mav_odometer_abs_callback);
  ros::Subscriber to_mav_odometer_raw_sub =
      n.subscribe("/to_mav_odometer_raw", 10, to_mav_odometer_raw_callback);
  ros::Subscriber to_mav_odometer_delta_sub =
      n.subscribe("/to_mav_odometer_delta", 10, to_mav_odometer_delta_callback);
  ros::Subscriber to_mav_odometer_delta_raw_sub = n.subscribe(
      "/to_mav_odometer_delta_raw", 10, to_mav_odometer_delta_raw_callback);
  ros::Subscriber to_mav_odometer_sub =
      n.subscribe("/to_mav_odometer", 10, to_mav_odometer_callback);
  ros::Subscriber to_mav_proximity_sub =
      n.subscribe("/to_mav_proximity", 10, to_mav_proximity_callback);
  ros::Subscriber to_mav_parking_lot_sub =
      n.subscribe("/to_mav_parking_lot", 10, to_mav_parking_lot_callback);
  ros::Subscriber to_mav_config_request_count_sub = n.subscribe(
      "/to_mav_config_request_count", 10, to_mav_config_request_count_callback);
  ros::Subscriber to_mav_config_request_sub =
      n.subscribe("/to_mav_config_request", 10, to_mav_config_request_callback);
  ros::Subscriber to_mav_config_request_params_sub =
      n.subscribe("/to_mav_config_request_params", 10,
                  to_mav_config_request_params_callback);
  ros::Subscriber to_mav_config_count_sub =
      n.subscribe("/to_mav_config_count", 10, to_mav_config_count_callback);
  ros::Subscriber to_mav_config_sub =
      n.subscribe("/to_mav_config", 10, to_mav_config_callback);
  ros::Subscriber to_mav_config_param_int_sub = n.subscribe(
      "/to_mav_config_param_int", 10, to_mav_config_param_int_callback);
  ros::Subscriber to_mav_config_param_bool_sub = n.subscribe(
      "/to_mav_config_param_bool", 10, to_mav_config_param_bool_callback);
  ros::Subscriber to_mav_config_param_float_sub = n.subscribe(
      "/to_mav_config_param_float", 10, to_mav_config_param_float_callback);
  ros::Subscriber to_mav_config_param_set_int_sub = n.subscribe(
      "/to_mav_config_param_set_int", 10, to_mav_config_param_set_int_callback);
  ros::Subscriber to_mav_config_param_set_bool_sub =
      n.subscribe("/to_mav_config_param_set_bool", 10,
                  to_mav_config_param_set_bool_callback);
  ros::Subscriber to_mav_config_param_set_float_sub =
      n.subscribe("/to_mav_config_param_set_float", 10,
                  to_mav_config_param_set_float_callback);
  ros::Subscriber to_mav_command_sub =
      n.subscribe("/to_mav_command", 10, to_mav_command_callback);

  ros::spin();
  return 0;
}
