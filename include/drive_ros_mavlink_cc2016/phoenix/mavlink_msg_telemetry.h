// MESSAGE TELEMETRY PACKING

#define MAVLINK_MSG_ID_TELEMETRY 3

typedef struct __mavlink_telemetry_t
{
 uint32_t timestamp; ///< Timestamp of the message (us since system start)
 float xacc; ///< Linear acceleration along x-axis [g]
 float yacc; ///< Linear acceleration along y-axis [g]
 float zacc; ///< Linear acceleration along z-axis [g]
 float xgyro; ///< Angular velocity around x-axis [rad/s]
 float ygyro; ///< Angular velocity around y-axis [rad/s]
 float zgyro; ///< Angular velocity around z-axis [rad/s]
 float dist_front; ///< Distance to obstacle (front) [m]
 float dist_rear; ///< Distance to obstacle (rear) [m]
 float dist_side; ///< Distance to obstacle (side) [m]
 float odom; ///< Wheel velocity [m/s]
 float odom_accumulated; ///< Accumulated hall measurements [m]
 float xmotion_front; ///< Vehcile velocity along x-axis (front) [m/s]
 float ymotion_front; ///< Vehcile velocity along y-axis (front) [m/s]
 float xmotion_rear; ///< Vehcile velocity along x-axis (rear) [m/s]
 float ymotion_rear; ///< Vehcile velocity along y-axis (rear) [m/s]
 int32_t current_motor; ///< Motor current [mA]
 uint16_t current_servo_front; ///< Current drained by Front Servo [mA]
 uint16_t current_servo_rear; ///< Current drained by Rear Servo [mA]
 uint16_t current_total; ///< Total current drained [mA]
 uint16_t pwm_servo_front; ///< Front Servo PWM
 uint16_t pwm_servo_rear; ///< Rear Servo PWM
 uint16_t battery_voltage; ///< Battery voltage [mV]
 uint8_t motion_front_quality; ///< Front motion quality
 uint8_t motion_rear_quality; ///< Rear motion quality
 uint8_t remote_control; ///< Remote Control status (see REMOTE_CONTROL_STATUS ENUM)
 uint8_t drive_mode; ///< Selected drive mode (see DRIVE_MODE ENUM)
} mavlink_telemetry_t;

#define MAVLINK_MSG_ID_TELEMETRY_LEN 84
#define MAVLINK_MSG_ID_3_LEN 84



#define MAVLINK_MESSAGE_INFO_TELEMETRY { \
	"TELEMETRY", \
	27, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_telemetry_t, timestamp) }, \
         { "xacc", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_telemetry_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_telemetry_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_telemetry_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_telemetry_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_telemetry_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_telemetry_t, zgyro) }, \
         { "dist_front", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_telemetry_t, dist_front) }, \
         { "dist_rear", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_telemetry_t, dist_rear) }, \
         { "dist_side", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_telemetry_t, dist_side) }, \
         { "odom", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_telemetry_t, odom) }, \
         { "odom_accumulated", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_telemetry_t, odom_accumulated) }, \
         { "xmotion_front", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_telemetry_t, xmotion_front) }, \
         { "ymotion_front", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_telemetry_t, ymotion_front) }, \
         { "xmotion_rear", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_telemetry_t, xmotion_rear) }, \
         { "ymotion_rear", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_telemetry_t, ymotion_rear) }, \
         { "current_motor", NULL, MAVLINK_TYPE_INT32_T, 0, 64, offsetof(mavlink_telemetry_t, current_motor) }, \
         { "current_servo_front", NULL, MAVLINK_TYPE_UINT16_T, 0, 68, offsetof(mavlink_telemetry_t, current_servo_front) }, \
         { "current_servo_rear", NULL, MAVLINK_TYPE_UINT16_T, 0, 70, offsetof(mavlink_telemetry_t, current_servo_rear) }, \
         { "current_total", NULL, MAVLINK_TYPE_UINT16_T, 0, 72, offsetof(mavlink_telemetry_t, current_total) }, \
         { "pwm_servo_front", NULL, MAVLINK_TYPE_UINT16_T, 0, 74, offsetof(mavlink_telemetry_t, pwm_servo_front) }, \
         { "pwm_servo_rear", NULL, MAVLINK_TYPE_UINT16_T, 0, 76, offsetof(mavlink_telemetry_t, pwm_servo_rear) }, \
         { "battery_voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 78, offsetof(mavlink_telemetry_t, battery_voltage) }, \
         { "motion_front_quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 80, offsetof(mavlink_telemetry_t, motion_front_quality) }, \
         { "motion_rear_quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 81, offsetof(mavlink_telemetry_t, motion_rear_quality) }, \
         { "remote_control", NULL, MAVLINK_TYPE_UINT8_T, 0, 82, offsetof(mavlink_telemetry_t, remote_control) }, \
         { "drive_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 83, offsetof(mavlink_telemetry_t, drive_mode) }, \
         } \
}


/**
 * @brief Pack a telemetry message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp Timestamp of the message (us since system start)
 * @param xacc Linear acceleration along x-axis [g]
 * @param yacc Linear acceleration along y-axis [g]
 * @param zacc Linear acceleration along z-axis [g]
 * @param xgyro Angular velocity around x-axis [rad/s]
 * @param ygyro Angular velocity around y-axis [rad/s]
 * @param zgyro Angular velocity around z-axis [rad/s]
 * @param dist_front Distance to obstacle (front) [m]
 * @param dist_rear Distance to obstacle (rear) [m]
 * @param dist_side Distance to obstacle (side) [m]
 * @param odom Wheel velocity [m/s]
 * @param odom_accumulated Accumulated hall measurements [m]
 * @param xmotion_front Vehcile velocity along x-axis (front) [m/s]
 * @param ymotion_front Vehcile velocity along y-axis (front) [m/s]
 * @param xmotion_rear Vehcile velocity along x-axis (rear) [m/s]
 * @param ymotion_rear Vehcile velocity along y-axis (rear) [m/s]
 * @param motion_front_quality Front motion quality
 * @param motion_rear_quality Rear motion quality
 * @param current_motor Motor current [mA]
 * @param current_servo_front Current drained by Front Servo [mA]
 * @param current_servo_rear Current drained by Rear Servo [mA]
 * @param current_total Total current drained [mA]
 * @param pwm_servo_front Front Servo PWM
 * @param pwm_servo_rear Rear Servo PWM
 * @param battery_voltage Battery voltage [mV]
 * @param remote_control Remote Control status (see REMOTE_CONTROL_STATUS ENUM)
 * @param drive_mode Selected drive mode (see DRIVE_MODE ENUM)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_telemetry_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t timestamp, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float dist_front, float dist_rear, float dist_side, float odom, float odom_accumulated, float xmotion_front, float ymotion_front, float xmotion_rear, float ymotion_rear, uint8_t motion_front_quality, uint8_t motion_rear_quality, int32_t current_motor, uint16_t current_servo_front, uint16_t current_servo_rear, uint16_t current_total, uint16_t pwm_servo_front, uint16_t pwm_servo_rear, uint16_t battery_voltage, uint8_t remote_control, uint8_t drive_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[84];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float(buf, 4, xacc);
	_mav_put_float(buf, 8, yacc);
	_mav_put_float(buf, 12, zacc);
	_mav_put_float(buf, 16, xgyro);
	_mav_put_float(buf, 20, ygyro);
	_mav_put_float(buf, 24, zgyro);
	_mav_put_float(buf, 28, dist_front);
	_mav_put_float(buf, 32, dist_rear);
	_mav_put_float(buf, 36, dist_side);
	_mav_put_float(buf, 40, odom);
	_mav_put_float(buf, 44, odom_accumulated);
	_mav_put_float(buf, 48, xmotion_front);
	_mav_put_float(buf, 52, ymotion_front);
	_mav_put_float(buf, 56, xmotion_rear);
	_mav_put_float(buf, 60, ymotion_rear);
	_mav_put_int32_t(buf, 64, current_motor);
	_mav_put_uint16_t(buf, 68, current_servo_front);
	_mav_put_uint16_t(buf, 70, current_servo_rear);
	_mav_put_uint16_t(buf, 72, current_total);
	_mav_put_uint16_t(buf, 74, pwm_servo_front);
	_mav_put_uint16_t(buf, 76, pwm_servo_rear);
	_mav_put_uint16_t(buf, 78, battery_voltage);
	_mav_put_uint8_t(buf, 80, motion_front_quality);
	_mav_put_uint8_t(buf, 81, motion_rear_quality);
	_mav_put_uint8_t(buf, 82, remote_control);
	_mav_put_uint8_t(buf, 83, drive_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 84);
#else
	mavlink_telemetry_t packet;
	packet.timestamp = timestamp;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.dist_front = dist_front;
	packet.dist_rear = dist_rear;
	packet.dist_side = dist_side;
	packet.odom = odom;
	packet.odom_accumulated = odom_accumulated;
	packet.xmotion_front = xmotion_front;
	packet.ymotion_front = ymotion_front;
	packet.xmotion_rear = xmotion_rear;
	packet.ymotion_rear = ymotion_rear;
	packet.current_motor = current_motor;
	packet.current_servo_front = current_servo_front;
	packet.current_servo_rear = current_servo_rear;
	packet.current_total = current_total;
	packet.pwm_servo_front = pwm_servo_front;
	packet.pwm_servo_rear = pwm_servo_rear;
	packet.battery_voltage = battery_voltage;
	packet.motion_front_quality = motion_front_quality;
	packet.motion_rear_quality = motion_rear_quality;
	packet.remote_control = remote_control;
	packet.drive_mode = drive_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 84);
#endif

	msg->msgid = MAVLINK_MSG_ID_TELEMETRY;
	return mavlink_finalize_message(msg, system_id, component_id, 84, 210);
}

/**
 * @brief Pack a telemetry message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp of the message (us since system start)
 * @param xacc Linear acceleration along x-axis [g]
 * @param yacc Linear acceleration along y-axis [g]
 * @param zacc Linear acceleration along z-axis [g]
 * @param xgyro Angular velocity around x-axis [rad/s]
 * @param ygyro Angular velocity around y-axis [rad/s]
 * @param zgyro Angular velocity around z-axis [rad/s]
 * @param dist_front Distance to obstacle (front) [m]
 * @param dist_rear Distance to obstacle (rear) [m]
 * @param dist_side Distance to obstacle (side) [m]
 * @param odom Wheel velocity [m/s]
 * @param odom_accumulated Accumulated hall measurements [m]
 * @param xmotion_front Vehcile velocity along x-axis (front) [m/s]
 * @param ymotion_front Vehcile velocity along y-axis (front) [m/s]
 * @param xmotion_rear Vehcile velocity along x-axis (rear) [m/s]
 * @param ymotion_rear Vehcile velocity along y-axis (rear) [m/s]
 * @param motion_front_quality Front motion quality
 * @param motion_rear_quality Rear motion quality
 * @param current_motor Motor current [mA]
 * @param current_servo_front Current drained by Front Servo [mA]
 * @param current_servo_rear Current drained by Rear Servo [mA]
 * @param current_total Total current drained [mA]
 * @param pwm_servo_front Front Servo PWM
 * @param pwm_servo_rear Rear Servo PWM
 * @param battery_voltage Battery voltage [mV]
 * @param remote_control Remote Control status (see REMOTE_CONTROL_STATUS ENUM)
 * @param drive_mode Selected drive mode (see DRIVE_MODE ENUM)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_telemetry_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t timestamp,float xacc,float yacc,float zacc,float xgyro,float ygyro,float zgyro,float dist_front,float dist_rear,float dist_side,float odom,float odom_accumulated,float xmotion_front,float ymotion_front,float xmotion_rear,float ymotion_rear,uint8_t motion_front_quality,uint8_t motion_rear_quality,int32_t current_motor,uint16_t current_servo_front,uint16_t current_servo_rear,uint16_t current_total,uint16_t pwm_servo_front,uint16_t pwm_servo_rear,uint16_t battery_voltage,uint8_t remote_control,uint8_t drive_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[84];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float(buf, 4, xacc);
	_mav_put_float(buf, 8, yacc);
	_mav_put_float(buf, 12, zacc);
	_mav_put_float(buf, 16, xgyro);
	_mav_put_float(buf, 20, ygyro);
	_mav_put_float(buf, 24, zgyro);
	_mav_put_float(buf, 28, dist_front);
	_mav_put_float(buf, 32, dist_rear);
	_mav_put_float(buf, 36, dist_side);
	_mav_put_float(buf, 40, odom);
	_mav_put_float(buf, 44, odom_accumulated);
	_mav_put_float(buf, 48, xmotion_front);
	_mav_put_float(buf, 52, ymotion_front);
	_mav_put_float(buf, 56, xmotion_rear);
	_mav_put_float(buf, 60, ymotion_rear);
	_mav_put_int32_t(buf, 64, current_motor);
	_mav_put_uint16_t(buf, 68, current_servo_front);
	_mav_put_uint16_t(buf, 70, current_servo_rear);
	_mav_put_uint16_t(buf, 72, current_total);
	_mav_put_uint16_t(buf, 74, pwm_servo_front);
	_mav_put_uint16_t(buf, 76, pwm_servo_rear);
	_mav_put_uint16_t(buf, 78, battery_voltage);
	_mav_put_uint8_t(buf, 80, motion_front_quality);
	_mav_put_uint8_t(buf, 81, motion_rear_quality);
	_mav_put_uint8_t(buf, 82, remote_control);
	_mav_put_uint8_t(buf, 83, drive_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 84);
#else
	mavlink_telemetry_t packet;
	packet.timestamp = timestamp;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.dist_front = dist_front;
	packet.dist_rear = dist_rear;
	packet.dist_side = dist_side;
	packet.odom = odom;
	packet.odom_accumulated = odom_accumulated;
	packet.xmotion_front = xmotion_front;
	packet.ymotion_front = ymotion_front;
	packet.xmotion_rear = xmotion_rear;
	packet.ymotion_rear = ymotion_rear;
	packet.current_motor = current_motor;
	packet.current_servo_front = current_servo_front;
	packet.current_servo_rear = current_servo_rear;
	packet.current_total = current_total;
	packet.pwm_servo_front = pwm_servo_front;
	packet.pwm_servo_rear = pwm_servo_rear;
	packet.battery_voltage = battery_voltage;
	packet.motion_front_quality = motion_front_quality;
	packet.motion_rear_quality = motion_rear_quality;
	packet.remote_control = remote_control;
	packet.drive_mode = drive_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 84);
#endif

	msg->msgid = MAVLINK_MSG_ID_TELEMETRY;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 84, 210);
}

/**
 * @brief Encode a telemetry struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param telemetry C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_telemetry_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_telemetry_t* telemetry)
{
	return mavlink_msg_telemetry_pack(system_id, component_id, msg, telemetry->timestamp, telemetry->xacc, telemetry->yacc, telemetry->zacc, telemetry->xgyro, telemetry->ygyro, telemetry->zgyro, telemetry->dist_front, telemetry->dist_rear, telemetry->dist_side, telemetry->odom, telemetry->odom_accumulated, telemetry->xmotion_front, telemetry->ymotion_front, telemetry->xmotion_rear, telemetry->ymotion_rear, telemetry->motion_front_quality, telemetry->motion_rear_quality, telemetry->current_motor, telemetry->current_servo_front, telemetry->current_servo_rear, telemetry->current_total, telemetry->pwm_servo_front, telemetry->pwm_servo_rear, telemetry->battery_voltage, telemetry->remote_control, telemetry->drive_mode);
}

/**
 * @brief Send a telemetry message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp of the message (us since system start)
 * @param xacc Linear acceleration along x-axis [g]
 * @param yacc Linear acceleration along y-axis [g]
 * @param zacc Linear acceleration along z-axis [g]
 * @param xgyro Angular velocity around x-axis [rad/s]
 * @param ygyro Angular velocity around y-axis [rad/s]
 * @param zgyro Angular velocity around z-axis [rad/s]
 * @param dist_front Distance to obstacle (front) [m]
 * @param dist_rear Distance to obstacle (rear) [m]
 * @param dist_side Distance to obstacle (side) [m]
 * @param odom Wheel velocity [m/s]
 * @param odom_accumulated Accumulated hall measurements [m]
 * @param xmotion_front Vehcile velocity along x-axis (front) [m/s]
 * @param ymotion_front Vehcile velocity along y-axis (front) [m/s]
 * @param xmotion_rear Vehcile velocity along x-axis (rear) [m/s]
 * @param ymotion_rear Vehcile velocity along y-axis (rear) [m/s]
 * @param motion_front_quality Front motion quality
 * @param motion_rear_quality Rear motion quality
 * @param current_motor Motor current [mA]
 * @param current_servo_front Current drained by Front Servo [mA]
 * @param current_servo_rear Current drained by Rear Servo [mA]
 * @param current_total Total current drained [mA]
 * @param pwm_servo_front Front Servo PWM
 * @param pwm_servo_rear Rear Servo PWM
 * @param battery_voltage Battery voltage [mV]
 * @param remote_control Remote Control status (see REMOTE_CONTROL_STATUS ENUM)
 * @param drive_mode Selected drive mode (see DRIVE_MODE ENUM)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_telemetry_send(mavlink_channel_t chan, uint32_t timestamp, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float dist_front, float dist_rear, float dist_side, float odom, float odom_accumulated, float xmotion_front, float ymotion_front, float xmotion_rear, float ymotion_rear, uint8_t motion_front_quality, uint8_t motion_rear_quality, int32_t current_motor, uint16_t current_servo_front, uint16_t current_servo_rear, uint16_t current_total, uint16_t pwm_servo_front, uint16_t pwm_servo_rear, uint16_t battery_voltage, uint8_t remote_control, uint8_t drive_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[84];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float(buf, 4, xacc);
	_mav_put_float(buf, 8, yacc);
	_mav_put_float(buf, 12, zacc);
	_mav_put_float(buf, 16, xgyro);
	_mav_put_float(buf, 20, ygyro);
	_mav_put_float(buf, 24, zgyro);
	_mav_put_float(buf, 28, dist_front);
	_mav_put_float(buf, 32, dist_rear);
	_mav_put_float(buf, 36, dist_side);
	_mav_put_float(buf, 40, odom);
	_mav_put_float(buf, 44, odom_accumulated);
	_mav_put_float(buf, 48, xmotion_front);
	_mav_put_float(buf, 52, ymotion_front);
	_mav_put_float(buf, 56, xmotion_rear);
	_mav_put_float(buf, 60, ymotion_rear);
	_mav_put_int32_t(buf, 64, current_motor);
	_mav_put_uint16_t(buf, 68, current_servo_front);
	_mav_put_uint16_t(buf, 70, current_servo_rear);
	_mav_put_uint16_t(buf, 72, current_total);
	_mav_put_uint16_t(buf, 74, pwm_servo_front);
	_mav_put_uint16_t(buf, 76, pwm_servo_rear);
	_mav_put_uint16_t(buf, 78, battery_voltage);
	_mav_put_uint8_t(buf, 80, motion_front_quality);
	_mav_put_uint8_t(buf, 81, motion_rear_quality);
	_mav_put_uint8_t(buf, 82, remote_control);
	_mav_put_uint8_t(buf, 83, drive_mode);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TELEMETRY, buf, 84, 210);
#else
	mavlink_telemetry_t packet;
	packet.timestamp = timestamp;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.dist_front = dist_front;
	packet.dist_rear = dist_rear;
	packet.dist_side = dist_side;
	packet.odom = odom;
	packet.odom_accumulated = odom_accumulated;
	packet.xmotion_front = xmotion_front;
	packet.ymotion_front = ymotion_front;
	packet.xmotion_rear = xmotion_rear;
	packet.ymotion_rear = ymotion_rear;
	packet.current_motor = current_motor;
	packet.current_servo_front = current_servo_front;
	packet.current_servo_rear = current_servo_rear;
	packet.current_total = current_total;
	packet.pwm_servo_front = pwm_servo_front;
	packet.pwm_servo_rear = pwm_servo_rear;
	packet.battery_voltage = battery_voltage;
	packet.motion_front_quality = motion_front_quality;
	packet.motion_rear_quality = motion_rear_quality;
	packet.remote_control = remote_control;
	packet.drive_mode = drive_mode;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TELEMETRY, (const char *)&packet, 84, 210);
#endif
}

#endif

// MESSAGE TELEMETRY UNPACKING


/**
 * @brief Get field timestamp from telemetry message
 *
 * @return Timestamp of the message (us since system start)
 */
static inline uint32_t mavlink_msg_telemetry_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field xacc from telemetry message
 *
 * @return Linear acceleration along x-axis [g]
 */
static inline float mavlink_msg_telemetry_get_xacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field yacc from telemetry message
 *
 * @return Linear acceleration along y-axis [g]
 */
static inline float mavlink_msg_telemetry_get_yacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field zacc from telemetry message
 *
 * @return Linear acceleration along z-axis [g]
 */
static inline float mavlink_msg_telemetry_get_zacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field xgyro from telemetry message
 *
 * @return Angular velocity around x-axis [rad/s]
 */
static inline float mavlink_msg_telemetry_get_xgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field ygyro from telemetry message
 *
 * @return Angular velocity around y-axis [rad/s]
 */
static inline float mavlink_msg_telemetry_get_ygyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field zgyro from telemetry message
 *
 * @return Angular velocity around z-axis [rad/s]
 */
static inline float mavlink_msg_telemetry_get_zgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field dist_front from telemetry message
 *
 * @return Distance to obstacle (front) [m]
 */
static inline float mavlink_msg_telemetry_get_dist_front(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field dist_rear from telemetry message
 *
 * @return Distance to obstacle (rear) [m]
 */
static inline float mavlink_msg_telemetry_get_dist_rear(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field dist_side from telemetry message
 *
 * @return Distance to obstacle (side) [m]
 */
static inline float mavlink_msg_telemetry_get_dist_side(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field odom from telemetry message
 *
 * @return Wheel velocity [m/s]
 */
static inline float mavlink_msg_telemetry_get_odom(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field odom_accumulated from telemetry message
 *
 * @return Accumulated hall measurements [m]
 */
static inline float mavlink_msg_telemetry_get_odom_accumulated(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field xmotion_front from telemetry message
 *
 * @return Vehcile velocity along x-axis (front) [m/s]
 */
static inline float mavlink_msg_telemetry_get_xmotion_front(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field ymotion_front from telemetry message
 *
 * @return Vehcile velocity along y-axis (front) [m/s]
 */
static inline float mavlink_msg_telemetry_get_ymotion_front(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field xmotion_rear from telemetry message
 *
 * @return Vehcile velocity along x-axis (rear) [m/s]
 */
static inline float mavlink_msg_telemetry_get_xmotion_rear(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field ymotion_rear from telemetry message
 *
 * @return Vehcile velocity along y-axis (rear) [m/s]
 */
static inline float mavlink_msg_telemetry_get_ymotion_rear(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field motion_front_quality from telemetry message
 *
 * @return Front motion quality
 */
static inline uint8_t mavlink_msg_telemetry_get_motion_front_quality(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  80);
}

/**
 * @brief Get field motion_rear_quality from telemetry message
 *
 * @return Rear motion quality
 */
static inline uint8_t mavlink_msg_telemetry_get_motion_rear_quality(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  81);
}

/**
 * @brief Get field current_motor from telemetry message
 *
 * @return Motor current [mA]
 */
static inline int32_t mavlink_msg_telemetry_get_current_motor(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  64);
}

/**
 * @brief Get field current_servo_front from telemetry message
 *
 * @return Current drained by Front Servo [mA]
 */
static inline uint16_t mavlink_msg_telemetry_get_current_servo_front(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  68);
}

/**
 * @brief Get field current_servo_rear from telemetry message
 *
 * @return Current drained by Rear Servo [mA]
 */
static inline uint16_t mavlink_msg_telemetry_get_current_servo_rear(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  70);
}

/**
 * @brief Get field current_total from telemetry message
 *
 * @return Total current drained [mA]
 */
static inline uint16_t mavlink_msg_telemetry_get_current_total(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  72);
}

/**
 * @brief Get field pwm_servo_front from telemetry message
 *
 * @return Front Servo PWM
 */
static inline uint16_t mavlink_msg_telemetry_get_pwm_servo_front(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  74);
}

/**
 * @brief Get field pwm_servo_rear from telemetry message
 *
 * @return Rear Servo PWM
 */
static inline uint16_t mavlink_msg_telemetry_get_pwm_servo_rear(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  76);
}

/**
 * @brief Get field battery_voltage from telemetry message
 *
 * @return Battery voltage [mV]
 */
static inline uint16_t mavlink_msg_telemetry_get_battery_voltage(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  78);
}

/**
 * @brief Get field remote_control from telemetry message
 *
 * @return Remote Control status (see REMOTE_CONTROL_STATUS ENUM)
 */
static inline uint8_t mavlink_msg_telemetry_get_remote_control(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  82);
}

/**
 * @brief Get field drive_mode from telemetry message
 *
 * @return Selected drive mode (see DRIVE_MODE ENUM)
 */
static inline uint8_t mavlink_msg_telemetry_get_drive_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  83);
}

/**
 * @brief Decode a telemetry message into a struct
 *
 * @param msg The message to decode
 * @param telemetry C-struct to decode the message contents into
 */
static inline void mavlink_msg_telemetry_decode(const mavlink_message_t* msg, mavlink_telemetry_t* telemetry)
{
#if MAVLINK_NEED_BYTE_SWAP
	telemetry->timestamp = mavlink_msg_telemetry_get_timestamp(msg);
	telemetry->xacc = mavlink_msg_telemetry_get_xacc(msg);
	telemetry->yacc = mavlink_msg_telemetry_get_yacc(msg);
	telemetry->zacc = mavlink_msg_telemetry_get_zacc(msg);
	telemetry->xgyro = mavlink_msg_telemetry_get_xgyro(msg);
	telemetry->ygyro = mavlink_msg_telemetry_get_ygyro(msg);
	telemetry->zgyro = mavlink_msg_telemetry_get_zgyro(msg);
	telemetry->dist_front = mavlink_msg_telemetry_get_dist_front(msg);
	telemetry->dist_rear = mavlink_msg_telemetry_get_dist_rear(msg);
	telemetry->dist_side = mavlink_msg_telemetry_get_dist_side(msg);
	telemetry->odom = mavlink_msg_telemetry_get_odom(msg);
	telemetry->odom_accumulated = mavlink_msg_telemetry_get_odom_accumulated(msg);
	telemetry->xmotion_front = mavlink_msg_telemetry_get_xmotion_front(msg);
	telemetry->ymotion_front = mavlink_msg_telemetry_get_ymotion_front(msg);
	telemetry->xmotion_rear = mavlink_msg_telemetry_get_xmotion_rear(msg);
	telemetry->ymotion_rear = mavlink_msg_telemetry_get_ymotion_rear(msg);
	telemetry->current_motor = mavlink_msg_telemetry_get_current_motor(msg);
	telemetry->current_servo_front = mavlink_msg_telemetry_get_current_servo_front(msg);
	telemetry->current_servo_rear = mavlink_msg_telemetry_get_current_servo_rear(msg);
	telemetry->current_total = mavlink_msg_telemetry_get_current_total(msg);
	telemetry->pwm_servo_front = mavlink_msg_telemetry_get_pwm_servo_front(msg);
	telemetry->pwm_servo_rear = mavlink_msg_telemetry_get_pwm_servo_rear(msg);
	telemetry->battery_voltage = mavlink_msg_telemetry_get_battery_voltage(msg);
	telemetry->motion_front_quality = mavlink_msg_telemetry_get_motion_front_quality(msg);
	telemetry->motion_rear_quality = mavlink_msg_telemetry_get_motion_rear_quality(msg);
	telemetry->remote_control = mavlink_msg_telemetry_get_remote_control(msg);
	telemetry->drive_mode = mavlink_msg_telemetry_get_drive_mode(msg);
#else
	memcpy(telemetry, _MAV_PAYLOAD(msg), 84);
#endif
}
