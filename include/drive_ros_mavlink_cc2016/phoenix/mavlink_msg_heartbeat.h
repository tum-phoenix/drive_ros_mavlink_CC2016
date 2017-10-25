// MESSAGE HEARTBEAT PACKING

#define MAVLINK_MSG_ID_HEARTBEAT 1

typedef struct __mavlink_heartbeat_t
{
 uint32_t timestamp; ///< Timestamp of the notification (us since system start)
 float rc_velocity; ///< Velocity set by remote control [m/s]
 float rc_steering_front; ///< Front steering angle set by remote control [rad]
 float rc_steering_rear; ///< Rear steering angle set by remote control [rad]
 uint16_t battery_voltage; ///< Battery voltage, in millivolts (1 = 1 millivolt)
 uint8_t remote_control; ///< Remote Control status (see REMOTE_CONTROL_STATUS ENUM)
 uint8_t drive_mode; ///< Selected drive mode (see DRIVE_MODE ENUM)
} mavlink_heartbeat_t;

#define MAVLINK_MSG_ID_HEARTBEAT_LEN 20
#define MAVLINK_MSG_ID_1_LEN 20



#define MAVLINK_MESSAGE_INFO_HEARTBEAT { \
	"HEARTBEAT", \
	7, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_heartbeat_t, timestamp) }, \
         { "rc_velocity", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_heartbeat_t, rc_velocity) }, \
         { "rc_steering_front", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_heartbeat_t, rc_steering_front) }, \
         { "rc_steering_rear", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_heartbeat_t, rc_steering_rear) }, \
         { "battery_voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_heartbeat_t, battery_voltage) }, \
         { "remote_control", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_heartbeat_t, remote_control) }, \
         { "drive_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_heartbeat_t, drive_mode) }, \
         } \
}


/**
 * @brief Pack a heartbeat message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp Timestamp of the notification (us since system start)
 * @param battery_voltage Battery voltage, in millivolts (1 = 1 millivolt)
 * @param remote_control Remote Control status (see REMOTE_CONTROL_STATUS ENUM)
 * @param drive_mode Selected drive mode (see DRIVE_MODE ENUM)
 * @param rc_velocity Velocity set by remote control [m/s]
 * @param rc_steering_front Front steering angle set by remote control [rad]
 * @param rc_steering_rear Rear steering angle set by remote control [rad]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t timestamp, uint16_t battery_voltage, uint8_t remote_control, uint8_t drive_mode, float rc_velocity, float rc_steering_front, float rc_steering_rear)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[20];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float(buf, 4, rc_velocity);
	_mav_put_float(buf, 8, rc_steering_front);
	_mav_put_float(buf, 12, rc_steering_rear);
	_mav_put_uint16_t(buf, 16, battery_voltage);
	_mav_put_uint8_t(buf, 18, remote_control);
	_mav_put_uint8_t(buf, 19, drive_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 20);
#else
	mavlink_heartbeat_t packet;
	packet.timestamp = timestamp;
	packet.rc_velocity = rc_velocity;
	packet.rc_steering_front = rc_steering_front;
	packet.rc_steering_rear = rc_steering_rear;
	packet.battery_voltage = battery_voltage;
	packet.remote_control = remote_control;
	packet.drive_mode = drive_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 20);
#endif

	msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;
	return mavlink_finalize_message(msg, system_id, component_id, 20, 139);
}

/**
 * @brief Pack a heartbeat message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp of the notification (us since system start)
 * @param battery_voltage Battery voltage, in millivolts (1 = 1 millivolt)
 * @param remote_control Remote Control status (see REMOTE_CONTROL_STATUS ENUM)
 * @param drive_mode Selected drive mode (see DRIVE_MODE ENUM)
 * @param rc_velocity Velocity set by remote control [m/s]
 * @param rc_steering_front Front steering angle set by remote control [rad]
 * @param rc_steering_rear Rear steering angle set by remote control [rad]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_heartbeat_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t timestamp,uint16_t battery_voltage,uint8_t remote_control,uint8_t drive_mode,float rc_velocity,float rc_steering_front,float rc_steering_rear)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[20];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float(buf, 4, rc_velocity);
	_mav_put_float(buf, 8, rc_steering_front);
	_mav_put_float(buf, 12, rc_steering_rear);
	_mav_put_uint16_t(buf, 16, battery_voltage);
	_mav_put_uint8_t(buf, 18, remote_control);
	_mav_put_uint8_t(buf, 19, drive_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 20);
#else
	mavlink_heartbeat_t packet;
	packet.timestamp = timestamp;
	packet.rc_velocity = rc_velocity;
	packet.rc_steering_front = rc_steering_front;
	packet.rc_steering_rear = rc_steering_rear;
	packet.battery_voltage = battery_voltage;
	packet.remote_control = remote_control;
	packet.drive_mode = drive_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 20);
#endif

	msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 20, 139);
}

/**
 * @brief Encode a heartbeat struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param heartbeat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_heartbeat_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_heartbeat_t* heartbeat)
{
	return mavlink_msg_heartbeat_pack(system_id, component_id, msg, heartbeat->timestamp, heartbeat->battery_voltage, heartbeat->remote_control, heartbeat->drive_mode, heartbeat->rc_velocity, heartbeat->rc_steering_front, heartbeat->rc_steering_rear);
}

/**
 * @brief Send a heartbeat message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp of the notification (us since system start)
 * @param battery_voltage Battery voltage, in millivolts (1 = 1 millivolt)
 * @param remote_control Remote Control status (see REMOTE_CONTROL_STATUS ENUM)
 * @param drive_mode Selected drive mode (see DRIVE_MODE ENUM)
 * @param rc_velocity Velocity set by remote control [m/s]
 * @param rc_steering_front Front steering angle set by remote control [rad]
 * @param rc_steering_rear Rear steering angle set by remote control [rad]
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_heartbeat_send(mavlink_channel_t chan, uint32_t timestamp, uint16_t battery_voltage, uint8_t remote_control, uint8_t drive_mode, float rc_velocity, float rc_steering_front, float rc_steering_rear)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[20];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float(buf, 4, rc_velocity);
	_mav_put_float(buf, 8, rc_steering_front);
	_mav_put_float(buf, 12, rc_steering_rear);
	_mav_put_uint16_t(buf, 16, battery_voltage);
	_mav_put_uint8_t(buf, 18, remote_control);
	_mav_put_uint8_t(buf, 19, drive_mode);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, buf, 20, 139);
#else
	mavlink_heartbeat_t packet;
	packet.timestamp = timestamp;
	packet.rc_velocity = rc_velocity;
	packet.rc_steering_front = rc_steering_front;
	packet.rc_steering_rear = rc_steering_rear;
	packet.battery_voltage = battery_voltage;
	packet.remote_control = remote_control;
	packet.drive_mode = drive_mode;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, (const char *)&packet, 20, 139);
#endif
}

#endif

// MESSAGE HEARTBEAT UNPACKING


/**
 * @brief Get field timestamp from heartbeat message
 *
 * @return Timestamp of the notification (us since system start)
 */
static inline uint32_t mavlink_msg_heartbeat_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field battery_voltage from heartbeat message
 *
 * @return Battery voltage, in millivolts (1 = 1 millivolt)
 */
static inline uint16_t mavlink_msg_heartbeat_get_battery_voltage(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field remote_control from heartbeat message
 *
 * @return Remote Control status (see REMOTE_CONTROL_STATUS ENUM)
 */
static inline uint8_t mavlink_msg_heartbeat_get_remote_control(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field drive_mode from heartbeat message
 *
 * @return Selected drive mode (see DRIVE_MODE ENUM)
 */
static inline uint8_t mavlink_msg_heartbeat_get_drive_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Get field rc_velocity from heartbeat message
 *
 * @return Velocity set by remote control [m/s]
 */
static inline float mavlink_msg_heartbeat_get_rc_velocity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field rc_steering_front from heartbeat message
 *
 * @return Front steering angle set by remote control [rad]
 */
static inline float mavlink_msg_heartbeat_get_rc_steering_front(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field rc_steering_rear from heartbeat message
 *
 * @return Rear steering angle set by remote control [rad]
 */
static inline float mavlink_msg_heartbeat_get_rc_steering_rear(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a heartbeat message into a struct
 *
 * @param msg The message to decode
 * @param heartbeat C-struct to decode the message contents into
 */
static inline void mavlink_msg_heartbeat_decode(const mavlink_message_t* msg, mavlink_heartbeat_t* heartbeat)
{
#if MAVLINK_NEED_BYTE_SWAP
	heartbeat->timestamp = mavlink_msg_heartbeat_get_timestamp(msg);
	heartbeat->rc_velocity = mavlink_msg_heartbeat_get_rc_velocity(msg);
	heartbeat->rc_steering_front = mavlink_msg_heartbeat_get_rc_steering_front(msg);
	heartbeat->rc_steering_rear = mavlink_msg_heartbeat_get_rc_steering_rear(msg);
	heartbeat->battery_voltage = mavlink_msg_heartbeat_get_battery_voltage(msg);
	heartbeat->remote_control = mavlink_msg_heartbeat_get_remote_control(msg);
	heartbeat->drive_mode = mavlink_msg_heartbeat_get_drive_mode(msg);
#else
	memcpy(heartbeat, _MAV_PAYLOAD(msg), 20);
#endif
}
