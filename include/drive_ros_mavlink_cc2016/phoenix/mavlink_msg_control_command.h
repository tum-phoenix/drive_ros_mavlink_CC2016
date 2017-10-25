// MESSAGE CONTROL_COMMAND PACKING

#define MAVLINK_MSG_ID_CONTROL_COMMAND 64

typedef struct __mavlink_control_command_t
{
 float velocity; ///< Desired velocity [m/s]
 float steering_front; ///< Front axle steering angle [rad]
 float steering_rear; ///< Rear axle steering angle [rad]
 uint8_t indicator_left; ///< Left turn signal indicator
 uint8_t indicator_right; ///< Right turn signal indicator
} mavlink_control_command_t;

#define MAVLINK_MSG_ID_CONTROL_COMMAND_LEN 14
#define MAVLINK_MSG_ID_64_LEN 14



#define MAVLINK_MESSAGE_INFO_CONTROL_COMMAND { \
	"CONTROL_COMMAND", \
	5, \
	{  { "velocity", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_control_command_t, velocity) }, \
         { "steering_front", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_control_command_t, steering_front) }, \
         { "steering_rear", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_control_command_t, steering_rear) }, \
         { "indicator_left", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_control_command_t, indicator_left) }, \
         { "indicator_right", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_control_command_t, indicator_right) }, \
         } \
}


/**
 * @brief Pack a control_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param velocity Desired velocity [m/s]
 * @param steering_front Front axle steering angle [rad]
 * @param steering_rear Rear axle steering angle [rad]
 * @param indicator_left Left turn signal indicator
 * @param indicator_right Right turn signal indicator
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_control_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float velocity, float steering_front, float steering_rear, uint8_t indicator_left, uint8_t indicator_right)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[14];
	_mav_put_float(buf, 0, velocity);
	_mav_put_float(buf, 4, steering_front);
	_mav_put_float(buf, 8, steering_rear);
	_mav_put_uint8_t(buf, 12, indicator_left);
	_mav_put_uint8_t(buf, 13, indicator_right);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 14);
#else
	mavlink_control_command_t packet;
	packet.velocity = velocity;
	packet.steering_front = steering_front;
	packet.steering_rear = steering_rear;
	packet.indicator_left = indicator_left;
	packet.indicator_right = indicator_right;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 14);
#endif

	msg->msgid = MAVLINK_MSG_ID_CONTROL_COMMAND;
	return mavlink_finalize_message(msg, system_id, component_id, 14, 142);
}

/**
 * @brief Pack a control_command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param velocity Desired velocity [m/s]
 * @param steering_front Front axle steering angle [rad]
 * @param steering_rear Rear axle steering angle [rad]
 * @param indicator_left Left turn signal indicator
 * @param indicator_right Right turn signal indicator
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_control_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float velocity,float steering_front,float steering_rear,uint8_t indicator_left,uint8_t indicator_right)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[14];
	_mav_put_float(buf, 0, velocity);
	_mav_put_float(buf, 4, steering_front);
	_mav_put_float(buf, 8, steering_rear);
	_mav_put_uint8_t(buf, 12, indicator_left);
	_mav_put_uint8_t(buf, 13, indicator_right);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 14);
#else
	mavlink_control_command_t packet;
	packet.velocity = velocity;
	packet.steering_front = steering_front;
	packet.steering_rear = steering_rear;
	packet.indicator_left = indicator_left;
	packet.indicator_right = indicator_right;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 14);
#endif

	msg->msgid = MAVLINK_MSG_ID_CONTROL_COMMAND;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 14, 142);
}

/**
 * @brief Encode a control_command struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param control_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_control_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_control_command_t* control_command)
{
	return mavlink_msg_control_command_pack(system_id, component_id, msg, control_command->velocity, control_command->steering_front, control_command->steering_rear, control_command->indicator_left, control_command->indicator_right);
}

/**
 * @brief Send a control_command message
 * @param chan MAVLink channel to send the message
 *
 * @param velocity Desired velocity [m/s]
 * @param steering_front Front axle steering angle [rad]
 * @param steering_rear Rear axle steering angle [rad]
 * @param indicator_left Left turn signal indicator
 * @param indicator_right Right turn signal indicator
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_control_command_send(mavlink_channel_t chan, float velocity, float steering_front, float steering_rear, uint8_t indicator_left, uint8_t indicator_right)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[14];
	_mav_put_float(buf, 0, velocity);
	_mav_put_float(buf, 4, steering_front);
	_mav_put_float(buf, 8, steering_rear);
	_mav_put_uint8_t(buf, 12, indicator_left);
	_mav_put_uint8_t(buf, 13, indicator_right);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_COMMAND, buf, 14, 142);
#else
	mavlink_control_command_t packet;
	packet.velocity = velocity;
	packet.steering_front = steering_front;
	packet.steering_rear = steering_rear;
	packet.indicator_left = indicator_left;
	packet.indicator_right = indicator_right;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_COMMAND, (const char *)&packet, 14, 142);
#endif
}

#endif

// MESSAGE CONTROL_COMMAND UNPACKING


/**
 * @brief Get field velocity from control_command message
 *
 * @return Desired velocity [m/s]
 */
static inline float mavlink_msg_control_command_get_velocity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field steering_front from control_command message
 *
 * @return Front axle steering angle [rad]
 */
static inline float mavlink_msg_control_command_get_steering_front(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field steering_rear from control_command message
 *
 * @return Rear axle steering angle [rad]
 */
static inline float mavlink_msg_control_command_get_steering_rear(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field indicator_left from control_command message
 *
 * @return Left turn signal indicator
 */
static inline uint8_t mavlink_msg_control_command_get_indicator_left(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field indicator_right from control_command message
 *
 * @return Right turn signal indicator
 */
static inline uint8_t mavlink_msg_control_command_get_indicator_right(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Decode a control_command message into a struct
 *
 * @param msg The message to decode
 * @param control_command C-struct to decode the message contents into
 */
static inline void mavlink_msg_control_command_decode(const mavlink_message_t* msg, mavlink_control_command_t* control_command)
{
#if MAVLINK_NEED_BYTE_SWAP
	control_command->velocity = mavlink_msg_control_command_get_velocity(msg);
	control_command->steering_front = mavlink_msg_control_command_get_steering_front(msg);
	control_command->steering_rear = mavlink_msg_control_command_get_steering_rear(msg);
	control_command->indicator_left = mavlink_msg_control_command_get_indicator_left(msg);
	control_command->indicator_right = mavlink_msg_control_command_get_indicator_right(msg);
#else
	memcpy(control_command, _MAV_PAYLOAD(msg), 14);
#endif
}
