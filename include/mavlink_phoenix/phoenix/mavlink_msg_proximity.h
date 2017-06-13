// MESSAGE PROXIMITY PACKING

#define MAVLINK_MSG_ID_PROXIMITY 144

typedef struct __mavlink_proximity_t
{
 uint32_t timestamp; ///< Timestamp of the measurement (us since system start)
 float distance; ///< Distance to obstacle [m]
} mavlink_proximity_t;

#define MAVLINK_MSG_ID_PROXIMITY_LEN 8
#define MAVLINK_MSG_ID_144_LEN 8



#define MAVLINK_MESSAGE_INFO_PROXIMITY { \
	"PROXIMITY", \
	2, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_proximity_t, timestamp) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_proximity_t, distance) }, \
         } \
}


/**
 * @brief Pack a proximity message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp Timestamp of the measurement (us since system start)
 * @param distance Distance to obstacle [m]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_proximity_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t timestamp, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[8];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float(buf, 4, distance);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 8);
#else
	mavlink_proximity_t packet;
	packet.timestamp = timestamp;
	packet.distance = distance;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 8);
#endif

	msg->msgid = MAVLINK_MSG_ID_PROXIMITY;
	return mavlink_finalize_message(msg, system_id, component_id, 8, 30);
}

/**
 * @brief Pack a proximity message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp of the measurement (us since system start)
 * @param distance Distance to obstacle [m]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_proximity_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t timestamp,float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[8];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float(buf, 4, distance);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 8);
#else
	mavlink_proximity_t packet;
	packet.timestamp = timestamp;
	packet.distance = distance;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 8);
#endif

	msg->msgid = MAVLINK_MSG_ID_PROXIMITY;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 8, 30);
}

/**
 * @brief Encode a proximity struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param proximity C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_proximity_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_proximity_t* proximity)
{
	return mavlink_msg_proximity_pack(system_id, component_id, msg, proximity->timestamp, proximity->distance);
}

/**
 * @brief Send a proximity message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp of the measurement (us since system start)
 * @param distance Distance to obstacle [m]
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_proximity_send(mavlink_channel_t chan, uint32_t timestamp, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[8];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float(buf, 4, distance);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PROXIMITY, buf, 8, 30);
#else
	mavlink_proximity_t packet;
	packet.timestamp = timestamp;
	packet.distance = distance;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PROXIMITY, (const char *)&packet, 8, 30);
#endif
}

#endif

// MESSAGE PROXIMITY UNPACKING


/**
 * @brief Get field timestamp from proximity message
 *
 * @return Timestamp of the measurement (us since system start)
 */
static inline uint32_t mavlink_msg_proximity_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field distance from proximity message
 *
 * @return Distance to obstacle [m]
 */
static inline float mavlink_msg_proximity_get_distance(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a proximity message into a struct
 *
 * @param msg The message to decode
 * @param proximity C-struct to decode the message contents into
 */
static inline void mavlink_msg_proximity_decode(const mavlink_message_t* msg, mavlink_proximity_t* proximity)
{
#if MAVLINK_NEED_BYTE_SWAP
	proximity->timestamp = mavlink_msg_proximity_get_timestamp(msg);
	proximity->distance = mavlink_msg_proximity_get_distance(msg);
#else
	memcpy(proximity, _MAV_PAYLOAD(msg), 8);
#endif
}
