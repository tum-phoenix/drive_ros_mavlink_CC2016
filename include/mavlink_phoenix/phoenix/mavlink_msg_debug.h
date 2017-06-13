// MESSAGE DEBUG PACKING

#define MAVLINK_MSG_ID_DEBUG 2

typedef struct __mavlink_debug_t
{
 uint32_t timestamp; ///< Timestamp of the message (us since system start)
 float data[12]; ///< Debug data
} mavlink_debug_t;

#define MAVLINK_MSG_ID_DEBUG_LEN 52
#define MAVLINK_MSG_ID_2_LEN 52

#define MAVLINK_MSG_DEBUG_FIELD_DATA_LEN 12

#define MAVLINK_MESSAGE_INFO_DEBUG { \
	"DEBUG", \
	2, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_debug_t, timestamp) }, \
         { "data", NULL, MAVLINK_TYPE_FLOAT, 12, 4, offsetof(mavlink_debug_t, data) }, \
         } \
}


/**
 * @brief Pack a debug message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp Timestamp of the message (us since system start)
 * @param data Debug data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_debug_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t timestamp, const float *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[52];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float_array(buf, 4, data, 12);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 52);
#else
	mavlink_debug_t packet;
	packet.timestamp = timestamp;
	mav_array_memcpy(packet.data, data, sizeof(float)*12);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 52);
#endif

	msg->msgid = MAVLINK_MSG_ID_DEBUG;
	return mavlink_finalize_message(msg, system_id, component_id, 52, 159);
}

/**
 * @brief Pack a debug message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp of the message (us since system start)
 * @param data Debug data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_debug_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t timestamp,const float *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[52];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float_array(buf, 4, data, 12);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 52);
#else
	mavlink_debug_t packet;
	packet.timestamp = timestamp;
	mav_array_memcpy(packet.data, data, sizeof(float)*12);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 52);
#endif

	msg->msgid = MAVLINK_MSG_ID_DEBUG;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 52, 159);
}

/**
 * @brief Encode a debug struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param debug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_debug_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_debug_t* debug)
{
	return mavlink_msg_debug_pack(system_id, component_id, msg, debug->timestamp, debug->data);
}

/**
 * @brief Send a debug message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp of the message (us since system start)
 * @param data Debug data
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_debug_send(mavlink_channel_t chan, uint32_t timestamp, const float *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[52];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float_array(buf, 4, data, 12);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG, buf, 52, 159);
#else
	mavlink_debug_t packet;
	packet.timestamp = timestamp;
	mav_array_memcpy(packet.data, data, sizeof(float)*12);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG, (const char *)&packet, 52, 159);
#endif
}

#endif

// MESSAGE DEBUG UNPACKING


/**
 * @brief Get field timestamp from debug message
 *
 * @return Timestamp of the message (us since system start)
 */
static inline uint32_t mavlink_msg_debug_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field data from debug message
 *
 * @return Debug data
 */
static inline uint16_t mavlink_msg_debug_get_data(const mavlink_message_t* msg, float *data)
{
	return _MAV_RETURN_float_array(msg, data, 12,  4);
}

/**
 * @brief Decode a debug message into a struct
 *
 * @param msg The message to decode
 * @param debug C-struct to decode the message contents into
 */
static inline void mavlink_msg_debug_decode(const mavlink_message_t* msg, mavlink_debug_t* debug)
{
#if MAVLINK_NEED_BYTE_SWAP
	debug->timestamp = mavlink_msg_debug_get_timestamp(msg);
	mavlink_msg_debug_get_data(msg, debug->data);
#else
	memcpy(debug, _MAV_PAYLOAD(msg), 52);
#endif
}
