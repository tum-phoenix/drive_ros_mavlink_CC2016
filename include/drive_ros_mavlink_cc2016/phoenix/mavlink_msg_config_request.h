// MESSAGE CONFIG_REQUEST PACKING

#define MAVLINK_MSG_ID_CONFIG_REQUEST 193

typedef struct __mavlink_config_request_t
{
 uint8_t config_id; ///< Id of the config to request
} mavlink_config_request_t;

#define MAVLINK_MSG_ID_CONFIG_REQUEST_LEN 1
#define MAVLINK_MSG_ID_193_LEN 1



#define MAVLINK_MESSAGE_INFO_CONFIG_REQUEST { \
	"CONFIG_REQUEST", \
	1, \
	{  { "config_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_config_request_t, config_id) }, \
         } \
}


/**
 * @brief Pack a config_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param config_id Id of the config to request
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_config_request_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t config_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, config_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 1);
#else
	mavlink_config_request_t packet;
	packet.config_id = config_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 1);
#endif

	msg->msgid = MAVLINK_MSG_ID_CONFIG_REQUEST;
	return mavlink_finalize_message(msg, system_id, component_id, 1, 36);
}

/**
 * @brief Pack a config_request message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param config_id Id of the config to request
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_config_request_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t config_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, config_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 1);
#else
	mavlink_config_request_t packet;
	packet.config_id = config_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 1);
#endif

	msg->msgid = MAVLINK_MSG_ID_CONFIG_REQUEST;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 1, 36);
}

/**
 * @brief Encode a config_request struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param config_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_config_request_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_config_request_t* config_request)
{
	return mavlink_msg_config_request_pack(system_id, component_id, msg, config_request->config_id);
}

/**
 * @brief Send a config_request message
 * @param chan MAVLink channel to send the message
 *
 * @param config_id Id of the config to request
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_config_request_send(mavlink_channel_t chan, uint8_t config_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, config_id);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONFIG_REQUEST, buf, 1, 36);
#else
	mavlink_config_request_t packet;
	packet.config_id = config_id;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONFIG_REQUEST, (const char *)&packet, 1, 36);
#endif
}

#endif

// MESSAGE CONFIG_REQUEST UNPACKING


/**
 * @brief Get field config_id from config_request message
 *
 * @return Id of the config to request
 */
static inline uint8_t mavlink_msg_config_request_get_config_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a config_request message into a struct
 *
 * @param msg The message to decode
 * @param config_request C-struct to decode the message contents into
 */
static inline void mavlink_msg_config_request_decode(const mavlink_message_t* msg, mavlink_config_request_t* config_request)
{
#if MAVLINK_NEED_BYTE_SWAP
	config_request->config_id = mavlink_msg_config_request_get_config_id(msg);
#else
	memcpy(config_request, _MAV_PAYLOAD(msg), 1);
#endif
}
