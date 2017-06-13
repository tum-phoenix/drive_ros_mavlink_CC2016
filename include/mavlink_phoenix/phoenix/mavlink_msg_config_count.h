// MESSAGE CONFIG_COUNT PACKING

#define MAVLINK_MSG_ID_CONFIG_COUNT 195

typedef struct __mavlink_config_count_t
{
 uint32_t config_id_mask; ///< Bitmask indicating which ids are used (LSB = id 0)
} mavlink_config_count_t;

#define MAVLINK_MSG_ID_CONFIG_COUNT_LEN 4
#define MAVLINK_MSG_ID_195_LEN 4



#define MAVLINK_MESSAGE_INFO_CONFIG_COUNT { \
	"CONFIG_COUNT", \
	1, \
	{  { "config_id_mask", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_config_count_t, config_id_mask) }, \
         } \
}


/**
 * @brief Pack a config_count message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param config_id_mask Bitmask indicating which ids are used (LSB = id 0)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_config_count_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t config_id_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint32_t(buf, 0, config_id_mask);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 4);
#else
	mavlink_config_count_t packet;
	packet.config_id_mask = config_id_mask;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_CONFIG_COUNT;
	return mavlink_finalize_message(msg, system_id, component_id, 4, 178);
}

/**
 * @brief Pack a config_count message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param config_id_mask Bitmask indicating which ids are used (LSB = id 0)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_config_count_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t config_id_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint32_t(buf, 0, config_id_mask);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 4);
#else
	mavlink_config_count_t packet;
	packet.config_id_mask = config_id_mask;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_CONFIG_COUNT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 4, 178);
}

/**
 * @brief Encode a config_count struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param config_count C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_config_count_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_config_count_t* config_count)
{
	return mavlink_msg_config_count_pack(system_id, component_id, msg, config_count->config_id_mask);
}

/**
 * @brief Send a config_count message
 * @param chan MAVLink channel to send the message
 *
 * @param config_id_mask Bitmask indicating which ids are used (LSB = id 0)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_config_count_send(mavlink_channel_t chan, uint32_t config_id_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint32_t(buf, 0, config_id_mask);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONFIG_COUNT, buf, 4, 178);
#else
	mavlink_config_count_t packet;
	packet.config_id_mask = config_id_mask;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONFIG_COUNT, (const char *)&packet, 4, 178);
#endif
}

#endif

// MESSAGE CONFIG_COUNT UNPACKING


/**
 * @brief Get field config_id_mask from config_count message
 *
 * @return Bitmask indicating which ids are used (LSB = id 0)
 */
static inline uint32_t mavlink_msg_config_count_get_config_id_mask(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a config_count message into a struct
 *
 * @param msg The message to decode
 * @param config_count C-struct to decode the message contents into
 */
static inline void mavlink_msg_config_count_decode(const mavlink_message_t* msg, mavlink_config_count_t* config_count)
{
#if MAVLINK_NEED_BYTE_SWAP
	config_count->config_id_mask = mavlink_msg_config_count_get_config_id_mask(msg);
#else
	memcpy(config_count, _MAV_PAYLOAD(msg), 4);
#endif
}
