// MESSAGE CONFIG PACKING

#define MAVLINK_MSG_ID_CONFIG 196

typedef struct __mavlink_config_t
{
 uint16_t param_id_mask; ///< Bitmask indicating which param ids are used (LSB = id 0)
 uint8_t config_id; ///< ID of the configuration set
 char name[30]; ///< Name of the configuration set
} mavlink_config_t;

#define MAVLINK_MSG_ID_CONFIG_LEN 33
#define MAVLINK_MSG_ID_196_LEN 33

#define MAVLINK_MSG_CONFIG_FIELD_NAME_LEN 30

#define MAVLINK_MESSAGE_INFO_CONFIG { \
	"CONFIG", \
	3, \
	{  { "param_id_mask", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_config_t, param_id_mask) }, \
         { "config_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_config_t, config_id) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 30, 3, offsetof(mavlink_config_t, name) }, \
         } \
}


/**
 * @brief Pack a config message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param config_id ID of the configuration set
 * @param name Name of the configuration set
 * @param param_id_mask Bitmask indicating which param ids are used (LSB = id 0)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_config_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t config_id, const char *name, uint16_t param_id_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[33];
	_mav_put_uint16_t(buf, 0, param_id_mask);
	_mav_put_uint8_t(buf, 2, config_id);
	_mav_put_char_array(buf, 3, name, 30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 33);
#else
	mavlink_config_t packet;
	packet.param_id_mask = param_id_mask;
	packet.config_id = config_id;
	mav_array_memcpy(packet.name, name, sizeof(char)*30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 33);
#endif

	msg->msgid = MAVLINK_MSG_ID_CONFIG;
	return mavlink_finalize_message(msg, system_id, component_id, 33, 189);
}

/**
 * @brief Pack a config message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param config_id ID of the configuration set
 * @param name Name of the configuration set
 * @param param_id_mask Bitmask indicating which param ids are used (LSB = id 0)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_config_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t config_id,const char *name,uint16_t param_id_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[33];
	_mav_put_uint16_t(buf, 0, param_id_mask);
	_mav_put_uint8_t(buf, 2, config_id);
	_mav_put_char_array(buf, 3, name, 30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 33);
#else
	mavlink_config_t packet;
	packet.param_id_mask = param_id_mask;
	packet.config_id = config_id;
	mav_array_memcpy(packet.name, name, sizeof(char)*30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 33);
#endif

	msg->msgid = MAVLINK_MSG_ID_CONFIG;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 33, 189);
}

/**
 * @brief Encode a config struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param config C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_config_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_config_t* config)
{
	return mavlink_msg_config_pack(system_id, component_id, msg, config->config_id, config->name, config->param_id_mask);
}

/**
 * @brief Send a config message
 * @param chan MAVLink channel to send the message
 *
 * @param config_id ID of the configuration set
 * @param name Name of the configuration set
 * @param param_id_mask Bitmask indicating which param ids are used (LSB = id 0)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_config_send(mavlink_channel_t chan, uint8_t config_id, const char *name, uint16_t param_id_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[33];
	_mav_put_uint16_t(buf, 0, param_id_mask);
	_mav_put_uint8_t(buf, 2, config_id);
	_mav_put_char_array(buf, 3, name, 30);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONFIG, buf, 33, 189);
#else
	mavlink_config_t packet;
	packet.param_id_mask = param_id_mask;
	packet.config_id = config_id;
	mav_array_memcpy(packet.name, name, sizeof(char)*30);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONFIG, (const char *)&packet, 33, 189);
#endif
}

#endif

// MESSAGE CONFIG UNPACKING


/**
 * @brief Get field config_id from config message
 *
 * @return ID of the configuration set
 */
static inline uint8_t mavlink_msg_config_get_config_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field name from config message
 *
 * @return Name of the configuration set
 */
static inline uint16_t mavlink_msg_config_get_name(const mavlink_message_t* msg, char *name)
{
	return _MAV_RETURN_char_array(msg, name, 30,  3);
}

/**
 * @brief Get field param_id_mask from config message
 *
 * @return Bitmask indicating which param ids are used (LSB = id 0)
 */
static inline uint16_t mavlink_msg_config_get_param_id_mask(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a config message into a struct
 *
 * @param msg The message to decode
 * @param config C-struct to decode the message contents into
 */
static inline void mavlink_msg_config_decode(const mavlink_message_t* msg, mavlink_config_t* config)
{
#if MAVLINK_NEED_BYTE_SWAP
	config->param_id_mask = mavlink_msg_config_get_param_id_mask(msg);
	config->config_id = mavlink_msg_config_get_config_id(msg);
	mavlink_msg_config_get_name(msg, config->name);
#else
	memcpy(config, _MAV_PAYLOAD(msg), 33);
#endif
}
