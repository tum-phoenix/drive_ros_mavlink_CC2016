// MESSAGE CONFIG_PARAM_SET_BOOL PACKING

#define MAVLINK_MSG_ID_CONFIG_PARAM_SET_BOOL 201

typedef struct __mavlink_config_param_set_bool_t
{
 uint8_t config_id; ///< ID of the configuration set
 uint8_t param_id; ///< ID of the configuration parameter within the config set
 uint8_t value; ///< New parameter value
} mavlink_config_param_set_bool_t;

#define MAVLINK_MSG_ID_CONFIG_PARAM_SET_BOOL_LEN 3
#define MAVLINK_MSG_ID_201_LEN 3



#define MAVLINK_MESSAGE_INFO_CONFIG_PARAM_SET_BOOL { \
	"CONFIG_PARAM_SET_BOOL", \
	3, \
	{  { "config_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_config_param_set_bool_t, config_id) }, \
         { "param_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_config_param_set_bool_t, param_id) }, \
         { "value", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_config_param_set_bool_t, value) }, \
         } \
}


/**
 * @brief Pack a config_param_set_bool message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param config_id ID of the configuration set
 * @param param_id ID of the configuration parameter within the config set
 * @param value New parameter value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_config_param_set_bool_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t config_id, uint8_t param_id, uint8_t value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[3];
	_mav_put_uint8_t(buf, 0, config_id);
	_mav_put_uint8_t(buf, 1, param_id);
	_mav_put_uint8_t(buf, 2, value);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 3);
#else
	mavlink_config_param_set_bool_t packet;
	packet.config_id = config_id;
	packet.param_id = param_id;
	packet.value = value;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 3);
#endif

	msg->msgid = MAVLINK_MSG_ID_CONFIG_PARAM_SET_BOOL;
	return mavlink_finalize_message(msg, system_id, component_id, 3, 112);
}

/**
 * @brief Pack a config_param_set_bool message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param config_id ID of the configuration set
 * @param param_id ID of the configuration parameter within the config set
 * @param value New parameter value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_config_param_set_bool_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t config_id,uint8_t param_id,uint8_t value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[3];
	_mav_put_uint8_t(buf, 0, config_id);
	_mav_put_uint8_t(buf, 1, param_id);
	_mav_put_uint8_t(buf, 2, value);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 3);
#else
	mavlink_config_param_set_bool_t packet;
	packet.config_id = config_id;
	packet.param_id = param_id;
	packet.value = value;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 3);
#endif

	msg->msgid = MAVLINK_MSG_ID_CONFIG_PARAM_SET_BOOL;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 3, 112);
}

/**
 * @brief Encode a config_param_set_bool struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param config_param_set_bool C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_config_param_set_bool_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_config_param_set_bool_t* config_param_set_bool)
{
	return mavlink_msg_config_param_set_bool_pack(system_id, component_id, msg, config_param_set_bool->config_id, config_param_set_bool->param_id, config_param_set_bool->value);
}

/**
 * @brief Send a config_param_set_bool message
 * @param chan MAVLink channel to send the message
 *
 * @param config_id ID of the configuration set
 * @param param_id ID of the configuration parameter within the config set
 * @param value New parameter value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_config_param_set_bool_send(mavlink_channel_t chan, uint8_t config_id, uint8_t param_id, uint8_t value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[3];
	_mav_put_uint8_t(buf, 0, config_id);
	_mav_put_uint8_t(buf, 1, param_id);
	_mav_put_uint8_t(buf, 2, value);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONFIG_PARAM_SET_BOOL, buf, 3, 112);
#else
	mavlink_config_param_set_bool_t packet;
	packet.config_id = config_id;
	packet.param_id = param_id;
	packet.value = value;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONFIG_PARAM_SET_BOOL, (const char *)&packet, 3, 112);
#endif
}

#endif

// MESSAGE CONFIG_PARAM_SET_BOOL UNPACKING


/**
 * @brief Get field config_id from config_param_set_bool message
 *
 * @return ID of the configuration set
 */
static inline uint8_t mavlink_msg_config_param_set_bool_get_config_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field param_id from config_param_set_bool message
 *
 * @return ID of the configuration parameter within the config set
 */
static inline uint8_t mavlink_msg_config_param_set_bool_get_param_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field value from config_param_set_bool message
 *
 * @return New parameter value
 */
static inline uint8_t mavlink_msg_config_param_set_bool_get_value(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a config_param_set_bool message into a struct
 *
 * @param msg The message to decode
 * @param config_param_set_bool C-struct to decode the message contents into
 */
static inline void mavlink_msg_config_param_set_bool_decode(const mavlink_message_t* msg, mavlink_config_param_set_bool_t* config_param_set_bool)
{
#if MAVLINK_NEED_BYTE_SWAP
	config_param_set_bool->config_id = mavlink_msg_config_param_set_bool_get_config_id(msg);
	config_param_set_bool->param_id = mavlink_msg_config_param_set_bool_get_param_id(msg);
	config_param_set_bool->value = mavlink_msg_config_param_set_bool_get_value(msg);
#else
	memcpy(config_param_set_bool, _MAV_PAYLOAD(msg), 3);
#endif
}
