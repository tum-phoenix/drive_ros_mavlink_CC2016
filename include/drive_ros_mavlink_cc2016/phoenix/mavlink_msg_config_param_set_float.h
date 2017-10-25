// MESSAGE CONFIG_PARAM_SET_FLOAT PACKING

#define MAVLINK_MSG_ID_CONFIG_PARAM_SET_FLOAT 202

typedef struct __mavlink_config_param_set_float_t
{
 float value; ///< New parameter value
 uint8_t config_id; ///< ID of the configuration set
 uint8_t param_id; ///< ID of the configuration parameter within the config set
} mavlink_config_param_set_float_t;

#define MAVLINK_MSG_ID_CONFIG_PARAM_SET_FLOAT_LEN 6
#define MAVLINK_MSG_ID_202_LEN 6



#define MAVLINK_MESSAGE_INFO_CONFIG_PARAM_SET_FLOAT { \
	"CONFIG_PARAM_SET_FLOAT", \
	3, \
	{  { "value", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_config_param_set_float_t, value) }, \
         { "config_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_config_param_set_float_t, config_id) }, \
         { "param_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_config_param_set_float_t, param_id) }, \
         } \
}


/**
 * @brief Pack a config_param_set_float message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param config_id ID of the configuration set
 * @param param_id ID of the configuration parameter within the config set
 * @param value New parameter value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_config_param_set_float_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t config_id, uint8_t param_id, float value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[6];
	_mav_put_float(buf, 0, value);
	_mav_put_uint8_t(buf, 4, config_id);
	_mav_put_uint8_t(buf, 5, param_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 6);
#else
	mavlink_config_param_set_float_t packet;
	packet.value = value;
	packet.config_id = config_id;
	packet.param_id = param_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 6);
#endif

	msg->msgid = MAVLINK_MSG_ID_CONFIG_PARAM_SET_FLOAT;
	return mavlink_finalize_message(msg, system_id, component_id, 6, 48);
}

/**
 * @brief Pack a config_param_set_float message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param config_id ID of the configuration set
 * @param param_id ID of the configuration parameter within the config set
 * @param value New parameter value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_config_param_set_float_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t config_id,uint8_t param_id,float value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[6];
	_mav_put_float(buf, 0, value);
	_mav_put_uint8_t(buf, 4, config_id);
	_mav_put_uint8_t(buf, 5, param_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 6);
#else
	mavlink_config_param_set_float_t packet;
	packet.value = value;
	packet.config_id = config_id;
	packet.param_id = param_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 6);
#endif

	msg->msgid = MAVLINK_MSG_ID_CONFIG_PARAM_SET_FLOAT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 6, 48);
}

/**
 * @brief Encode a config_param_set_float struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param config_param_set_float C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_config_param_set_float_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_config_param_set_float_t* config_param_set_float)
{
	return mavlink_msg_config_param_set_float_pack(system_id, component_id, msg, config_param_set_float->config_id, config_param_set_float->param_id, config_param_set_float->value);
}

/**
 * @brief Send a config_param_set_float message
 * @param chan MAVLink channel to send the message
 *
 * @param config_id ID of the configuration set
 * @param param_id ID of the configuration parameter within the config set
 * @param value New parameter value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_config_param_set_float_send(mavlink_channel_t chan, uint8_t config_id, uint8_t param_id, float value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[6];
	_mav_put_float(buf, 0, value);
	_mav_put_uint8_t(buf, 4, config_id);
	_mav_put_uint8_t(buf, 5, param_id);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONFIG_PARAM_SET_FLOAT, buf, 6, 48);
#else
	mavlink_config_param_set_float_t packet;
	packet.value = value;
	packet.config_id = config_id;
	packet.param_id = param_id;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONFIG_PARAM_SET_FLOAT, (const char *)&packet, 6, 48);
#endif
}

#endif

// MESSAGE CONFIG_PARAM_SET_FLOAT UNPACKING


/**
 * @brief Get field config_id from config_param_set_float message
 *
 * @return ID of the configuration set
 */
static inline uint8_t mavlink_msg_config_param_set_float_get_config_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field param_id from config_param_set_float message
 *
 * @return ID of the configuration parameter within the config set
 */
static inline uint8_t mavlink_msg_config_param_set_float_get_param_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field value from config_param_set_float message
 *
 * @return New parameter value
 */
static inline float mavlink_msg_config_param_set_float_get_value(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a config_param_set_float message into a struct
 *
 * @param msg The message to decode
 * @param config_param_set_float C-struct to decode the message contents into
 */
static inline void mavlink_msg_config_param_set_float_decode(const mavlink_message_t* msg, mavlink_config_param_set_float_t* config_param_set_float)
{
#if MAVLINK_NEED_BYTE_SWAP
	config_param_set_float->value = mavlink_msg_config_param_set_float_get_value(msg);
	config_param_set_float->config_id = mavlink_msg_config_param_set_float_get_config_id(msg);
	config_param_set_float->param_id = mavlink_msg_config_param_set_float_get_param_id(msg);
#else
	memcpy(config_param_set_float, _MAV_PAYLOAD(msg), 6);
#endif
}
