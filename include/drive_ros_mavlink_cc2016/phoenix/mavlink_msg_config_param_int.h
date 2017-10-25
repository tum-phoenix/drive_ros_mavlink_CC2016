// MESSAGE CONFIG_PARAM_INT PACKING

#define MAVLINK_MSG_ID_CONFIG_PARAM_INT 197

typedef struct __mavlink_config_param_int_t
{
 int32_t value; ///< Current parameter value
 int32_t default_value; ///< default parameter value
 int32_t min; ///< Minimum parameter value
 int32_t max; ///< Maximum parameter value
 uint8_t config_id; ///< ID of the configuration set
 uint8_t param_id; ///< ID of the configuration parameter within the config set
 char name[30]; ///< Name of the configuration parameter
} mavlink_config_param_int_t;

#define MAVLINK_MSG_ID_CONFIG_PARAM_INT_LEN 48
#define MAVLINK_MSG_ID_197_LEN 48

#define MAVLINK_MSG_CONFIG_PARAM_INT_FIELD_NAME_LEN 30

#define MAVLINK_MESSAGE_INFO_CONFIG_PARAM_INT { \
	"CONFIG_PARAM_INT", \
	7, \
	{  { "value", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_config_param_int_t, value) }, \
         { "default_value", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_config_param_int_t, default_value) }, \
         { "min", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_config_param_int_t, min) }, \
         { "max", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_config_param_int_t, max) }, \
         { "config_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_config_param_int_t, config_id) }, \
         { "param_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_config_param_int_t, param_id) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 30, 18, offsetof(mavlink_config_param_int_t, name) }, \
         } \
}


/**
 * @brief Pack a config_param_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param config_id ID of the configuration set
 * @param param_id ID of the configuration parameter within the config set
 * @param name Name of the configuration parameter
 * @param value Current parameter value
 * @param default_value default parameter value
 * @param min Minimum parameter value
 * @param max Maximum parameter value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_config_param_int_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t config_id, uint8_t param_id, const char *name, int32_t value, int32_t default_value, int32_t min, int32_t max)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[48];
	_mav_put_int32_t(buf, 0, value);
	_mav_put_int32_t(buf, 4, default_value);
	_mav_put_int32_t(buf, 8, min);
	_mav_put_int32_t(buf, 12, max);
	_mav_put_uint8_t(buf, 16, config_id);
	_mav_put_uint8_t(buf, 17, param_id);
	_mav_put_char_array(buf, 18, name, 30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 48);
#else
	mavlink_config_param_int_t packet;
	packet.value = value;
	packet.default_value = default_value;
	packet.min = min;
	packet.max = max;
	packet.config_id = config_id;
	packet.param_id = param_id;
	mav_array_memcpy(packet.name, name, sizeof(char)*30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 48);
#endif

	msg->msgid = MAVLINK_MSG_ID_CONFIG_PARAM_INT;
	return mavlink_finalize_message(msg, system_id, component_id, 48, 207);
}

/**
 * @brief Pack a config_param_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param config_id ID of the configuration set
 * @param param_id ID of the configuration parameter within the config set
 * @param name Name of the configuration parameter
 * @param value Current parameter value
 * @param default_value default parameter value
 * @param min Minimum parameter value
 * @param max Maximum parameter value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_config_param_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t config_id,uint8_t param_id,const char *name,int32_t value,int32_t default_value,int32_t min,int32_t max)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[48];
	_mav_put_int32_t(buf, 0, value);
	_mav_put_int32_t(buf, 4, default_value);
	_mav_put_int32_t(buf, 8, min);
	_mav_put_int32_t(buf, 12, max);
	_mav_put_uint8_t(buf, 16, config_id);
	_mav_put_uint8_t(buf, 17, param_id);
	_mav_put_char_array(buf, 18, name, 30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 48);
#else
	mavlink_config_param_int_t packet;
	packet.value = value;
	packet.default_value = default_value;
	packet.min = min;
	packet.max = max;
	packet.config_id = config_id;
	packet.param_id = param_id;
	mav_array_memcpy(packet.name, name, sizeof(char)*30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 48);
#endif

	msg->msgid = MAVLINK_MSG_ID_CONFIG_PARAM_INT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 48, 207);
}

/**
 * @brief Encode a config_param_int struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param config_param_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_config_param_int_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_config_param_int_t* config_param_int)
{
	return mavlink_msg_config_param_int_pack(system_id, component_id, msg, config_param_int->config_id, config_param_int->param_id, config_param_int->name, config_param_int->value, config_param_int->default_value, config_param_int->min, config_param_int->max);
}

/**
 * @brief Send a config_param_int message
 * @param chan MAVLink channel to send the message
 *
 * @param config_id ID of the configuration set
 * @param param_id ID of the configuration parameter within the config set
 * @param name Name of the configuration parameter
 * @param value Current parameter value
 * @param default_value default parameter value
 * @param min Minimum parameter value
 * @param max Maximum parameter value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_config_param_int_send(mavlink_channel_t chan, uint8_t config_id, uint8_t param_id, const char *name, int32_t value, int32_t default_value, int32_t min, int32_t max)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[48];
	_mav_put_int32_t(buf, 0, value);
	_mav_put_int32_t(buf, 4, default_value);
	_mav_put_int32_t(buf, 8, min);
	_mav_put_int32_t(buf, 12, max);
	_mav_put_uint8_t(buf, 16, config_id);
	_mav_put_uint8_t(buf, 17, param_id);
	_mav_put_char_array(buf, 18, name, 30);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONFIG_PARAM_INT, buf, 48, 207);
#else
	mavlink_config_param_int_t packet;
	packet.value = value;
	packet.default_value = default_value;
	packet.min = min;
	packet.max = max;
	packet.config_id = config_id;
	packet.param_id = param_id;
	mav_array_memcpy(packet.name, name, sizeof(char)*30);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONFIG_PARAM_INT, (const char *)&packet, 48, 207);
#endif
}

#endif

// MESSAGE CONFIG_PARAM_INT UNPACKING


/**
 * @brief Get field config_id from config_param_int message
 *
 * @return ID of the configuration set
 */
static inline uint8_t mavlink_msg_config_param_int_get_config_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field param_id from config_param_int message
 *
 * @return ID of the configuration parameter within the config set
 */
static inline uint8_t mavlink_msg_config_param_int_get_param_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field name from config_param_int message
 *
 * @return Name of the configuration parameter
 */
static inline uint16_t mavlink_msg_config_param_int_get_name(const mavlink_message_t* msg, char *name)
{
	return _MAV_RETURN_char_array(msg, name, 30,  18);
}

/**
 * @brief Get field value from config_param_int message
 *
 * @return Current parameter value
 */
static inline int32_t mavlink_msg_config_param_int_get_value(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field default_value from config_param_int message
 *
 * @return default parameter value
 */
static inline int32_t mavlink_msg_config_param_int_get_default_value(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field min from config_param_int message
 *
 * @return Minimum parameter value
 */
static inline int32_t mavlink_msg_config_param_int_get_min(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field max from config_param_int message
 *
 * @return Maximum parameter value
 */
static inline int32_t mavlink_msg_config_param_int_get_max(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Decode a config_param_int message into a struct
 *
 * @param msg The message to decode
 * @param config_param_int C-struct to decode the message contents into
 */
static inline void mavlink_msg_config_param_int_decode(const mavlink_message_t* msg, mavlink_config_param_int_t* config_param_int)
{
#if MAVLINK_NEED_BYTE_SWAP
	config_param_int->value = mavlink_msg_config_param_int_get_value(msg);
	config_param_int->default_value = mavlink_msg_config_param_int_get_default_value(msg);
	config_param_int->min = mavlink_msg_config_param_int_get_min(msg);
	config_param_int->max = mavlink_msg_config_param_int_get_max(msg);
	config_param_int->config_id = mavlink_msg_config_param_int_get_config_id(msg);
	config_param_int->param_id = mavlink_msg_config_param_int_get_param_id(msg);
	mavlink_msg_config_param_int_get_name(msg, config_param_int->name);
#else
	memcpy(config_param_int, _MAV_PAYLOAD(msg), 48);
#endif
}
