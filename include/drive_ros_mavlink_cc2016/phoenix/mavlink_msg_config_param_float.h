// MESSAGE CONFIG_PARAM_FLOAT PACKING

#define MAVLINK_MSG_ID_CONFIG_PARAM_FLOAT 199

typedef struct __mavlink_config_param_float_t
{
 float value; ///< Current parameter value
 float default_value; ///< Default parameter value
 float min; ///< Minimum parameter value
 float max; ///< Maximum parameter value
 uint8_t config_id; ///< ID of the configuration set
 uint8_t param_id; ///< ID of the configuration parameter within the config set
 char name[30]; ///< Name of the configuration parameter
} mavlink_config_param_float_t;

#define MAVLINK_MSG_ID_CONFIG_PARAM_FLOAT_LEN 48
#define MAVLINK_MSG_ID_199_LEN 48

#define MAVLINK_MSG_CONFIG_PARAM_FLOAT_FIELD_NAME_LEN 30

#define MAVLINK_MESSAGE_INFO_CONFIG_PARAM_FLOAT { \
	"CONFIG_PARAM_FLOAT", \
	7, \
	{  { "value", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_config_param_float_t, value) }, \
         { "default_value", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_config_param_float_t, default_value) }, \
         { "min", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_config_param_float_t, min) }, \
         { "max", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_config_param_float_t, max) }, \
         { "config_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_config_param_float_t, config_id) }, \
         { "param_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_config_param_float_t, param_id) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 30, 18, offsetof(mavlink_config_param_float_t, name) }, \
         } \
}


/**
 * @brief Pack a config_param_float message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param config_id ID of the configuration set
 * @param param_id ID of the configuration parameter within the config set
 * @param name Name of the configuration parameter
 * @param value Current parameter value
 * @param default_value Default parameter value
 * @param min Minimum parameter value
 * @param max Maximum parameter value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_config_param_float_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t config_id, uint8_t param_id, const char *name, float value, float default_value, float min, float max)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[48];
	_mav_put_float(buf, 0, value);
	_mav_put_float(buf, 4, default_value);
	_mav_put_float(buf, 8, min);
	_mav_put_float(buf, 12, max);
	_mav_put_uint8_t(buf, 16, config_id);
	_mav_put_uint8_t(buf, 17, param_id);
	_mav_put_char_array(buf, 18, name, 30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 48);
#else
	mavlink_config_param_float_t packet;
	packet.value = value;
	packet.default_value = default_value;
	packet.min = min;
	packet.max = max;
	packet.config_id = config_id;
	packet.param_id = param_id;
	mav_array_memcpy(packet.name, name, sizeof(char)*30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 48);
#endif

	msg->msgid = MAVLINK_MSG_ID_CONFIG_PARAM_FLOAT;
	return mavlink_finalize_message(msg, system_id, component_id, 48, 225);
}

/**
 * @brief Pack a config_param_float message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param config_id ID of the configuration set
 * @param param_id ID of the configuration parameter within the config set
 * @param name Name of the configuration parameter
 * @param value Current parameter value
 * @param default_value Default parameter value
 * @param min Minimum parameter value
 * @param max Maximum parameter value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_config_param_float_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t config_id,uint8_t param_id,const char *name,float value,float default_value,float min,float max)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[48];
	_mav_put_float(buf, 0, value);
	_mav_put_float(buf, 4, default_value);
	_mav_put_float(buf, 8, min);
	_mav_put_float(buf, 12, max);
	_mav_put_uint8_t(buf, 16, config_id);
	_mav_put_uint8_t(buf, 17, param_id);
	_mav_put_char_array(buf, 18, name, 30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 48);
#else
	mavlink_config_param_float_t packet;
	packet.value = value;
	packet.default_value = default_value;
	packet.min = min;
	packet.max = max;
	packet.config_id = config_id;
	packet.param_id = param_id;
	mav_array_memcpy(packet.name, name, sizeof(char)*30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 48);
#endif

	msg->msgid = MAVLINK_MSG_ID_CONFIG_PARAM_FLOAT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 48, 225);
}

/**
 * @brief Encode a config_param_float struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param config_param_float C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_config_param_float_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_config_param_float_t* config_param_float)
{
	return mavlink_msg_config_param_float_pack(system_id, component_id, msg, config_param_float->config_id, config_param_float->param_id, config_param_float->name, config_param_float->value, config_param_float->default_value, config_param_float->min, config_param_float->max);
}

/**
 * @brief Send a config_param_float message
 * @param chan MAVLink channel to send the message
 *
 * @param config_id ID of the configuration set
 * @param param_id ID of the configuration parameter within the config set
 * @param name Name of the configuration parameter
 * @param value Current parameter value
 * @param default_value Default parameter value
 * @param min Minimum parameter value
 * @param max Maximum parameter value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_config_param_float_send(mavlink_channel_t chan, uint8_t config_id, uint8_t param_id, const char *name, float value, float default_value, float min, float max)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[48];
	_mav_put_float(buf, 0, value);
	_mav_put_float(buf, 4, default_value);
	_mav_put_float(buf, 8, min);
	_mav_put_float(buf, 12, max);
	_mav_put_uint8_t(buf, 16, config_id);
	_mav_put_uint8_t(buf, 17, param_id);
	_mav_put_char_array(buf, 18, name, 30);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONFIG_PARAM_FLOAT, buf, 48, 225);
#else
	mavlink_config_param_float_t packet;
	packet.value = value;
	packet.default_value = default_value;
	packet.min = min;
	packet.max = max;
	packet.config_id = config_id;
	packet.param_id = param_id;
	mav_array_memcpy(packet.name, name, sizeof(char)*30);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONFIG_PARAM_FLOAT, (const char *)&packet, 48, 225);
#endif
}

#endif

// MESSAGE CONFIG_PARAM_FLOAT UNPACKING


/**
 * @brief Get field config_id from config_param_float message
 *
 * @return ID of the configuration set
 */
static inline uint8_t mavlink_msg_config_param_float_get_config_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field param_id from config_param_float message
 *
 * @return ID of the configuration parameter within the config set
 */
static inline uint8_t mavlink_msg_config_param_float_get_param_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field name from config_param_float message
 *
 * @return Name of the configuration parameter
 */
static inline uint16_t mavlink_msg_config_param_float_get_name(const mavlink_message_t* msg, char *name)
{
	return _MAV_RETURN_char_array(msg, name, 30,  18);
}

/**
 * @brief Get field value from config_param_float message
 *
 * @return Current parameter value
 */
static inline float mavlink_msg_config_param_float_get_value(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field default_value from config_param_float message
 *
 * @return Default parameter value
 */
static inline float mavlink_msg_config_param_float_get_default_value(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field min from config_param_float message
 *
 * @return Minimum parameter value
 */
static inline float mavlink_msg_config_param_float_get_min(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field max from config_param_float message
 *
 * @return Maximum parameter value
 */
static inline float mavlink_msg_config_param_float_get_max(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a config_param_float message into a struct
 *
 * @param msg The message to decode
 * @param config_param_float C-struct to decode the message contents into
 */
static inline void mavlink_msg_config_param_float_decode(const mavlink_message_t* msg, mavlink_config_param_float_t* config_param_float)
{
#if MAVLINK_NEED_BYTE_SWAP
	config_param_float->value = mavlink_msg_config_param_float_get_value(msg);
	config_param_float->default_value = mavlink_msg_config_param_float_get_default_value(msg);
	config_param_float->min = mavlink_msg_config_param_float_get_min(msg);
	config_param_float->max = mavlink_msg_config_param_float_get_max(msg);
	config_param_float->config_id = mavlink_msg_config_param_float_get_config_id(msg);
	config_param_float->param_id = mavlink_msg_config_param_float_get_param_id(msg);
	mavlink_msg_config_param_float_get_name(msg, config_param_float->name);
#else
	memcpy(config_param_float, _MAV_PAYLOAD(msg), 48);
#endif
}
