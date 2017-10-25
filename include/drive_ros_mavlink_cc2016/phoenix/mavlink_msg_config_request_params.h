// MESSAGE CONFIG_REQUEST_PARAMS PACKING

#define MAVLINK_MSG_ID_CONFIG_REQUEST_PARAMS 194

typedef struct __mavlink_config_request_params_t
{
 uint8_t config_id; ///< ID of the configuration set
 uint8_t param_id; ///< ID of the config parameter within the set
} mavlink_config_request_params_t;

#define MAVLINK_MSG_ID_CONFIG_REQUEST_PARAMS_LEN 2
#define MAVLINK_MSG_ID_194_LEN 2



#define MAVLINK_MESSAGE_INFO_CONFIG_REQUEST_PARAMS { \
	"CONFIG_REQUEST_PARAMS", \
	2, \
	{  { "config_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_config_request_params_t, config_id) }, \
         { "param_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_config_request_params_t, param_id) }, \
         } \
}


/**
 * @brief Pack a config_request_params message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param config_id ID of the configuration set
 * @param param_id ID of the config parameter within the set
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_config_request_params_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t config_id, uint8_t param_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_uint8_t(buf, 0, config_id);
	_mav_put_uint8_t(buf, 1, param_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 2);
#else
	mavlink_config_request_params_t packet;
	packet.config_id = config_id;
	packet.param_id = param_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 2);
#endif

	msg->msgid = MAVLINK_MSG_ID_CONFIG_REQUEST_PARAMS;
	return mavlink_finalize_message(msg, system_id, component_id, 2, 88);
}

/**
 * @brief Pack a config_request_params message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param config_id ID of the configuration set
 * @param param_id ID of the config parameter within the set
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_config_request_params_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t config_id,uint8_t param_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_uint8_t(buf, 0, config_id);
	_mav_put_uint8_t(buf, 1, param_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 2);
#else
	mavlink_config_request_params_t packet;
	packet.config_id = config_id;
	packet.param_id = param_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 2);
#endif

	msg->msgid = MAVLINK_MSG_ID_CONFIG_REQUEST_PARAMS;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 2, 88);
}

/**
 * @brief Encode a config_request_params struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param config_request_params C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_config_request_params_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_config_request_params_t* config_request_params)
{
	return mavlink_msg_config_request_params_pack(system_id, component_id, msg, config_request_params->config_id, config_request_params->param_id);
}

/**
 * @brief Send a config_request_params message
 * @param chan MAVLink channel to send the message
 *
 * @param config_id ID of the configuration set
 * @param param_id ID of the config parameter within the set
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_config_request_params_send(mavlink_channel_t chan, uint8_t config_id, uint8_t param_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_uint8_t(buf, 0, config_id);
	_mav_put_uint8_t(buf, 1, param_id);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONFIG_REQUEST_PARAMS, buf, 2, 88);
#else
	mavlink_config_request_params_t packet;
	packet.config_id = config_id;
	packet.param_id = param_id;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONFIG_REQUEST_PARAMS, (const char *)&packet, 2, 88);
#endif
}

#endif

// MESSAGE CONFIG_REQUEST_PARAMS UNPACKING


/**
 * @brief Get field config_id from config_request_params message
 *
 * @return ID of the configuration set
 */
static inline uint8_t mavlink_msg_config_request_params_get_config_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field param_id from config_request_params message
 *
 * @return ID of the config parameter within the set
 */
static inline uint8_t mavlink_msg_config_request_params_get_param_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a config_request_params message into a struct
 *
 * @param msg The message to decode
 * @param config_request_params C-struct to decode the message contents into
 */
static inline void mavlink_msg_config_request_params_decode(const mavlink_message_t* msg, mavlink_config_request_params_t* config_request_params)
{
#if MAVLINK_NEED_BYTE_SWAP
	config_request_params->config_id = mavlink_msg_config_request_params_get_config_id(msg);
	config_request_params->param_id = mavlink_msg_config_request_params_get_param_id(msg);
#else
	memcpy(config_request_params, _MAV_PAYLOAD(msg), 2);
#endif
}
