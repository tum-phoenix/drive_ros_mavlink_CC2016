// MESSAGE CONTROL_LIGHTS PACKING

#define MAVLINK_MSG_ID_CONTROL_LIGHTS 32

typedef struct __mavlink_control_lights_t
{
 uint32_t colors[15]; ///< Colors-4 bytes, r,g,b, nothing
} mavlink_control_lights_t;

#define MAVLINK_MSG_ID_CONTROL_LIGHTS_LEN 60
#define MAVLINK_MSG_ID_32_LEN 60

#define MAVLINK_MSG_CONTROL_LIGHTS_FIELD_COLORS_LEN 15

#define MAVLINK_MESSAGE_INFO_CONTROL_LIGHTS { \
	"CONTROL_LIGHTS", \
	1, \
	{  { "colors", NULL, MAVLINK_TYPE_UINT32_T, 15, 0, offsetof(mavlink_control_lights_t, colors) }, \
         } \
}


/**
 * @brief Pack a control_lights message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param colors Colors-4 bytes, r,g,b, nothing
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_control_lights_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const uint32_t *colors)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[60];

	_mav_put_uint32_t_array(buf, 0, colors, 15);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 60);
#else
	mavlink_control_lights_t packet;

	mav_array_memcpy(packet.colors, colors, sizeof(uint32_t)*15);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 60);
#endif

	msg->msgid = MAVLINK_MSG_ID_CONTROL_LIGHTS;
	return mavlink_finalize_message(msg, system_id, component_id, 60, 95);
}

/**
 * @brief Pack a control_lights message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param colors Colors-4 bytes, r,g,b, nothing
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_control_lights_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const uint32_t *colors)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[60];

	_mav_put_uint32_t_array(buf, 0, colors, 15);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 60);
#else
	mavlink_control_lights_t packet;

	mav_array_memcpy(packet.colors, colors, sizeof(uint32_t)*15);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 60);
#endif

	msg->msgid = MAVLINK_MSG_ID_CONTROL_LIGHTS;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 60, 95);
}

/**
 * @brief Encode a control_lights struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param control_lights C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_control_lights_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_control_lights_t* control_lights)
{
	return mavlink_msg_control_lights_pack(system_id, component_id, msg, control_lights->colors);
}

/**
 * @brief Send a control_lights message
 * @param chan MAVLink channel to send the message
 *
 * @param colors Colors-4 bytes, r,g,b, nothing
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_control_lights_send(mavlink_channel_t chan, const uint32_t *colors)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[60];

	_mav_put_uint32_t_array(buf, 0, colors, 15);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_LIGHTS, buf, 60, 95);
#else
	mavlink_control_lights_t packet;

	mav_array_memcpy(packet.colors, colors, sizeof(uint32_t)*15);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_LIGHTS, (const char *)&packet, 60, 95);
#endif
}

#endif

// MESSAGE CONTROL_LIGHTS UNPACKING


/**
 * @brief Get field colors from control_lights message
 *
 * @return Colors-4 bytes, r,g,b, nothing
 */
static inline uint16_t mavlink_msg_control_lights_get_colors(const mavlink_message_t* msg, uint32_t *colors)
{
	return _MAV_RETURN_uint32_t_array(msg, colors, 15,  0);
}

/**
 * @brief Decode a control_lights message into a struct
 *
 * @param msg The message to decode
 * @param control_lights C-struct to decode the message contents into
 */
static inline void mavlink_msg_control_lights_decode(const mavlink_message_t* msg, mavlink_control_lights_t* control_lights)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_control_lights_get_colors(msg, control_lights->colors);
#else
	memcpy(control_lights, _MAV_PAYLOAD(msg), 60);
#endif
}
