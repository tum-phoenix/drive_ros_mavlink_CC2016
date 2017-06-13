// MESSAGE NOTIFICATION PACKING

#define MAVLINK_MSG_ID_NOTIFICATION 0

typedef struct __mavlink_notification_t
{
 uint32_t timestamp; ///< Timestamp of the notification (us since start of microcontroller)
 uint8_t type; ///< Type of the notification (see enum NOTIFICATION_TYPE)
 char description[50]; ///< Notification
 char tag[30]; ///< Tag
} mavlink_notification_t;

#define MAVLINK_MSG_ID_NOTIFICATION_LEN 85
#define MAVLINK_MSG_ID_0_LEN 85

#define MAVLINK_MSG_NOTIFICATION_FIELD_DESCRIPTION_LEN 50
#define MAVLINK_MSG_NOTIFICATION_FIELD_TAG_LEN 30

#define MAVLINK_MESSAGE_INFO_NOTIFICATION { \
	"NOTIFICATION", \
	4, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_notification_t, timestamp) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_notification_t, type) }, \
         { "description", NULL, MAVLINK_TYPE_CHAR, 50, 5, offsetof(mavlink_notification_t, description) }, \
         { "tag", NULL, MAVLINK_TYPE_CHAR, 30, 55, offsetof(mavlink_notification_t, tag) }, \
         } \
}


/**
 * @brief Pack a notification message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp Timestamp of the notification (us since start of microcontroller)
 * @param type Type of the notification (see enum NOTIFICATION_TYPE)
 * @param description Notification
 * @param tag Tag
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_notification_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t timestamp, uint8_t type, const char *description, const char *tag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[85];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_uint8_t(buf, 4, type);
	_mav_put_char_array(buf, 5, description, 50);
	_mav_put_char_array(buf, 55, tag, 30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 85);
#else
	mavlink_notification_t packet;
	packet.timestamp = timestamp;
	packet.type = type;
	mav_array_memcpy(packet.description, description, sizeof(char)*50);
	mav_array_memcpy(packet.tag, tag, sizeof(char)*30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 85);
#endif

	msg->msgid = MAVLINK_MSG_ID_NOTIFICATION;
	return mavlink_finalize_message(msg, system_id, component_id, 85, 195);
}

/**
 * @brief Pack a notification message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp of the notification (us since start of microcontroller)
 * @param type Type of the notification (see enum NOTIFICATION_TYPE)
 * @param description Notification
 * @param tag Tag
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_notification_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t timestamp,uint8_t type,const char *description,const char *tag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[85];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_uint8_t(buf, 4, type);
	_mav_put_char_array(buf, 5, description, 50);
	_mav_put_char_array(buf, 55, tag, 30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 85);
#else
	mavlink_notification_t packet;
	packet.timestamp = timestamp;
	packet.type = type;
	mav_array_memcpy(packet.description, description, sizeof(char)*50);
	mav_array_memcpy(packet.tag, tag, sizeof(char)*30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 85);
#endif

	msg->msgid = MAVLINK_MSG_ID_NOTIFICATION;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 85, 195);
}

/**
 * @brief Encode a notification struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param notification C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_notification_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_notification_t* notification)
{
	return mavlink_msg_notification_pack(system_id, component_id, msg, notification->timestamp, notification->type, notification->description, notification->tag);
}

/**
 * @brief Send a notification message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp of the notification (us since start of microcontroller)
 * @param type Type of the notification (see enum NOTIFICATION_TYPE)
 * @param description Notification
 * @param tag Tag
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_notification_send(mavlink_channel_t chan, uint32_t timestamp, uint8_t type, const char *description, const char *tag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[85];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_uint8_t(buf, 4, type);
	_mav_put_char_array(buf, 5, description, 50);
	_mav_put_char_array(buf, 55, tag, 30);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NOTIFICATION, buf, 85, 195);
#else
	mavlink_notification_t packet;
	packet.timestamp = timestamp;
	packet.type = type;
	mav_array_memcpy(packet.description, description, sizeof(char)*50);
	mav_array_memcpy(packet.tag, tag, sizeof(char)*30);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NOTIFICATION, (const char *)&packet, 85, 195);
#endif
}

#endif

// MESSAGE NOTIFICATION UNPACKING


/**
 * @brief Get field timestamp from notification message
 *
 * @return Timestamp of the notification (us since start of microcontroller)
 */
static inline uint32_t mavlink_msg_notification_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field type from notification message
 *
 * @return Type of the notification (see enum NOTIFICATION_TYPE)
 */
static inline uint8_t mavlink_msg_notification_get_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field description from notification message
 *
 * @return Notification
 */
static inline uint16_t mavlink_msg_notification_get_description(const mavlink_message_t* msg, char *description)
{
	return _MAV_RETURN_char_array(msg, description, 50,  5);
}

/**
 * @brief Get field tag from notification message
 *
 * @return Tag
 */
static inline uint16_t mavlink_msg_notification_get_tag(const mavlink_message_t* msg, char *tag)
{
	return _MAV_RETURN_char_array(msg, tag, 30,  55);
}

/**
 * @brief Decode a notification message into a struct
 *
 * @param msg The message to decode
 * @param notification C-struct to decode the message contents into
 */
static inline void mavlink_msg_notification_decode(const mavlink_message_t* msg, mavlink_notification_t* notification)
{
#if MAVLINK_NEED_BYTE_SWAP
	notification->timestamp = mavlink_msg_notification_get_timestamp(msg);
	notification->type = mavlink_msg_notification_get_type(msg);
	mavlink_msg_notification_get_description(msg, notification->description);
	mavlink_msg_notification_get_tag(msg, notification->tag);
#else
	memcpy(notification, _MAV_PAYLOAD(msg), 85);
#endif
}
