// MESSAGE COMMAND PACKING

#define MAVLINK_MSG_ID_COMMAND 210

typedef struct __mavlink_command_t
{
 uint8_t command; ///< Command to perform (see COMMAND_TYPE ENUM)
} mavlink_command_t;

#define MAVLINK_MSG_ID_COMMAND_LEN 1
#define MAVLINK_MSG_ID_210_LEN 1



#define MAVLINK_MESSAGE_INFO_COMMAND { \
	"COMMAND", \
	1, \
	{  { "command", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_command_t, command) }, \
         } \
}


/**
 * @brief Pack a command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param command Command to perform (see COMMAND_TYPE ENUM)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, command);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 1);
#else
	mavlink_command_t packet;
	packet.command = command;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 1);
#endif

	msg->msgid = MAVLINK_MSG_ID_COMMAND;
	return mavlink_finalize_message(msg, system_id, component_id, 1, 16);
}

/**
 * @brief Pack a command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param command Command to perform (see COMMAND_TYPE ENUM)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, command);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 1);
#else
	mavlink_command_t packet;
	packet.command = command;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 1);
#endif

	msg->msgid = MAVLINK_MSG_ID_COMMAND;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 1, 16);
}

/**
 * @brief Encode a command struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_command_t* command)
{
	return mavlink_msg_command_pack(system_id, component_id, msg, command->command);
}

/**
 * @brief Send a command message
 * @param chan MAVLink channel to send the message
 *
 * @param command Command to perform (see COMMAND_TYPE ENUM)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_command_send(mavlink_channel_t chan, uint8_t command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, command);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND, buf, 1, 16);
#else
	mavlink_command_t packet;
	packet.command = command;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND, (const char *)&packet, 1, 16);
#endif
}

#endif

// MESSAGE COMMAND UNPACKING


/**
 * @brief Get field command from command message
 *
 * @return Command to perform (see COMMAND_TYPE ENUM)
 */
static inline uint8_t mavlink_msg_command_get_command(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a command message into a struct
 *
 * @param msg The message to decode
 * @param command C-struct to decode the message contents into
 */
static inline void mavlink_msg_command_decode(const mavlink_message_t* msg, mavlink_command_t* command)
{
#if MAVLINK_NEED_BYTE_SWAP
	command->command = mavlink_msg_command_get_command(msg);
#else
	memcpy(command, _MAV_PAYLOAD(msg), 1);
#endif
}
