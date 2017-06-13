// MESSAGE PARKING_LOT PACKING

#define MAVLINK_MSG_ID_PARKING_LOT 150

typedef struct __mavlink_parking_lot_t
{
 uint32_t timestamp; ///< Timestamp of the measurement (us since system start)
 float parking_lot_size; ///< Size of the parking lot [m]
 float parking_lot_position; ///< Position of the second edge of the parking lot [m]
} mavlink_parking_lot_t;

#define MAVLINK_MSG_ID_PARKING_LOT_LEN 12
#define MAVLINK_MSG_ID_150_LEN 12



#define MAVLINK_MESSAGE_INFO_PARKING_LOT { \
	"PARKING_LOT", \
	3, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_parking_lot_t, timestamp) }, \
         { "parking_lot_size", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_parking_lot_t, parking_lot_size) }, \
         { "parking_lot_position", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_parking_lot_t, parking_lot_position) }, \
         } \
}


/**
 * @brief Pack a parking_lot message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp Timestamp of the measurement (us since system start)
 * @param parking_lot_size Size of the parking lot [m]
 * @param parking_lot_position Position of the second edge of the parking lot [m]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_parking_lot_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t timestamp, float parking_lot_size, float parking_lot_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float(buf, 4, parking_lot_size);
	_mav_put_float(buf, 8, parking_lot_position);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_parking_lot_t packet;
	packet.timestamp = timestamp;
	packet.parking_lot_size = parking_lot_size;
	packet.parking_lot_position = parking_lot_position;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_PARKING_LOT;
	return mavlink_finalize_message(msg, system_id, component_id, 12, 31);
}

/**
 * @brief Pack a parking_lot message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp of the measurement (us since system start)
 * @param parking_lot_size Size of the parking lot [m]
 * @param parking_lot_position Position of the second edge of the parking lot [m]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_parking_lot_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t timestamp,float parking_lot_size,float parking_lot_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float(buf, 4, parking_lot_size);
	_mav_put_float(buf, 8, parking_lot_position);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_parking_lot_t packet;
	packet.timestamp = timestamp;
	packet.parking_lot_size = parking_lot_size;
	packet.parking_lot_position = parking_lot_position;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_PARKING_LOT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 12, 31);
}

/**
 * @brief Encode a parking_lot struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param parking_lot C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_parking_lot_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_parking_lot_t* parking_lot)
{
	return mavlink_msg_parking_lot_pack(system_id, component_id, msg, parking_lot->timestamp, parking_lot->parking_lot_size, parking_lot->parking_lot_position);
}

/**
 * @brief Send a parking_lot message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp of the measurement (us since system start)
 * @param parking_lot_size Size of the parking lot [m]
 * @param parking_lot_position Position of the second edge of the parking lot [m]
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_parking_lot_send(mavlink_channel_t chan, uint32_t timestamp, float parking_lot_size, float parking_lot_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float(buf, 4, parking_lot_size);
	_mav_put_float(buf, 8, parking_lot_position);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARKING_LOT, buf, 12, 31);
#else
	mavlink_parking_lot_t packet;
	packet.timestamp = timestamp;
	packet.parking_lot_size = parking_lot_size;
	packet.parking_lot_position = parking_lot_position;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARKING_LOT, (const char *)&packet, 12, 31);
#endif
}

#endif

// MESSAGE PARKING_LOT UNPACKING


/**
 * @brief Get field timestamp from parking_lot message
 *
 * @return Timestamp of the measurement (us since system start)
 */
static inline uint32_t mavlink_msg_parking_lot_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field parking_lot_size from parking_lot message
 *
 * @return Size of the parking lot [m]
 */
static inline float mavlink_msg_parking_lot_get_parking_lot_size(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field parking_lot_position from parking_lot message
 *
 * @return Position of the second edge of the parking lot [m]
 */
static inline float mavlink_msg_parking_lot_get_parking_lot_position(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a parking_lot message into a struct
 *
 * @param msg The message to decode
 * @param parking_lot C-struct to decode the message contents into
 */
static inline void mavlink_msg_parking_lot_decode(const mavlink_message_t* msg, mavlink_parking_lot_t* parking_lot)
{
#if MAVLINK_NEED_BYTE_SWAP
	parking_lot->timestamp = mavlink_msg_parking_lot_get_timestamp(msg);
	parking_lot->parking_lot_size = mavlink_msg_parking_lot_get_parking_lot_size(msg);
	parking_lot->parking_lot_position = mavlink_msg_parking_lot_get_parking_lot_position(msg);
#else
	memcpy(parking_lot, _MAV_PAYLOAD(msg), 12);
#endif
}
