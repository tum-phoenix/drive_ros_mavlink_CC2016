// MESSAGE ODOMETER_RAW PACKING

#define MAVLINK_MSG_ID_ODOMETER_RAW 137

typedef struct __mavlink_odometer_raw_t
{
 uint32_t timestamp; ///< Timestamp of the measurement (us since system start)
 int32_t xdist; ///< Distance travelled along x-axis [ticks]
 int32_t ydist; ///< Distance travelled along y-axis [ticks]
 int32_t zdist; ///< Distance travelled along z-axis [ticks]
 int16_t quality; ///< Measurement quality indicator (-1 for no quality)
} mavlink_odometer_raw_t;

#define MAVLINK_MSG_ID_ODOMETER_RAW_LEN 18
#define MAVLINK_MSG_ID_137_LEN 18



#define MAVLINK_MESSAGE_INFO_ODOMETER_RAW { \
	"ODOMETER_RAW", \
	5, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_odometer_raw_t, timestamp) }, \
         { "xdist", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_odometer_raw_t, xdist) }, \
         { "ydist", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_odometer_raw_t, ydist) }, \
         { "zdist", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_odometer_raw_t, zdist) }, \
         { "quality", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_odometer_raw_t, quality) }, \
         } \
}


/**
 * @brief Pack a odometer_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp Timestamp of the measurement (us since system start)
 * @param xdist Distance travelled along x-axis [ticks]
 * @param ydist Distance travelled along y-axis [ticks]
 * @param zdist Distance travelled along z-axis [ticks]
 * @param quality Measurement quality indicator (-1 for no quality)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_odometer_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t timestamp, int32_t xdist, int32_t ydist, int32_t zdist, int16_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_int32_t(buf, 4, xdist);
	_mav_put_int32_t(buf, 8, ydist);
	_mav_put_int32_t(buf, 12, zdist);
	_mav_put_int16_t(buf, 16, quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 18);
#else
	mavlink_odometer_raw_t packet;
	packet.timestamp = timestamp;
	packet.xdist = xdist;
	packet.ydist = ydist;
	packet.zdist = zdist;
	packet.quality = quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 18);
#endif

	msg->msgid = MAVLINK_MSG_ID_ODOMETER_RAW;
	return mavlink_finalize_message(msg, system_id, component_id, 18, 255);
}

/**
 * @brief Pack a odometer_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp of the measurement (us since system start)
 * @param xdist Distance travelled along x-axis [ticks]
 * @param ydist Distance travelled along y-axis [ticks]
 * @param zdist Distance travelled along z-axis [ticks]
 * @param quality Measurement quality indicator (-1 for no quality)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_odometer_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t timestamp,int32_t xdist,int32_t ydist,int32_t zdist,int16_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_int32_t(buf, 4, xdist);
	_mav_put_int32_t(buf, 8, ydist);
	_mav_put_int32_t(buf, 12, zdist);
	_mav_put_int16_t(buf, 16, quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 18);
#else
	mavlink_odometer_raw_t packet;
	packet.timestamp = timestamp;
	packet.xdist = xdist;
	packet.ydist = ydist;
	packet.zdist = zdist;
	packet.quality = quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 18);
#endif

	msg->msgid = MAVLINK_MSG_ID_ODOMETER_RAW;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 18, 255);
}

/**
 * @brief Encode a odometer_raw struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param odometer_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_odometer_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_odometer_raw_t* odometer_raw)
{
	return mavlink_msg_odometer_raw_pack(system_id, component_id, msg, odometer_raw->timestamp, odometer_raw->xdist, odometer_raw->ydist, odometer_raw->zdist, odometer_raw->quality);
}

/**
 * @brief Send a odometer_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp of the measurement (us since system start)
 * @param xdist Distance travelled along x-axis [ticks]
 * @param ydist Distance travelled along y-axis [ticks]
 * @param zdist Distance travelled along z-axis [ticks]
 * @param quality Measurement quality indicator (-1 for no quality)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_odometer_raw_send(mavlink_channel_t chan, uint32_t timestamp, int32_t xdist, int32_t ydist, int32_t zdist, int16_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_int32_t(buf, 4, xdist);
	_mav_put_int32_t(buf, 8, ydist);
	_mav_put_int32_t(buf, 12, zdist);
	_mav_put_int16_t(buf, 16, quality);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ODOMETER_RAW, buf, 18, 255);
#else
	mavlink_odometer_raw_t packet;
	packet.timestamp = timestamp;
	packet.xdist = xdist;
	packet.ydist = ydist;
	packet.zdist = zdist;
	packet.quality = quality;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ODOMETER_RAW, (const char *)&packet, 18, 255);
#endif
}

#endif

// MESSAGE ODOMETER_RAW UNPACKING


/**
 * @brief Get field timestamp from odometer_raw message
 *
 * @return Timestamp of the measurement (us since system start)
 */
static inline uint32_t mavlink_msg_odometer_raw_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field xdist from odometer_raw message
 *
 * @return Distance travelled along x-axis [ticks]
 */
static inline int32_t mavlink_msg_odometer_raw_get_xdist(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field ydist from odometer_raw message
 *
 * @return Distance travelled along y-axis [ticks]
 */
static inline int32_t mavlink_msg_odometer_raw_get_ydist(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field zdist from odometer_raw message
 *
 * @return Distance travelled along z-axis [ticks]
 */
static inline int32_t mavlink_msg_odometer_raw_get_zdist(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field quality from odometer_raw message
 *
 * @return Measurement quality indicator (-1 for no quality)
 */
static inline int16_t mavlink_msg_odometer_raw_get_quality(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Decode a odometer_raw message into a struct
 *
 * @param msg The message to decode
 * @param odometer_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_odometer_raw_decode(const mavlink_message_t* msg, mavlink_odometer_raw_t* odometer_raw)
{
#if MAVLINK_NEED_BYTE_SWAP
	odometer_raw->timestamp = mavlink_msg_odometer_raw_get_timestamp(msg);
	odometer_raw->xdist = mavlink_msg_odometer_raw_get_xdist(msg);
	odometer_raw->ydist = mavlink_msg_odometer_raw_get_ydist(msg);
	odometer_raw->zdist = mavlink_msg_odometer_raw_get_zdist(msg);
	odometer_raw->quality = mavlink_msg_odometer_raw_get_quality(msg);
#else
	memcpy(odometer_raw, _MAV_PAYLOAD(msg), 18);
#endif
}
