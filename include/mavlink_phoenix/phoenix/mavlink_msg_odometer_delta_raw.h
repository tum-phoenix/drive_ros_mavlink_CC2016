// MESSAGE ODOMETER_DELTA_RAW PACKING

#define MAVLINK_MSG_ID_ODOMETER_DELTA_RAW 139

typedef struct __mavlink_odometer_delta_raw_t
{
 uint32_t timestamp; ///< Timestamp of the measurement (us since system start)
 float delta; ///< Time-delta within which the measurement has taken place [s]
 int32_t xdist; ///< Distance travelled along x-axis within period [ticks]
 int32_t ydist; ///< Distance travelled along y-axis within period [ticks]
 int32_t zdist; ///< Distance travelled along z-axis within period [ticks]
 int16_t quality; ///< Measurement quality indicator (-1 for no quality)
} mavlink_odometer_delta_raw_t;

#define MAVLINK_MSG_ID_ODOMETER_DELTA_RAW_LEN 22
#define MAVLINK_MSG_ID_139_LEN 22



#define MAVLINK_MESSAGE_INFO_ODOMETER_DELTA_RAW { \
	"ODOMETER_DELTA_RAW", \
	6, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_odometer_delta_raw_t, timestamp) }, \
         { "delta", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_odometer_delta_raw_t, delta) }, \
         { "xdist", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_odometer_delta_raw_t, xdist) }, \
         { "ydist", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_odometer_delta_raw_t, ydist) }, \
         { "zdist", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_odometer_delta_raw_t, zdist) }, \
         { "quality", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_odometer_delta_raw_t, quality) }, \
         } \
}


/**
 * @brief Pack a odometer_delta_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp Timestamp of the measurement (us since system start)
 * @param delta Time-delta within which the measurement has taken place [s]
 * @param xdist Distance travelled along x-axis within period [ticks]
 * @param ydist Distance travelled along y-axis within period [ticks]
 * @param zdist Distance travelled along z-axis within period [ticks]
 * @param quality Measurement quality indicator (-1 for no quality)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_odometer_delta_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t timestamp, float delta, int32_t xdist, int32_t ydist, int32_t zdist, int16_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[22];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float(buf, 4, delta);
	_mav_put_int32_t(buf, 8, xdist);
	_mav_put_int32_t(buf, 12, ydist);
	_mav_put_int32_t(buf, 16, zdist);
	_mav_put_int16_t(buf, 20, quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 22);
#else
	mavlink_odometer_delta_raw_t packet;
	packet.timestamp = timestamp;
	packet.delta = delta;
	packet.xdist = xdist;
	packet.ydist = ydist;
	packet.zdist = zdist;
	packet.quality = quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 22);
#endif

	msg->msgid = MAVLINK_MSG_ID_ODOMETER_DELTA_RAW;
	return mavlink_finalize_message(msg, system_id, component_id, 22, 228);
}

/**
 * @brief Pack a odometer_delta_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp of the measurement (us since system start)
 * @param delta Time-delta within which the measurement has taken place [s]
 * @param xdist Distance travelled along x-axis within period [ticks]
 * @param ydist Distance travelled along y-axis within period [ticks]
 * @param zdist Distance travelled along z-axis within period [ticks]
 * @param quality Measurement quality indicator (-1 for no quality)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_odometer_delta_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t timestamp,float delta,int32_t xdist,int32_t ydist,int32_t zdist,int16_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[22];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float(buf, 4, delta);
	_mav_put_int32_t(buf, 8, xdist);
	_mav_put_int32_t(buf, 12, ydist);
	_mav_put_int32_t(buf, 16, zdist);
	_mav_put_int16_t(buf, 20, quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 22);
#else
	mavlink_odometer_delta_raw_t packet;
	packet.timestamp = timestamp;
	packet.delta = delta;
	packet.xdist = xdist;
	packet.ydist = ydist;
	packet.zdist = zdist;
	packet.quality = quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 22);
#endif

	msg->msgid = MAVLINK_MSG_ID_ODOMETER_DELTA_RAW;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 22, 228);
}

/**
 * @brief Encode a odometer_delta_raw struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param odometer_delta_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_odometer_delta_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_odometer_delta_raw_t* odometer_delta_raw)
{
	return mavlink_msg_odometer_delta_raw_pack(system_id, component_id, msg, odometer_delta_raw->timestamp, odometer_delta_raw->delta, odometer_delta_raw->xdist, odometer_delta_raw->ydist, odometer_delta_raw->zdist, odometer_delta_raw->quality);
}

/**
 * @brief Send a odometer_delta_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp of the measurement (us since system start)
 * @param delta Time-delta within which the measurement has taken place [s]
 * @param xdist Distance travelled along x-axis within period [ticks]
 * @param ydist Distance travelled along y-axis within period [ticks]
 * @param zdist Distance travelled along z-axis within period [ticks]
 * @param quality Measurement quality indicator (-1 for no quality)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_odometer_delta_raw_send(mavlink_channel_t chan, uint32_t timestamp, float delta, int32_t xdist, int32_t ydist, int32_t zdist, int16_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[22];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float(buf, 4, delta);
	_mav_put_int32_t(buf, 8, xdist);
	_mav_put_int32_t(buf, 12, ydist);
	_mav_put_int32_t(buf, 16, zdist);
	_mav_put_int16_t(buf, 20, quality);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ODOMETER_DELTA_RAW, buf, 22, 228);
#else
	mavlink_odometer_delta_raw_t packet;
	packet.timestamp = timestamp;
	packet.delta = delta;
	packet.xdist = xdist;
	packet.ydist = ydist;
	packet.zdist = zdist;
	packet.quality = quality;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ODOMETER_DELTA_RAW, (const char *)&packet, 22, 228);
#endif
}

#endif

// MESSAGE ODOMETER_DELTA_RAW UNPACKING


/**
 * @brief Get field timestamp from odometer_delta_raw message
 *
 * @return Timestamp of the measurement (us since system start)
 */
static inline uint32_t mavlink_msg_odometer_delta_raw_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field delta from odometer_delta_raw message
 *
 * @return Time-delta within which the measurement has taken place [s]
 */
static inline float mavlink_msg_odometer_delta_raw_get_delta(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field xdist from odometer_delta_raw message
 *
 * @return Distance travelled along x-axis within period [ticks]
 */
static inline int32_t mavlink_msg_odometer_delta_raw_get_xdist(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field ydist from odometer_delta_raw message
 *
 * @return Distance travelled along y-axis within period [ticks]
 */
static inline int32_t mavlink_msg_odometer_delta_raw_get_ydist(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field zdist from odometer_delta_raw message
 *
 * @return Distance travelled along z-axis within period [ticks]
 */
static inline int32_t mavlink_msg_odometer_delta_raw_get_zdist(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field quality from odometer_delta_raw message
 *
 * @return Measurement quality indicator (-1 for no quality)
 */
static inline int16_t mavlink_msg_odometer_delta_raw_get_quality(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Decode a odometer_delta_raw message into a struct
 *
 * @param msg The message to decode
 * @param odometer_delta_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_odometer_delta_raw_decode(const mavlink_message_t* msg, mavlink_odometer_delta_raw_t* odometer_delta_raw)
{
#if MAVLINK_NEED_BYTE_SWAP
	odometer_delta_raw->timestamp = mavlink_msg_odometer_delta_raw_get_timestamp(msg);
	odometer_delta_raw->delta = mavlink_msg_odometer_delta_raw_get_delta(msg);
	odometer_delta_raw->xdist = mavlink_msg_odometer_delta_raw_get_xdist(msg);
	odometer_delta_raw->ydist = mavlink_msg_odometer_delta_raw_get_ydist(msg);
	odometer_delta_raw->zdist = mavlink_msg_odometer_delta_raw_get_zdist(msg);
	odometer_delta_raw->quality = mavlink_msg_odometer_delta_raw_get_quality(msg);
#else
	memcpy(odometer_delta_raw, _MAV_PAYLOAD(msg), 22);
#endif
}
