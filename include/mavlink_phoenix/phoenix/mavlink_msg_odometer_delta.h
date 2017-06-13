// MESSAGE ODOMETER_DELTA PACKING

#define MAVLINK_MSG_ID_ODOMETER_DELTA 138

typedef struct __mavlink_odometer_delta_t
{
 uint32_t timestamp; ///< Timestamp of the measurement (us since system start)
 float delta; ///< Time-delta within which the measurement has taken place [s]
 float xdist; ///< Distance travelled along x-axis within period [m]
 float ydist; ///< Distance travelled along y-axis within period [m]
 float zdist; ///< Distance travelled along z-axis within period [m]
 float xvelocity; ///< Velocity along x-axis [m/s]
 float yvelocity; ///< Velocity along y-axis [m/s]
 float zvelocity; ///< Velocity along z-axis [m/s]
 int16_t quality; ///< Measurement quality indicator (-1 for no quality)
} mavlink_odometer_delta_t;

#define MAVLINK_MSG_ID_ODOMETER_DELTA_LEN 34
#define MAVLINK_MSG_ID_138_LEN 34



#define MAVLINK_MESSAGE_INFO_ODOMETER_DELTA { \
	"ODOMETER_DELTA", \
	9, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_odometer_delta_t, timestamp) }, \
         { "delta", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_odometer_delta_t, delta) }, \
         { "xdist", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_odometer_delta_t, xdist) }, \
         { "ydist", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_odometer_delta_t, ydist) }, \
         { "zdist", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_odometer_delta_t, zdist) }, \
         { "xvelocity", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_odometer_delta_t, xvelocity) }, \
         { "yvelocity", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_odometer_delta_t, yvelocity) }, \
         { "zvelocity", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_odometer_delta_t, zvelocity) }, \
         { "quality", NULL, MAVLINK_TYPE_INT16_T, 0, 32, offsetof(mavlink_odometer_delta_t, quality) }, \
         } \
}


/**
 * @brief Pack a odometer_delta message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp Timestamp of the measurement (us since system start)
 * @param delta Time-delta within which the measurement has taken place [s]
 * @param xdist Distance travelled along x-axis within period [m]
 * @param ydist Distance travelled along y-axis within period [m]
 * @param zdist Distance travelled along z-axis within period [m]
 * @param xvelocity Velocity along x-axis [m/s]
 * @param yvelocity Velocity along y-axis [m/s]
 * @param zvelocity Velocity along z-axis [m/s]
 * @param quality Measurement quality indicator (-1 for no quality)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_odometer_delta_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t timestamp, float delta, float xdist, float ydist, float zdist, float xvelocity, float yvelocity, float zvelocity, int16_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[34];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float(buf, 4, delta);
	_mav_put_float(buf, 8, xdist);
	_mav_put_float(buf, 12, ydist);
	_mav_put_float(buf, 16, zdist);
	_mav_put_float(buf, 20, xvelocity);
	_mav_put_float(buf, 24, yvelocity);
	_mav_put_float(buf, 28, zvelocity);
	_mav_put_int16_t(buf, 32, quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 34);
#else
	mavlink_odometer_delta_t packet;
	packet.timestamp = timestamp;
	packet.delta = delta;
	packet.xdist = xdist;
	packet.ydist = ydist;
	packet.zdist = zdist;
	packet.xvelocity = xvelocity;
	packet.yvelocity = yvelocity;
	packet.zvelocity = zvelocity;
	packet.quality = quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 34);
#endif

	msg->msgid = MAVLINK_MSG_ID_ODOMETER_DELTA;
	return mavlink_finalize_message(msg, system_id, component_id, 34, 51);
}

/**
 * @brief Pack a odometer_delta message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp of the measurement (us since system start)
 * @param delta Time-delta within which the measurement has taken place [s]
 * @param xdist Distance travelled along x-axis within period [m]
 * @param ydist Distance travelled along y-axis within period [m]
 * @param zdist Distance travelled along z-axis within period [m]
 * @param xvelocity Velocity along x-axis [m/s]
 * @param yvelocity Velocity along y-axis [m/s]
 * @param zvelocity Velocity along z-axis [m/s]
 * @param quality Measurement quality indicator (-1 for no quality)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_odometer_delta_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t timestamp,float delta,float xdist,float ydist,float zdist,float xvelocity,float yvelocity,float zvelocity,int16_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[34];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float(buf, 4, delta);
	_mav_put_float(buf, 8, xdist);
	_mav_put_float(buf, 12, ydist);
	_mav_put_float(buf, 16, zdist);
	_mav_put_float(buf, 20, xvelocity);
	_mav_put_float(buf, 24, yvelocity);
	_mav_put_float(buf, 28, zvelocity);
	_mav_put_int16_t(buf, 32, quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 34);
#else
	mavlink_odometer_delta_t packet;
	packet.timestamp = timestamp;
	packet.delta = delta;
	packet.xdist = xdist;
	packet.ydist = ydist;
	packet.zdist = zdist;
	packet.xvelocity = xvelocity;
	packet.yvelocity = yvelocity;
	packet.zvelocity = zvelocity;
	packet.quality = quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 34);
#endif

	msg->msgid = MAVLINK_MSG_ID_ODOMETER_DELTA;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 34, 51);
}

/**
 * @brief Encode a odometer_delta struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param odometer_delta C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_odometer_delta_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_odometer_delta_t* odometer_delta)
{
	return mavlink_msg_odometer_delta_pack(system_id, component_id, msg, odometer_delta->timestamp, odometer_delta->delta, odometer_delta->xdist, odometer_delta->ydist, odometer_delta->zdist, odometer_delta->xvelocity, odometer_delta->yvelocity, odometer_delta->zvelocity, odometer_delta->quality);
}

/**
 * @brief Send a odometer_delta message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp of the measurement (us since system start)
 * @param delta Time-delta within which the measurement has taken place [s]
 * @param xdist Distance travelled along x-axis within period [m]
 * @param ydist Distance travelled along y-axis within period [m]
 * @param zdist Distance travelled along z-axis within period [m]
 * @param xvelocity Velocity along x-axis [m/s]
 * @param yvelocity Velocity along y-axis [m/s]
 * @param zvelocity Velocity along z-axis [m/s]
 * @param quality Measurement quality indicator (-1 for no quality)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_odometer_delta_send(mavlink_channel_t chan, uint32_t timestamp, float delta, float xdist, float ydist, float zdist, float xvelocity, float yvelocity, float zvelocity, int16_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[34];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float(buf, 4, delta);
	_mav_put_float(buf, 8, xdist);
	_mav_put_float(buf, 12, ydist);
	_mav_put_float(buf, 16, zdist);
	_mav_put_float(buf, 20, xvelocity);
	_mav_put_float(buf, 24, yvelocity);
	_mav_put_float(buf, 28, zvelocity);
	_mav_put_int16_t(buf, 32, quality);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ODOMETER_DELTA, buf, 34, 51);
#else
	mavlink_odometer_delta_t packet;
	packet.timestamp = timestamp;
	packet.delta = delta;
	packet.xdist = xdist;
	packet.ydist = ydist;
	packet.zdist = zdist;
	packet.xvelocity = xvelocity;
	packet.yvelocity = yvelocity;
	packet.zvelocity = zvelocity;
	packet.quality = quality;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ODOMETER_DELTA, (const char *)&packet, 34, 51);
#endif
}

#endif

// MESSAGE ODOMETER_DELTA UNPACKING


/**
 * @brief Get field timestamp from odometer_delta message
 *
 * @return Timestamp of the measurement (us since system start)
 */
static inline uint32_t mavlink_msg_odometer_delta_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field delta from odometer_delta message
 *
 * @return Time-delta within which the measurement has taken place [s]
 */
static inline float mavlink_msg_odometer_delta_get_delta(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field xdist from odometer_delta message
 *
 * @return Distance travelled along x-axis within period [m]
 */
static inline float mavlink_msg_odometer_delta_get_xdist(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field ydist from odometer_delta message
 *
 * @return Distance travelled along y-axis within period [m]
 */
static inline float mavlink_msg_odometer_delta_get_ydist(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field zdist from odometer_delta message
 *
 * @return Distance travelled along z-axis within period [m]
 */
static inline float mavlink_msg_odometer_delta_get_zdist(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field xvelocity from odometer_delta message
 *
 * @return Velocity along x-axis [m/s]
 */
static inline float mavlink_msg_odometer_delta_get_xvelocity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field yvelocity from odometer_delta message
 *
 * @return Velocity along y-axis [m/s]
 */
static inline float mavlink_msg_odometer_delta_get_yvelocity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field zvelocity from odometer_delta message
 *
 * @return Velocity along z-axis [m/s]
 */
static inline float mavlink_msg_odometer_delta_get_zvelocity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field quality from odometer_delta message
 *
 * @return Measurement quality indicator (-1 for no quality)
 */
static inline int16_t mavlink_msg_odometer_delta_get_quality(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  32);
}

/**
 * @brief Decode a odometer_delta message into a struct
 *
 * @param msg The message to decode
 * @param odometer_delta C-struct to decode the message contents into
 */
static inline void mavlink_msg_odometer_delta_decode(const mavlink_message_t* msg, mavlink_odometer_delta_t* odometer_delta)
{
#if MAVLINK_NEED_BYTE_SWAP
	odometer_delta->timestamp = mavlink_msg_odometer_delta_get_timestamp(msg);
	odometer_delta->delta = mavlink_msg_odometer_delta_get_delta(msg);
	odometer_delta->xdist = mavlink_msg_odometer_delta_get_xdist(msg);
	odometer_delta->ydist = mavlink_msg_odometer_delta_get_ydist(msg);
	odometer_delta->zdist = mavlink_msg_odometer_delta_get_zdist(msg);
	odometer_delta->xvelocity = mavlink_msg_odometer_delta_get_xvelocity(msg);
	odometer_delta->yvelocity = mavlink_msg_odometer_delta_get_yvelocity(msg);
	odometer_delta->zvelocity = mavlink_msg_odometer_delta_get_zvelocity(msg);
	odometer_delta->quality = mavlink_msg_odometer_delta_get_quality(msg);
#else
	memcpy(odometer_delta, _MAV_PAYLOAD(msg), 34);
#endif
}
