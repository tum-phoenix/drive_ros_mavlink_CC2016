// MESSAGE ODOMETER_ABS PACKING

#define MAVLINK_MSG_ID_ODOMETER_ABS 136

typedef struct __mavlink_odometer_abs_t
{
 uint32_t timestamp; ///< Timestamp of the measurement (us since system start)
 float xdist; ///< Distance travelled along x-axis [m]
 float ydist; ///< Distance travelled along y-axis [m]
 float zdist; ///< Distance travelled along z-axis [m]
 float xvelocity; ///< Velocity along x-axis [m/s]
 float yvelocity; ///< Velocity along y-axis [m/s]
 float zvelocity; ///< Velocity along z-axis [m/s]
 int16_t quality; ///< Measurement quality indicator (-1 for no quality)
} mavlink_odometer_abs_t;

#define MAVLINK_MSG_ID_ODOMETER_ABS_LEN 30
#define MAVLINK_MSG_ID_136_LEN 30



#define MAVLINK_MESSAGE_INFO_ODOMETER_ABS { \
	"ODOMETER_ABS", \
	8, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_odometer_abs_t, timestamp) }, \
         { "xdist", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_odometer_abs_t, xdist) }, \
         { "ydist", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_odometer_abs_t, ydist) }, \
         { "zdist", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_odometer_abs_t, zdist) }, \
         { "xvelocity", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_odometer_abs_t, xvelocity) }, \
         { "yvelocity", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_odometer_abs_t, yvelocity) }, \
         { "zvelocity", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_odometer_abs_t, zvelocity) }, \
         { "quality", NULL, MAVLINK_TYPE_INT16_T, 0, 28, offsetof(mavlink_odometer_abs_t, quality) }, \
         } \
}


/**
 * @brief Pack a odometer_abs message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp Timestamp of the measurement (us since system start)
 * @param xdist Distance travelled along x-axis [m]
 * @param ydist Distance travelled along y-axis [m]
 * @param zdist Distance travelled along z-axis [m]
 * @param xvelocity Velocity along x-axis [m/s]
 * @param yvelocity Velocity along y-axis [m/s]
 * @param zvelocity Velocity along z-axis [m/s]
 * @param quality Measurement quality indicator (-1 for no quality)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_odometer_abs_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t timestamp, float xdist, float ydist, float zdist, float xvelocity, float yvelocity, float zvelocity, int16_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[30];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float(buf, 4, xdist);
	_mav_put_float(buf, 8, ydist);
	_mav_put_float(buf, 12, zdist);
	_mav_put_float(buf, 16, xvelocity);
	_mav_put_float(buf, 20, yvelocity);
	_mav_put_float(buf, 24, zvelocity);
	_mav_put_int16_t(buf, 28, quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 30);
#else
	mavlink_odometer_abs_t packet;
	packet.timestamp = timestamp;
	packet.xdist = xdist;
	packet.ydist = ydist;
	packet.zdist = zdist;
	packet.xvelocity = xvelocity;
	packet.yvelocity = yvelocity;
	packet.zvelocity = zvelocity;
	packet.quality = quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 30);
#endif

	msg->msgid = MAVLINK_MSG_ID_ODOMETER_ABS;
	return mavlink_finalize_message(msg, system_id, component_id, 30, 192);
}

/**
 * @brief Pack a odometer_abs message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp of the measurement (us since system start)
 * @param xdist Distance travelled along x-axis [m]
 * @param ydist Distance travelled along y-axis [m]
 * @param zdist Distance travelled along z-axis [m]
 * @param xvelocity Velocity along x-axis [m/s]
 * @param yvelocity Velocity along y-axis [m/s]
 * @param zvelocity Velocity along z-axis [m/s]
 * @param quality Measurement quality indicator (-1 for no quality)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_odometer_abs_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t timestamp,float xdist,float ydist,float zdist,float xvelocity,float yvelocity,float zvelocity,int16_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[30];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float(buf, 4, xdist);
	_mav_put_float(buf, 8, ydist);
	_mav_put_float(buf, 12, zdist);
	_mav_put_float(buf, 16, xvelocity);
	_mav_put_float(buf, 20, yvelocity);
	_mav_put_float(buf, 24, zvelocity);
	_mav_put_int16_t(buf, 28, quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 30);
#else
	mavlink_odometer_abs_t packet;
	packet.timestamp = timestamp;
	packet.xdist = xdist;
	packet.ydist = ydist;
	packet.zdist = zdist;
	packet.xvelocity = xvelocity;
	packet.yvelocity = yvelocity;
	packet.zvelocity = zvelocity;
	packet.quality = quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 30);
#endif

	msg->msgid = MAVLINK_MSG_ID_ODOMETER_ABS;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 30, 192);
}

/**
 * @brief Encode a odometer_abs struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param odometer_abs C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_odometer_abs_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_odometer_abs_t* odometer_abs)
{
	return mavlink_msg_odometer_abs_pack(system_id, component_id, msg, odometer_abs->timestamp, odometer_abs->xdist, odometer_abs->ydist, odometer_abs->zdist, odometer_abs->xvelocity, odometer_abs->yvelocity, odometer_abs->zvelocity, odometer_abs->quality);
}

/**
 * @brief Send a odometer_abs message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp of the measurement (us since system start)
 * @param xdist Distance travelled along x-axis [m]
 * @param ydist Distance travelled along y-axis [m]
 * @param zdist Distance travelled along z-axis [m]
 * @param xvelocity Velocity along x-axis [m/s]
 * @param yvelocity Velocity along y-axis [m/s]
 * @param zvelocity Velocity along z-axis [m/s]
 * @param quality Measurement quality indicator (-1 for no quality)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_odometer_abs_send(mavlink_channel_t chan, uint32_t timestamp, float xdist, float ydist, float zdist, float xvelocity, float yvelocity, float zvelocity, int16_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[30];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float(buf, 4, xdist);
	_mav_put_float(buf, 8, ydist);
	_mav_put_float(buf, 12, zdist);
	_mav_put_float(buf, 16, xvelocity);
	_mav_put_float(buf, 20, yvelocity);
	_mav_put_float(buf, 24, zvelocity);
	_mav_put_int16_t(buf, 28, quality);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ODOMETER_ABS, buf, 30, 192);
#else
	mavlink_odometer_abs_t packet;
	packet.timestamp = timestamp;
	packet.xdist = xdist;
	packet.ydist = ydist;
	packet.zdist = zdist;
	packet.xvelocity = xvelocity;
	packet.yvelocity = yvelocity;
	packet.zvelocity = zvelocity;
	packet.quality = quality;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ODOMETER_ABS, (const char *)&packet, 30, 192);
#endif
}

#endif

// MESSAGE ODOMETER_ABS UNPACKING


/**
 * @brief Get field timestamp from odometer_abs message
 *
 * @return Timestamp of the measurement (us since system start)
 */
static inline uint32_t mavlink_msg_odometer_abs_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field xdist from odometer_abs message
 *
 * @return Distance travelled along x-axis [m]
 */
static inline float mavlink_msg_odometer_abs_get_xdist(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field ydist from odometer_abs message
 *
 * @return Distance travelled along y-axis [m]
 */
static inline float mavlink_msg_odometer_abs_get_ydist(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field zdist from odometer_abs message
 *
 * @return Distance travelled along z-axis [m]
 */
static inline float mavlink_msg_odometer_abs_get_zdist(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field xvelocity from odometer_abs message
 *
 * @return Velocity along x-axis [m/s]
 */
static inline float mavlink_msg_odometer_abs_get_xvelocity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field yvelocity from odometer_abs message
 *
 * @return Velocity along y-axis [m/s]
 */
static inline float mavlink_msg_odometer_abs_get_yvelocity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field zvelocity from odometer_abs message
 *
 * @return Velocity along z-axis [m/s]
 */
static inline float mavlink_msg_odometer_abs_get_zvelocity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field quality from odometer_abs message
 *
 * @return Measurement quality indicator (-1 for no quality)
 */
static inline int16_t mavlink_msg_odometer_abs_get_quality(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  28);
}

/**
 * @brief Decode a odometer_abs message into a struct
 *
 * @param msg The message to decode
 * @param odometer_abs C-struct to decode the message contents into
 */
static inline void mavlink_msg_odometer_abs_decode(const mavlink_message_t* msg, mavlink_odometer_abs_t* odometer_abs)
{
#if MAVLINK_NEED_BYTE_SWAP
	odometer_abs->timestamp = mavlink_msg_odometer_abs_get_timestamp(msg);
	odometer_abs->xdist = mavlink_msg_odometer_abs_get_xdist(msg);
	odometer_abs->ydist = mavlink_msg_odometer_abs_get_ydist(msg);
	odometer_abs->zdist = mavlink_msg_odometer_abs_get_zdist(msg);
	odometer_abs->xvelocity = mavlink_msg_odometer_abs_get_xvelocity(msg);
	odometer_abs->yvelocity = mavlink_msg_odometer_abs_get_yvelocity(msg);
	odometer_abs->zvelocity = mavlink_msg_odometer_abs_get_zvelocity(msg);
	odometer_abs->quality = mavlink_msg_odometer_abs_get_quality(msg);
#else
	memcpy(odometer_abs, _MAV_PAYLOAD(msg), 30);
#endif
}
