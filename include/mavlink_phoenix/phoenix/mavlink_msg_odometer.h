// MESSAGE ODOMETER PACKING

#define MAVLINK_MSG_ID_ODOMETER 140

typedef struct __mavlink_odometer_t
{
 uint32_t timestamp; ///< Timestamp of the measurement (us since system start)
 float time_delta; ///< Time-delta within which the measurement has taken place [s]
 float xdist_delta; ///< Distance travelled along x-axis within period [m]
 float ydist_delta; ///< Distance travelled along y-axis within period [m]
 float zdist_delta; ///< Distance travelled along z-axis within period [m]
 float xdist_abs; ///< Distance travelled along x-axis [m]
 float ydist_abs; ///< Distance travelled along y-axis [m]
 float zdist_abs; ///< Distance travelled along z-axis [m]
 float xvelocity; ///< Velocity along x-axis [m/s]
 float yvelocity; ///< Velocity along y-axis [m/s]
 float zvelocity; ///< Velocity along z-axis [m/s]
 int16_t quality; ///< Measurement quality indicator (-1 for no quality)
} mavlink_odometer_t;

#define MAVLINK_MSG_ID_ODOMETER_LEN 46
#define MAVLINK_MSG_ID_140_LEN 46



#define MAVLINK_MESSAGE_INFO_ODOMETER { \
	"ODOMETER", \
	12, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_odometer_t, timestamp) }, \
         { "time_delta", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_odometer_t, time_delta) }, \
         { "xdist_delta", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_odometer_t, xdist_delta) }, \
         { "ydist_delta", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_odometer_t, ydist_delta) }, \
         { "zdist_delta", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_odometer_t, zdist_delta) }, \
         { "xdist_abs", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_odometer_t, xdist_abs) }, \
         { "ydist_abs", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_odometer_t, ydist_abs) }, \
         { "zdist_abs", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_odometer_t, zdist_abs) }, \
         { "xvelocity", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_odometer_t, xvelocity) }, \
         { "yvelocity", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_odometer_t, yvelocity) }, \
         { "zvelocity", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_odometer_t, zvelocity) }, \
         { "quality", NULL, MAVLINK_TYPE_INT16_T, 0, 44, offsetof(mavlink_odometer_t, quality) }, \
         } \
}


/**
 * @brief Pack a odometer message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp Timestamp of the measurement (us since system start)
 * @param time_delta Time-delta within which the measurement has taken place [s]
 * @param xdist_delta Distance travelled along x-axis within period [m]
 * @param ydist_delta Distance travelled along y-axis within period [m]
 * @param zdist_delta Distance travelled along z-axis within period [m]
 * @param xdist_abs Distance travelled along x-axis [m]
 * @param ydist_abs Distance travelled along y-axis [m]
 * @param zdist_abs Distance travelled along z-axis [m]
 * @param xvelocity Velocity along x-axis [m/s]
 * @param yvelocity Velocity along y-axis [m/s]
 * @param zvelocity Velocity along z-axis [m/s]
 * @param quality Measurement quality indicator (-1 for no quality)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_odometer_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t timestamp, float time_delta, float xdist_delta, float ydist_delta, float zdist_delta, float xdist_abs, float ydist_abs, float zdist_abs, float xvelocity, float yvelocity, float zvelocity, int16_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[46];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float(buf, 4, time_delta);
	_mav_put_float(buf, 8, xdist_delta);
	_mav_put_float(buf, 12, ydist_delta);
	_mav_put_float(buf, 16, zdist_delta);
	_mav_put_float(buf, 20, xdist_abs);
	_mav_put_float(buf, 24, ydist_abs);
	_mav_put_float(buf, 28, zdist_abs);
	_mav_put_float(buf, 32, xvelocity);
	_mav_put_float(buf, 36, yvelocity);
	_mav_put_float(buf, 40, zvelocity);
	_mav_put_int16_t(buf, 44, quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 46);
#else
	mavlink_odometer_t packet;
	packet.timestamp = timestamp;
	packet.time_delta = time_delta;
	packet.xdist_delta = xdist_delta;
	packet.ydist_delta = ydist_delta;
	packet.zdist_delta = zdist_delta;
	packet.xdist_abs = xdist_abs;
	packet.ydist_abs = ydist_abs;
	packet.zdist_abs = zdist_abs;
	packet.xvelocity = xvelocity;
	packet.yvelocity = yvelocity;
	packet.zvelocity = zvelocity;
	packet.quality = quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 46);
#endif

	msg->msgid = MAVLINK_MSG_ID_ODOMETER;
	return mavlink_finalize_message(msg, system_id, component_id, 46, 23);
}

/**
 * @brief Pack a odometer message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp of the measurement (us since system start)
 * @param time_delta Time-delta within which the measurement has taken place [s]
 * @param xdist_delta Distance travelled along x-axis within period [m]
 * @param ydist_delta Distance travelled along y-axis within period [m]
 * @param zdist_delta Distance travelled along z-axis within period [m]
 * @param xdist_abs Distance travelled along x-axis [m]
 * @param ydist_abs Distance travelled along y-axis [m]
 * @param zdist_abs Distance travelled along z-axis [m]
 * @param xvelocity Velocity along x-axis [m/s]
 * @param yvelocity Velocity along y-axis [m/s]
 * @param zvelocity Velocity along z-axis [m/s]
 * @param quality Measurement quality indicator (-1 for no quality)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_odometer_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t timestamp,float time_delta,float xdist_delta,float ydist_delta,float zdist_delta,float xdist_abs,float ydist_abs,float zdist_abs,float xvelocity,float yvelocity,float zvelocity,int16_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[46];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float(buf, 4, time_delta);
	_mav_put_float(buf, 8, xdist_delta);
	_mav_put_float(buf, 12, ydist_delta);
	_mav_put_float(buf, 16, zdist_delta);
	_mav_put_float(buf, 20, xdist_abs);
	_mav_put_float(buf, 24, ydist_abs);
	_mav_put_float(buf, 28, zdist_abs);
	_mav_put_float(buf, 32, xvelocity);
	_mav_put_float(buf, 36, yvelocity);
	_mav_put_float(buf, 40, zvelocity);
	_mav_put_int16_t(buf, 44, quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 46);
#else
	mavlink_odometer_t packet;
	packet.timestamp = timestamp;
	packet.time_delta = time_delta;
	packet.xdist_delta = xdist_delta;
	packet.ydist_delta = ydist_delta;
	packet.zdist_delta = zdist_delta;
	packet.xdist_abs = xdist_abs;
	packet.ydist_abs = ydist_abs;
	packet.zdist_abs = zdist_abs;
	packet.xvelocity = xvelocity;
	packet.yvelocity = yvelocity;
	packet.zvelocity = zvelocity;
	packet.quality = quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 46);
#endif

	msg->msgid = MAVLINK_MSG_ID_ODOMETER;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 46, 23);
}

/**
 * @brief Encode a odometer struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param odometer C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_odometer_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_odometer_t* odometer)
{
	return mavlink_msg_odometer_pack(system_id, component_id, msg, odometer->timestamp, odometer->time_delta, odometer->xdist_delta, odometer->ydist_delta, odometer->zdist_delta, odometer->xdist_abs, odometer->ydist_abs, odometer->zdist_abs, odometer->xvelocity, odometer->yvelocity, odometer->zvelocity, odometer->quality);
}

/**
 * @brief Send a odometer message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp of the measurement (us since system start)
 * @param time_delta Time-delta within which the measurement has taken place [s]
 * @param xdist_delta Distance travelled along x-axis within period [m]
 * @param ydist_delta Distance travelled along y-axis within period [m]
 * @param zdist_delta Distance travelled along z-axis within period [m]
 * @param xdist_abs Distance travelled along x-axis [m]
 * @param ydist_abs Distance travelled along y-axis [m]
 * @param zdist_abs Distance travelled along z-axis [m]
 * @param xvelocity Velocity along x-axis [m/s]
 * @param yvelocity Velocity along y-axis [m/s]
 * @param zvelocity Velocity along z-axis [m/s]
 * @param quality Measurement quality indicator (-1 for no quality)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_odometer_send(mavlink_channel_t chan, uint32_t timestamp, float time_delta, float xdist_delta, float ydist_delta, float zdist_delta, float xdist_abs, float ydist_abs, float zdist_abs, float xvelocity, float yvelocity, float zvelocity, int16_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[46];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float(buf, 4, time_delta);
	_mav_put_float(buf, 8, xdist_delta);
	_mav_put_float(buf, 12, ydist_delta);
	_mav_put_float(buf, 16, zdist_delta);
	_mav_put_float(buf, 20, xdist_abs);
	_mav_put_float(buf, 24, ydist_abs);
	_mav_put_float(buf, 28, zdist_abs);
	_mav_put_float(buf, 32, xvelocity);
	_mav_put_float(buf, 36, yvelocity);
	_mav_put_float(buf, 40, zvelocity);
	_mav_put_int16_t(buf, 44, quality);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ODOMETER, buf, 46, 23);
#else
	mavlink_odometer_t packet;
	packet.timestamp = timestamp;
	packet.time_delta = time_delta;
	packet.xdist_delta = xdist_delta;
	packet.ydist_delta = ydist_delta;
	packet.zdist_delta = zdist_delta;
	packet.xdist_abs = xdist_abs;
	packet.ydist_abs = ydist_abs;
	packet.zdist_abs = zdist_abs;
	packet.xvelocity = xvelocity;
	packet.yvelocity = yvelocity;
	packet.zvelocity = zvelocity;
	packet.quality = quality;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ODOMETER, (const char *)&packet, 46, 23);
#endif
}

#endif

// MESSAGE ODOMETER UNPACKING


/**
 * @brief Get field timestamp from odometer message
 *
 * @return Timestamp of the measurement (us since system start)
 */
static inline uint32_t mavlink_msg_odometer_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field time_delta from odometer message
 *
 * @return Time-delta within which the measurement has taken place [s]
 */
static inline float mavlink_msg_odometer_get_time_delta(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field xdist_delta from odometer message
 *
 * @return Distance travelled along x-axis within period [m]
 */
static inline float mavlink_msg_odometer_get_xdist_delta(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field ydist_delta from odometer message
 *
 * @return Distance travelled along y-axis within period [m]
 */
static inline float mavlink_msg_odometer_get_ydist_delta(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field zdist_delta from odometer message
 *
 * @return Distance travelled along z-axis within period [m]
 */
static inline float mavlink_msg_odometer_get_zdist_delta(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field xdist_abs from odometer message
 *
 * @return Distance travelled along x-axis [m]
 */
static inline float mavlink_msg_odometer_get_xdist_abs(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field ydist_abs from odometer message
 *
 * @return Distance travelled along y-axis [m]
 */
static inline float mavlink_msg_odometer_get_ydist_abs(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field zdist_abs from odometer message
 *
 * @return Distance travelled along z-axis [m]
 */
static inline float mavlink_msg_odometer_get_zdist_abs(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field xvelocity from odometer message
 *
 * @return Velocity along x-axis [m/s]
 */
static inline float mavlink_msg_odometer_get_xvelocity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field yvelocity from odometer message
 *
 * @return Velocity along y-axis [m/s]
 */
static inline float mavlink_msg_odometer_get_yvelocity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field zvelocity from odometer message
 *
 * @return Velocity along z-axis [m/s]
 */
static inline float mavlink_msg_odometer_get_zvelocity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field quality from odometer message
 *
 * @return Measurement quality indicator (-1 for no quality)
 */
static inline int16_t mavlink_msg_odometer_get_quality(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  44);
}

/**
 * @brief Decode a odometer message into a struct
 *
 * @param msg The message to decode
 * @param odometer C-struct to decode the message contents into
 */
static inline void mavlink_msg_odometer_decode(const mavlink_message_t* msg, mavlink_odometer_t* odometer)
{
#if MAVLINK_NEED_BYTE_SWAP
	odometer->timestamp = mavlink_msg_odometer_get_timestamp(msg);
	odometer->time_delta = mavlink_msg_odometer_get_time_delta(msg);
	odometer->xdist_delta = mavlink_msg_odometer_get_xdist_delta(msg);
	odometer->ydist_delta = mavlink_msg_odometer_get_ydist_delta(msg);
	odometer->zdist_delta = mavlink_msg_odometer_get_zdist_delta(msg);
	odometer->xdist_abs = mavlink_msg_odometer_get_xdist_abs(msg);
	odometer->ydist_abs = mavlink_msg_odometer_get_ydist_abs(msg);
	odometer->zdist_abs = mavlink_msg_odometer_get_zdist_abs(msg);
	odometer->xvelocity = mavlink_msg_odometer_get_xvelocity(msg);
	odometer->yvelocity = mavlink_msg_odometer_get_yvelocity(msg);
	odometer->zvelocity = mavlink_msg_odometer_get_zvelocity(msg);
	odometer->quality = mavlink_msg_odometer_get_quality(msg);
#else
	memcpy(odometer, _MAV_PAYLOAD(msg), 46);
#endif
}
