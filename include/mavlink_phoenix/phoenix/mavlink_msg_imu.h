// MESSAGE IMU PACKING

#define MAVLINK_MSG_ID_IMU 128

typedef struct __mavlink_imu_t
{
 uint32_t timestamp; ///< Timestamp of the measurement (us since system start)
 float xacc; ///< Linear acceleration along x-axis [g]
 float yacc; ///< Linear acceleration along y-axis [g]
 float zacc; ///< Linear acceleration along z-axis [g]
 float xgyro; ///< Angular velocity around x-axis [rad/s]
 float ygyro; ///< Angular velocity around y-axis [rad/s]
 float zgyro; ///< Angular velocity around z-axis [rad/s]
 float xmag; ///< Magnetic field strength along x-axis [T]
 float ymag; ///< Magnetic field strength along y-axis [T]
 float zmag; ///< Magnetic field strength along z-axis [T]
} mavlink_imu_t;

#define MAVLINK_MSG_ID_IMU_LEN 40
#define MAVLINK_MSG_ID_128_LEN 40



#define MAVLINK_MESSAGE_INFO_IMU { \
	"IMU", \
	10, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_imu_t, timestamp) }, \
         { "xacc", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_imu_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_imu_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_imu_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_imu_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_imu_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_imu_t, zgyro) }, \
         { "xmag", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_imu_t, xmag) }, \
         { "ymag", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_imu_t, ymag) }, \
         { "zmag", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_imu_t, zmag) }, \
         } \
}


/**
 * @brief Pack a imu message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp Timestamp of the measurement (us since system start)
 * @param xacc Linear acceleration along x-axis [g]
 * @param yacc Linear acceleration along y-axis [g]
 * @param zacc Linear acceleration along z-axis [g]
 * @param xgyro Angular velocity around x-axis [rad/s]
 * @param ygyro Angular velocity around y-axis [rad/s]
 * @param zgyro Angular velocity around z-axis [rad/s]
 * @param xmag Magnetic field strength along x-axis [T]
 * @param ymag Magnetic field strength along y-axis [T]
 * @param zmag Magnetic field strength along z-axis [T]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_imu_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t timestamp, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[40];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float(buf, 4, xacc);
	_mav_put_float(buf, 8, yacc);
	_mav_put_float(buf, 12, zacc);
	_mav_put_float(buf, 16, xgyro);
	_mav_put_float(buf, 20, ygyro);
	_mav_put_float(buf, 24, zgyro);
	_mav_put_float(buf, 28, xmag);
	_mav_put_float(buf, 32, ymag);
	_mav_put_float(buf, 36, zmag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 40);
#else
	mavlink_imu_t packet;
	packet.timestamp = timestamp;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xmag = xmag;
	packet.ymag = ymag;
	packet.zmag = zmag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 40);
#endif

	msg->msgid = MAVLINK_MSG_ID_IMU;
	return mavlink_finalize_message(msg, system_id, component_id, 40, 19);
}

/**
 * @brief Pack a imu message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp of the measurement (us since system start)
 * @param xacc Linear acceleration along x-axis [g]
 * @param yacc Linear acceleration along y-axis [g]
 * @param zacc Linear acceleration along z-axis [g]
 * @param xgyro Angular velocity around x-axis [rad/s]
 * @param ygyro Angular velocity around y-axis [rad/s]
 * @param zgyro Angular velocity around z-axis [rad/s]
 * @param xmag Magnetic field strength along x-axis [T]
 * @param ymag Magnetic field strength along y-axis [T]
 * @param zmag Magnetic field strength along z-axis [T]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_imu_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t timestamp,float xacc,float yacc,float zacc,float xgyro,float ygyro,float zgyro,float xmag,float ymag,float zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[40];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float(buf, 4, xacc);
	_mav_put_float(buf, 8, yacc);
	_mav_put_float(buf, 12, zacc);
	_mav_put_float(buf, 16, xgyro);
	_mav_put_float(buf, 20, ygyro);
	_mav_put_float(buf, 24, zgyro);
	_mav_put_float(buf, 28, xmag);
	_mav_put_float(buf, 32, ymag);
	_mav_put_float(buf, 36, zmag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 40);
#else
	mavlink_imu_t packet;
	packet.timestamp = timestamp;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xmag = xmag;
	packet.ymag = ymag;
	packet.zmag = zmag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 40);
#endif

	msg->msgid = MAVLINK_MSG_ID_IMU;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 40, 19);
}

/**
 * @brief Encode a imu struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_imu_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_imu_t* imu)
{
	return mavlink_msg_imu_pack(system_id, component_id, msg, imu->timestamp, imu->xacc, imu->yacc, imu->zacc, imu->xgyro, imu->ygyro, imu->zgyro, imu->xmag, imu->ymag, imu->zmag);
}

/**
 * @brief Send a imu message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp of the measurement (us since system start)
 * @param xacc Linear acceleration along x-axis [g]
 * @param yacc Linear acceleration along y-axis [g]
 * @param zacc Linear acceleration along z-axis [g]
 * @param xgyro Angular velocity around x-axis [rad/s]
 * @param ygyro Angular velocity around y-axis [rad/s]
 * @param zgyro Angular velocity around z-axis [rad/s]
 * @param xmag Magnetic field strength along x-axis [T]
 * @param ymag Magnetic field strength along y-axis [T]
 * @param zmag Magnetic field strength along z-axis [T]
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_imu_send(mavlink_channel_t chan, uint32_t timestamp, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[40];
	_mav_put_uint32_t(buf, 0, timestamp);
	_mav_put_float(buf, 4, xacc);
	_mav_put_float(buf, 8, yacc);
	_mav_put_float(buf, 12, zacc);
	_mav_put_float(buf, 16, xgyro);
	_mav_put_float(buf, 20, ygyro);
	_mav_put_float(buf, 24, zgyro);
	_mav_put_float(buf, 28, xmag);
	_mav_put_float(buf, 32, ymag);
	_mav_put_float(buf, 36, zmag);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU, buf, 40, 19);
#else
	mavlink_imu_t packet;
	packet.timestamp = timestamp;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xmag = xmag;
	packet.ymag = ymag;
	packet.zmag = zmag;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU, (const char *)&packet, 40, 19);
#endif
}

#endif

// MESSAGE IMU UNPACKING


/**
 * @brief Get field timestamp from imu message
 *
 * @return Timestamp of the measurement (us since system start)
 */
static inline uint32_t mavlink_msg_imu_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field xacc from imu message
 *
 * @return Linear acceleration along x-axis [g]
 */
static inline float mavlink_msg_imu_get_xacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field yacc from imu message
 *
 * @return Linear acceleration along y-axis [g]
 */
static inline float mavlink_msg_imu_get_yacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field zacc from imu message
 *
 * @return Linear acceleration along z-axis [g]
 */
static inline float mavlink_msg_imu_get_zacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field xgyro from imu message
 *
 * @return Angular velocity around x-axis [rad/s]
 */
static inline float mavlink_msg_imu_get_xgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field ygyro from imu message
 *
 * @return Angular velocity around y-axis [rad/s]
 */
static inline float mavlink_msg_imu_get_ygyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field zgyro from imu message
 *
 * @return Angular velocity around z-axis [rad/s]
 */
static inline float mavlink_msg_imu_get_zgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field xmag from imu message
 *
 * @return Magnetic field strength along x-axis [T]
 */
static inline float mavlink_msg_imu_get_xmag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field ymag from imu message
 *
 * @return Magnetic field strength along y-axis [T]
 */
static inline float mavlink_msg_imu_get_ymag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field zmag from imu message
 *
 * @return Magnetic field strength along z-axis [T]
 */
static inline float mavlink_msg_imu_get_zmag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Decode a imu message into a struct
 *
 * @param msg The message to decode
 * @param imu C-struct to decode the message contents into
 */
static inline void mavlink_msg_imu_decode(const mavlink_message_t* msg, mavlink_imu_t* imu)
{
#if MAVLINK_NEED_BYTE_SWAP
	imu->timestamp = mavlink_msg_imu_get_timestamp(msg);
	imu->xacc = mavlink_msg_imu_get_xacc(msg);
	imu->yacc = mavlink_msg_imu_get_yacc(msg);
	imu->zacc = mavlink_msg_imu_get_zacc(msg);
	imu->xgyro = mavlink_msg_imu_get_xgyro(msg);
	imu->ygyro = mavlink_msg_imu_get_ygyro(msg);
	imu->zgyro = mavlink_msg_imu_get_zgyro(msg);
	imu->xmag = mavlink_msg_imu_get_xmag(msg);
	imu->ymag = mavlink_msg_imu_get_ymag(msg);
	imu->zmag = mavlink_msg_imu_get_zmag(msg);
#else
	memcpy(imu, _MAV_PAYLOAD(msg), 40);
#endif
}
