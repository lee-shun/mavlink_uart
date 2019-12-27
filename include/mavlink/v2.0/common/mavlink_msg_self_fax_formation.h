#pragma once
// MESSAGE SELF_FAX_FORMATION PACKING

#define MAVLINK_MSG_ID_SELF_FAX_FORMATION 304

MAVPACKED(
typedef struct __mavlink_self_fax_formation_t {
 double ned_vel_x; /*< [m/s] ned_vel_x*/
 double ned_vel_y; /*< [m/s] ned_vel_y*/
 double ned_vel_z; /*< [m/s] ned_vel_z*/
 double latitude; /*<  latitude*/
 double altitude; /*<  altitude*/
 double longtitude; /*<  longtitude*/
}) mavlink_self_fax_formation_t;

#define MAVLINK_MSG_ID_SELF_FAX_FORMATION_LEN 48
#define MAVLINK_MSG_ID_SELF_FAX_FORMATION_MIN_LEN 48
#define MAVLINK_MSG_ID_304_LEN 48
#define MAVLINK_MSG_ID_304_MIN_LEN 48

#define MAVLINK_MSG_ID_SELF_FAX_FORMATION_CRC 208
#define MAVLINK_MSG_ID_304_CRC 208



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SELF_FAX_FORMATION { \
    304, \
    "SELF_FAX_FORMATION", \
    6, \
    {  { "ned_vel_x", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_self_fax_formation_t, ned_vel_x) }, \
         { "ned_vel_y", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_self_fax_formation_t, ned_vel_y) }, \
         { "ned_vel_z", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_self_fax_formation_t, ned_vel_z) }, \
         { "latitude", NULL, MAVLINK_TYPE_DOUBLE, 0, 24, offsetof(mavlink_self_fax_formation_t, latitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_DOUBLE, 0, 32, offsetof(mavlink_self_fax_formation_t, altitude) }, \
         { "longtitude", NULL, MAVLINK_TYPE_DOUBLE, 0, 40, offsetof(mavlink_self_fax_formation_t, longtitude) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SELF_FAX_FORMATION { \
    "SELF_FAX_FORMATION", \
    6, \
    {  { "ned_vel_x", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_self_fax_formation_t, ned_vel_x) }, \
         { "ned_vel_y", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_self_fax_formation_t, ned_vel_y) }, \
         { "ned_vel_z", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_self_fax_formation_t, ned_vel_z) }, \
         { "latitude", NULL, MAVLINK_TYPE_DOUBLE, 0, 24, offsetof(mavlink_self_fax_formation_t, latitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_DOUBLE, 0, 32, offsetof(mavlink_self_fax_formation_t, altitude) }, \
         { "longtitude", NULL, MAVLINK_TYPE_DOUBLE, 0, 40, offsetof(mavlink_self_fax_formation_t, longtitude) }, \
         } \
}
#endif

/**
 * @brief Pack a self_fax_formation message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ned_vel_x [m/s] ned_vel_x
 * @param ned_vel_y [m/s] ned_vel_y
 * @param ned_vel_z [m/s] ned_vel_z
 * @param latitude  latitude
 * @param altitude  altitude
 * @param longtitude  longtitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_self_fax_formation_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               double ned_vel_x, double ned_vel_y, double ned_vel_z, double latitude, double altitude, double longtitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SELF_FAX_FORMATION_LEN];
    _mav_put_double(buf, 0, ned_vel_x);
    _mav_put_double(buf, 8, ned_vel_y);
    _mav_put_double(buf, 16, ned_vel_z);
    _mav_put_double(buf, 24, latitude);
    _mav_put_double(buf, 32, altitude);
    _mav_put_double(buf, 40, longtitude);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SELF_FAX_FORMATION_LEN);
#else
    mavlink_self_fax_formation_t packet;
    packet.ned_vel_x = ned_vel_x;
    packet.ned_vel_y = ned_vel_y;
    packet.ned_vel_z = ned_vel_z;
    packet.latitude = latitude;
    packet.altitude = altitude;
    packet.longtitude = longtitude;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SELF_FAX_FORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SELF_FAX_FORMATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SELF_FAX_FORMATION_MIN_LEN, MAVLINK_MSG_ID_SELF_FAX_FORMATION_LEN, MAVLINK_MSG_ID_SELF_FAX_FORMATION_CRC);
}

/**
 * @brief Pack a self_fax_formation message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ned_vel_x [m/s] ned_vel_x
 * @param ned_vel_y [m/s] ned_vel_y
 * @param ned_vel_z [m/s] ned_vel_z
 * @param latitude  latitude
 * @param altitude  altitude
 * @param longtitude  longtitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_self_fax_formation_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   double ned_vel_x,double ned_vel_y,double ned_vel_z,double latitude,double altitude,double longtitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SELF_FAX_FORMATION_LEN];
    _mav_put_double(buf, 0, ned_vel_x);
    _mav_put_double(buf, 8, ned_vel_y);
    _mav_put_double(buf, 16, ned_vel_z);
    _mav_put_double(buf, 24, latitude);
    _mav_put_double(buf, 32, altitude);
    _mav_put_double(buf, 40, longtitude);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SELF_FAX_FORMATION_LEN);
#else
    mavlink_self_fax_formation_t packet;
    packet.ned_vel_x = ned_vel_x;
    packet.ned_vel_y = ned_vel_y;
    packet.ned_vel_z = ned_vel_z;
    packet.latitude = latitude;
    packet.altitude = altitude;
    packet.longtitude = longtitude;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SELF_FAX_FORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SELF_FAX_FORMATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SELF_FAX_FORMATION_MIN_LEN, MAVLINK_MSG_ID_SELF_FAX_FORMATION_LEN, MAVLINK_MSG_ID_SELF_FAX_FORMATION_CRC);
}

/**
 * @brief Encode a self_fax_formation struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param self_fax_formation C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_self_fax_formation_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_self_fax_formation_t* self_fax_formation)
{
    return mavlink_msg_self_fax_formation_pack(system_id, component_id, msg, self_fax_formation->ned_vel_x, self_fax_formation->ned_vel_y, self_fax_formation->ned_vel_z, self_fax_formation->latitude, self_fax_formation->altitude, self_fax_formation->longtitude);
}

/**
 * @brief Encode a self_fax_formation struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param self_fax_formation C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_self_fax_formation_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_self_fax_formation_t* self_fax_formation)
{
    return mavlink_msg_self_fax_formation_pack_chan(system_id, component_id, chan, msg, self_fax_formation->ned_vel_x, self_fax_formation->ned_vel_y, self_fax_formation->ned_vel_z, self_fax_formation->latitude, self_fax_formation->altitude, self_fax_formation->longtitude);
}

/**
 * @brief Send a self_fax_formation message
 * @param chan MAVLink channel to send the message
 *
 * @param ned_vel_x [m/s] ned_vel_x
 * @param ned_vel_y [m/s] ned_vel_y
 * @param ned_vel_z [m/s] ned_vel_z
 * @param latitude  latitude
 * @param altitude  altitude
 * @param longtitude  longtitude
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_self_fax_formation_send(mavlink_channel_t chan, double ned_vel_x, double ned_vel_y, double ned_vel_z, double latitude, double altitude, double longtitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SELF_FAX_FORMATION_LEN];
    _mav_put_double(buf, 0, ned_vel_x);
    _mav_put_double(buf, 8, ned_vel_y);
    _mav_put_double(buf, 16, ned_vel_z);
    _mav_put_double(buf, 24, latitude);
    _mav_put_double(buf, 32, altitude);
    _mav_put_double(buf, 40, longtitude);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SELF_FAX_FORMATION, buf, MAVLINK_MSG_ID_SELF_FAX_FORMATION_MIN_LEN, MAVLINK_MSG_ID_SELF_FAX_FORMATION_LEN, MAVLINK_MSG_ID_SELF_FAX_FORMATION_CRC);
#else
    mavlink_self_fax_formation_t packet;
    packet.ned_vel_x = ned_vel_x;
    packet.ned_vel_y = ned_vel_y;
    packet.ned_vel_z = ned_vel_z;
    packet.latitude = latitude;
    packet.altitude = altitude;
    packet.longtitude = longtitude;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SELF_FAX_FORMATION, (const char *)&packet, MAVLINK_MSG_ID_SELF_FAX_FORMATION_MIN_LEN, MAVLINK_MSG_ID_SELF_FAX_FORMATION_LEN, MAVLINK_MSG_ID_SELF_FAX_FORMATION_CRC);
#endif
}

/**
 * @brief Send a self_fax_formation message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_self_fax_formation_send_struct(mavlink_channel_t chan, const mavlink_self_fax_formation_t* self_fax_formation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_self_fax_formation_send(chan, self_fax_formation->ned_vel_x, self_fax_formation->ned_vel_y, self_fax_formation->ned_vel_z, self_fax_formation->latitude, self_fax_formation->altitude, self_fax_formation->longtitude);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SELF_FAX_FORMATION, (const char *)self_fax_formation, MAVLINK_MSG_ID_SELF_FAX_FORMATION_MIN_LEN, MAVLINK_MSG_ID_SELF_FAX_FORMATION_LEN, MAVLINK_MSG_ID_SELF_FAX_FORMATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_SELF_FAX_FORMATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_self_fax_formation_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  double ned_vel_x, double ned_vel_y, double ned_vel_z, double latitude, double altitude, double longtitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_double(buf, 0, ned_vel_x);
    _mav_put_double(buf, 8, ned_vel_y);
    _mav_put_double(buf, 16, ned_vel_z);
    _mav_put_double(buf, 24, latitude);
    _mav_put_double(buf, 32, altitude);
    _mav_put_double(buf, 40, longtitude);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SELF_FAX_FORMATION, buf, MAVLINK_MSG_ID_SELF_FAX_FORMATION_MIN_LEN, MAVLINK_MSG_ID_SELF_FAX_FORMATION_LEN, MAVLINK_MSG_ID_SELF_FAX_FORMATION_CRC);
#else
    mavlink_self_fax_formation_t *packet = (mavlink_self_fax_formation_t *)msgbuf;
    packet->ned_vel_x = ned_vel_x;
    packet->ned_vel_y = ned_vel_y;
    packet->ned_vel_z = ned_vel_z;
    packet->latitude = latitude;
    packet->altitude = altitude;
    packet->longtitude = longtitude;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SELF_FAX_FORMATION, (const char *)packet, MAVLINK_MSG_ID_SELF_FAX_FORMATION_MIN_LEN, MAVLINK_MSG_ID_SELF_FAX_FORMATION_LEN, MAVLINK_MSG_ID_SELF_FAX_FORMATION_CRC);
#endif
}
#endif

#endif

// MESSAGE SELF_FAX_FORMATION UNPACKING


/**
 * @brief Get field ned_vel_x from self_fax_formation message
 *
 * @return [m/s] ned_vel_x
 */
static inline double mavlink_msg_self_fax_formation_get_ned_vel_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  0);
}

/**
 * @brief Get field ned_vel_y from self_fax_formation message
 *
 * @return [m/s] ned_vel_y
 */
static inline double mavlink_msg_self_fax_formation_get_ned_vel_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  8);
}

/**
 * @brief Get field ned_vel_z from self_fax_formation message
 *
 * @return [m/s] ned_vel_z
 */
static inline double mavlink_msg_self_fax_formation_get_ned_vel_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  16);
}

/**
 * @brief Get field latitude from self_fax_formation message
 *
 * @return  latitude
 */
static inline double mavlink_msg_self_fax_formation_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  24);
}

/**
 * @brief Get field altitude from self_fax_formation message
 *
 * @return  altitude
 */
static inline double mavlink_msg_self_fax_formation_get_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  32);
}

/**
 * @brief Get field longtitude from self_fax_formation message
 *
 * @return  longtitude
 */
static inline double mavlink_msg_self_fax_formation_get_longtitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  40);
}

/**
 * @brief Decode a self_fax_formation message into a struct
 *
 * @param msg The message to decode
 * @param self_fax_formation C-struct to decode the message contents into
 */
static inline void mavlink_msg_self_fax_formation_decode(const mavlink_message_t* msg, mavlink_self_fax_formation_t* self_fax_formation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    self_fax_formation->ned_vel_x = mavlink_msg_self_fax_formation_get_ned_vel_x(msg);
    self_fax_formation->ned_vel_y = mavlink_msg_self_fax_formation_get_ned_vel_y(msg);
    self_fax_formation->ned_vel_z = mavlink_msg_self_fax_formation_get_ned_vel_z(msg);
    self_fax_formation->latitude = mavlink_msg_self_fax_formation_get_latitude(msg);
    self_fax_formation->altitude = mavlink_msg_self_fax_formation_get_altitude(msg);
    self_fax_formation->longtitude = mavlink_msg_self_fax_formation_get_longtitude(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SELF_FAX_FORMATION_LEN? msg->len : MAVLINK_MSG_ID_SELF_FAX_FORMATION_LEN;
        memset(self_fax_formation, 0, MAVLINK_MSG_ID_SELF_FAX_FORMATION_LEN);
    memcpy(self_fax_formation, _MAV_PAYLOAD(msg), len);
#endif
}
