#pragma once
// MESSAGE ROBOT_MOTORS_PIDS PACKING

#define MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS 0


typedef struct __mavlink_robot_motors_pids_t {
 float kp; /*<  Kp*/
 float ki; /*<  Ki*/
 float kd; /*<  Kd*/
 uint16_t sample_rate; /*<  Time for pids refresh*/
 uint8_t id; /*<  Parameter identifier*/
} mavlink_robot_motors_pids_t;

#define MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_LEN 15
#define MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_MIN_LEN 15
#define MAVLINK_MSG_ID_0_LEN 15
#define MAVLINK_MSG_ID_0_MIN_LEN 15

#define MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_CRC 57
#define MAVLINK_MSG_ID_0_CRC 57



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROBOT_MOTORS_PIDS { \
    0, \
    "ROBOT_MOTORS_PIDS", \
    5, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_robot_motors_pids_t, id) }, \
         { "kp", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_robot_motors_pids_t, kp) }, \
         { "ki", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_robot_motors_pids_t, ki) }, \
         { "kd", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_robot_motors_pids_t, kd) }, \
         { "sample_rate", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_robot_motors_pids_t, sample_rate) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROBOT_MOTORS_PIDS { \
    "ROBOT_MOTORS_PIDS", \
    5, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_robot_motors_pids_t, id) }, \
         { "kp", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_robot_motors_pids_t, kp) }, \
         { "ki", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_robot_motors_pids_t, ki) }, \
         { "kd", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_robot_motors_pids_t, kd) }, \
         { "sample_rate", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_robot_motors_pids_t, sample_rate) }, \
         } \
}
#endif

/**
 * @brief Pack a robot_motors_pids message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param id  Parameter identifier
 * @param kp  Kp
 * @param ki  Ki
 * @param kd  Kd
 * @param sample_rate  Time for pids refresh
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robot_motors_pids_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t id, float kp, float ki, float kd, uint16_t sample_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_LEN];
    _mav_put_float(buf, 0, kp);
    _mav_put_float(buf, 4, ki);
    _mav_put_float(buf, 8, kd);
    _mav_put_uint16_t(buf, 12, sample_rate);
    _mav_put_uint8_t(buf, 14, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_LEN);
#else
    mavlink_robot_motors_pids_t packet;
    packet.kp = kp;
    packet.ki = ki;
    packet.kd = kd;
    packet.sample_rate = sample_rate;
    packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_CRC);
}

/**
 * @brief Pack a robot_motors_pids message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param id  Parameter identifier
 * @param kp  Kp
 * @param ki  Ki
 * @param kd  Kd
 * @param sample_rate  Time for pids refresh
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robot_motors_pids_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t id,float kp,float ki,float kd,uint16_t sample_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_LEN];
    _mav_put_float(buf, 0, kp);
    _mav_put_float(buf, 4, ki);
    _mav_put_float(buf, 8, kd);
    _mav_put_uint16_t(buf, 12, sample_rate);
    _mav_put_uint8_t(buf, 14, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_LEN);
#else
    mavlink_robot_motors_pids_t packet;
    packet.kp = kp;
    packet.ki = ki;
    packet.kd = kd;
    packet.sample_rate = sample_rate;
    packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_CRC);
}

/**
 * @brief Encode a robot_motors_pids struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param robot_motors_pids C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_robot_motors_pids_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_robot_motors_pids_t* robot_motors_pids)
{
    return mavlink_msg_robot_motors_pids_pack(system_id, component_id, msg, robot_motors_pids->id, robot_motors_pids->kp, robot_motors_pids->ki, robot_motors_pids->kd, robot_motors_pids->sample_rate);
}

/**
 * @brief Encode a robot_motors_pids struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param robot_motors_pids C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_robot_motors_pids_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_robot_motors_pids_t* robot_motors_pids)
{
    return mavlink_msg_robot_motors_pids_pack_chan(system_id, component_id, chan, msg, robot_motors_pids->id, robot_motors_pids->kp, robot_motors_pids->ki, robot_motors_pids->kd, robot_motors_pids->sample_rate);
}

/**
 * @brief Send a robot_motors_pids message
 * @param chan MAVLink channel to send the message
 *
 * @param id  Parameter identifier
 * @param kp  Kp
 * @param ki  Ki
 * @param kd  Kd
 * @param sample_rate  Time for pids refresh
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_robot_motors_pids_send(mavlink_channel_t chan, uint8_t id, float kp, float ki, float kd, uint16_t sample_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_LEN];
    _mav_put_float(buf, 0, kp);
    _mav_put_float(buf, 4, ki);
    _mav_put_float(buf, 8, kd);
    _mav_put_uint16_t(buf, 12, sample_rate);
    _mav_put_uint8_t(buf, 14, id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS, buf, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_CRC);
#else
    mavlink_robot_motors_pids_t packet;
    packet.kp = kp;
    packet.ki = ki;
    packet.kd = kd;
    packet.sample_rate = sample_rate;
    packet.id = id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS, (const char *)&packet, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_CRC);
#endif
}

/**
 * @brief Send a robot_motors_pids message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_robot_motors_pids_send_struct(mavlink_channel_t chan, const mavlink_robot_motors_pids_t* robot_motors_pids)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_robot_motors_pids_send(chan, robot_motors_pids->id, robot_motors_pids->kp, robot_motors_pids->ki, robot_motors_pids->kd, robot_motors_pids->sample_rate);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS, (const char *)robot_motors_pids, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_robot_motors_pids_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t id, float kp, float ki, float kd, uint16_t sample_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, kp);
    _mav_put_float(buf, 4, ki);
    _mav_put_float(buf, 8, kd);
    _mav_put_uint16_t(buf, 12, sample_rate);
    _mav_put_uint8_t(buf, 14, id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS, buf, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_CRC);
#else
    mavlink_robot_motors_pids_t *packet = (mavlink_robot_motors_pids_t *)msgbuf;
    packet->kp = kp;
    packet->ki = ki;
    packet->kd = kd;
    packet->sample_rate = sample_rate;
    packet->id = id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS, (const char *)packet, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_CRC);
#endif
}
#endif

#endif

// MESSAGE ROBOT_MOTORS_PIDS UNPACKING


/**
 * @brief Get field id from robot_motors_pids message
 *
 * @return  Parameter identifier
 */
static inline uint8_t mavlink_msg_robot_motors_pids_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field kp from robot_motors_pids message
 *
 * @return  Kp
 */
static inline float mavlink_msg_robot_motors_pids_get_kp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field ki from robot_motors_pids message
 *
 * @return  Ki
 */
static inline float mavlink_msg_robot_motors_pids_get_ki(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field kd from robot_motors_pids message
 *
 * @return  Kd
 */
static inline float mavlink_msg_robot_motors_pids_get_kd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field sample_rate from robot_motors_pids message
 *
 * @return  Time for pids refresh
 */
static inline uint16_t mavlink_msg_robot_motors_pids_get_sample_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Decode a robot_motors_pids message into a struct
 *
 * @param msg The message to decode
 * @param robot_motors_pids C-struct to decode the message contents into
 */
static inline void mavlink_msg_robot_motors_pids_decode(const mavlink_message_t* msg, mavlink_robot_motors_pids_t* robot_motors_pids)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    robot_motors_pids->kp = mavlink_msg_robot_motors_pids_get_kp(msg);
    robot_motors_pids->ki = mavlink_msg_robot_motors_pids_get_ki(msg);
    robot_motors_pids->kd = mavlink_msg_robot_motors_pids_get_kd(msg);
    robot_motors_pids->sample_rate = mavlink_msg_robot_motors_pids_get_sample_rate(msg);
    robot_motors_pids->id = mavlink_msg_robot_motors_pids_get_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_LEN? msg->len : MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_LEN;
        memset(robot_motors_pids, 0, MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_LEN);
    memcpy(robot_motors_pids, _MAV_PAYLOAD(msg), len);
#endif
}
