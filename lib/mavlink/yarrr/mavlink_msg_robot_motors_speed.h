#pragma once
// MESSAGE ROBOT_MOTORS_SPEED PACKING

#define MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED 2


typedef struct __mavlink_robot_motors_speed_t {
 uint16_t speed_left; /*<  Speed of left motor*/
 uint16_t speed_right; /*<  Speed of right motor*/
 uint8_t id; /*<  Parameter identifier*/
} mavlink_robot_motors_speed_t;

#define MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_LEN 5
#define MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_MIN_LEN 5
#define MAVLINK_MSG_ID_2_LEN 5
#define MAVLINK_MSG_ID_2_MIN_LEN 5

#define MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_CRC 91
#define MAVLINK_MSG_ID_2_CRC 91



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROBOT_MOTORS_SPEED { \
    2, \
    "ROBOT_MOTORS_SPEED", \
    3, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_robot_motors_speed_t, id) }, \
         { "speed_left", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_robot_motors_speed_t, speed_left) }, \
         { "speed_right", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_robot_motors_speed_t, speed_right) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROBOT_MOTORS_SPEED { \
    "ROBOT_MOTORS_SPEED", \
    3, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_robot_motors_speed_t, id) }, \
         { "speed_left", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_robot_motors_speed_t, speed_left) }, \
         { "speed_right", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_robot_motors_speed_t, speed_right) }, \
         } \
}
#endif

/**
 * @brief Pack a robot_motors_speed message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param id  Parameter identifier
 * @param speed_left  Speed of left motor
 * @param speed_right  Speed of right motor
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robot_motors_speed_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t id, uint16_t speed_left, uint16_t speed_right)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_LEN];
    _mav_put_uint16_t(buf, 0, speed_left);
    _mav_put_uint16_t(buf, 2, speed_right);
    _mav_put_uint8_t(buf, 4, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_LEN);
#else
    mavlink_robot_motors_speed_t packet;
    packet.speed_left = speed_left;
    packet.speed_right = speed_right;
    packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_MIN_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_CRC);
}

/**
 * @brief Pack a robot_motors_speed message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param id  Parameter identifier
 * @param speed_left  Speed of left motor
 * @param speed_right  Speed of right motor
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robot_motors_speed_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t id,uint16_t speed_left,uint16_t speed_right)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_LEN];
    _mav_put_uint16_t(buf, 0, speed_left);
    _mav_put_uint16_t(buf, 2, speed_right);
    _mav_put_uint8_t(buf, 4, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_LEN);
#else
    mavlink_robot_motors_speed_t packet;
    packet.speed_left = speed_left;
    packet.speed_right = speed_right;
    packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_MIN_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_CRC);
}

/**
 * @brief Encode a robot_motors_speed struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param robot_motors_speed C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_robot_motors_speed_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_robot_motors_speed_t* robot_motors_speed)
{
    return mavlink_msg_robot_motors_speed_pack(system_id, component_id, msg, robot_motors_speed->id, robot_motors_speed->speed_left, robot_motors_speed->speed_right);
}

/**
 * @brief Encode a robot_motors_speed struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param robot_motors_speed C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_robot_motors_speed_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_robot_motors_speed_t* robot_motors_speed)
{
    return mavlink_msg_robot_motors_speed_pack_chan(system_id, component_id, chan, msg, robot_motors_speed->id, robot_motors_speed->speed_left, robot_motors_speed->speed_right);
}

/**
 * @brief Send a robot_motors_speed message
 * @param chan MAVLink channel to send the message
 *
 * @param id  Parameter identifier
 * @param speed_left  Speed of left motor
 * @param speed_right  Speed of right motor
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_robot_motors_speed_send(mavlink_channel_t chan, uint8_t id, uint16_t speed_left, uint16_t speed_right)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_LEN];
    _mav_put_uint16_t(buf, 0, speed_left);
    _mav_put_uint16_t(buf, 2, speed_right);
    _mav_put_uint8_t(buf, 4, id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED, buf, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_MIN_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_CRC);
#else
    mavlink_robot_motors_speed_t packet;
    packet.speed_left = speed_left;
    packet.speed_right = speed_right;
    packet.id = id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED, (const char *)&packet, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_MIN_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_CRC);
#endif
}

/**
 * @brief Send a robot_motors_speed message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_robot_motors_speed_send_struct(mavlink_channel_t chan, const mavlink_robot_motors_speed_t* robot_motors_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_robot_motors_speed_send(chan, robot_motors_speed->id, robot_motors_speed->speed_left, robot_motors_speed->speed_right);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED, (const char *)robot_motors_speed, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_MIN_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_CRC);
#endif
}

#if MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_robot_motors_speed_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t id, uint16_t speed_left, uint16_t speed_right)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, speed_left);
    _mav_put_uint16_t(buf, 2, speed_right);
    _mav_put_uint8_t(buf, 4, id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED, buf, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_MIN_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_CRC);
#else
    mavlink_robot_motors_speed_t *packet = (mavlink_robot_motors_speed_t *)msgbuf;
    packet->speed_left = speed_left;
    packet->speed_right = speed_right;
    packet->id = id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED, (const char *)packet, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_MIN_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_LEN, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_CRC);
#endif
}
#endif

#endif

// MESSAGE ROBOT_MOTORS_SPEED UNPACKING


/**
 * @brief Get field id from robot_motors_speed message
 *
 * @return  Parameter identifier
 */
static inline uint8_t mavlink_msg_robot_motors_speed_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field speed_left from robot_motors_speed message
 *
 * @return  Speed of left motor
 */
static inline uint16_t mavlink_msg_robot_motors_speed_get_speed_left(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field speed_right from robot_motors_speed message
 *
 * @return  Speed of right motor
 */
static inline uint16_t mavlink_msg_robot_motors_speed_get_speed_right(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Decode a robot_motors_speed message into a struct
 *
 * @param msg The message to decode
 * @param robot_motors_speed C-struct to decode the message contents into
 */
static inline void mavlink_msg_robot_motors_speed_decode(const mavlink_message_t* msg, mavlink_robot_motors_speed_t* robot_motors_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    robot_motors_speed->speed_left = mavlink_msg_robot_motors_speed_get_speed_left(msg);
    robot_motors_speed->speed_right = mavlink_msg_robot_motors_speed_get_speed_right(msg);
    robot_motors_speed->id = mavlink_msg_robot_motors_speed_get_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_LEN? msg->len : MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_LEN;
        memset(robot_motors_speed, 0, MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_LEN);
    memcpy(robot_motors_speed, _MAV_PAYLOAD(msg), len);
#endif
}
