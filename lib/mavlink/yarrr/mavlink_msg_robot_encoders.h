#pragma once
// MESSAGE ROBOT_ENCODERS PACKING

#define MAVLINK_MSG_ID_ROBOT_ENCODERS 1


typedef struct __mavlink_robot_encoders_t {
 uint32_t total_left; /*<  Total value*/
 uint32_t total_right; /*<  Total value*/
 int32_t delta_left; /*<  Increment from last message*/
 int32_t delta_right; /*<  Increment from last message*/
 uint8_t id; /*<  Parameter identifier*/
} mavlink_robot_encoders_t;

#define MAVLINK_MSG_ID_ROBOT_ENCODERS_LEN 17
#define MAVLINK_MSG_ID_ROBOT_ENCODERS_MIN_LEN 17
#define MAVLINK_MSG_ID_1_LEN 17
#define MAVLINK_MSG_ID_1_MIN_LEN 17

#define MAVLINK_MSG_ID_ROBOT_ENCODERS_CRC 17
#define MAVLINK_MSG_ID_1_CRC 17



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROBOT_ENCODERS { \
    1, \
    "ROBOT_ENCODERS", \
    5, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_robot_encoders_t, id) }, \
         { "total_left", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_robot_encoders_t, total_left) }, \
         { "total_right", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_robot_encoders_t, total_right) }, \
         { "delta_left", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_robot_encoders_t, delta_left) }, \
         { "delta_right", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_robot_encoders_t, delta_right) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROBOT_ENCODERS { \
    "ROBOT_ENCODERS", \
    5, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_robot_encoders_t, id) }, \
         { "total_left", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_robot_encoders_t, total_left) }, \
         { "total_right", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_robot_encoders_t, total_right) }, \
         { "delta_left", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_robot_encoders_t, delta_left) }, \
         { "delta_right", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_robot_encoders_t, delta_right) }, \
         } \
}
#endif

/**
 * @brief Pack a robot_encoders message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param id  Parameter identifier
 * @param total_left  Total value
 * @param total_right  Total value
 * @param delta_left  Increment from last message
 * @param delta_right  Increment from last message
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robot_encoders_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t id, uint32_t total_left, uint32_t total_right, int32_t delta_left, int32_t delta_right)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOT_ENCODERS_LEN];
    _mav_put_uint32_t(buf, 0, total_left);
    _mav_put_uint32_t(buf, 4, total_right);
    _mav_put_int32_t(buf, 8, delta_left);
    _mav_put_int32_t(buf, 12, delta_right);
    _mav_put_uint8_t(buf, 16, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOT_ENCODERS_LEN);
#else
    mavlink_robot_encoders_t packet;
    packet.total_left = total_left;
    packet.total_right = total_right;
    packet.delta_left = delta_left;
    packet.delta_right = delta_right;
    packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROBOT_ENCODERS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROBOT_ENCODERS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROBOT_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_ENCODERS_LEN, MAVLINK_MSG_ID_ROBOT_ENCODERS_CRC);
}

/**
 * @brief Pack a robot_encoders message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param id  Parameter identifier
 * @param total_left  Total value
 * @param total_right  Total value
 * @param delta_left  Increment from last message
 * @param delta_right  Increment from last message
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robot_encoders_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t id,uint32_t total_left,uint32_t total_right,int32_t delta_left,int32_t delta_right)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOT_ENCODERS_LEN];
    _mav_put_uint32_t(buf, 0, total_left);
    _mav_put_uint32_t(buf, 4, total_right);
    _mav_put_int32_t(buf, 8, delta_left);
    _mav_put_int32_t(buf, 12, delta_right);
    _mav_put_uint8_t(buf, 16, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOT_ENCODERS_LEN);
#else
    mavlink_robot_encoders_t packet;
    packet.total_left = total_left;
    packet.total_right = total_right;
    packet.delta_left = delta_left;
    packet.delta_right = delta_right;
    packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROBOT_ENCODERS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROBOT_ENCODERS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROBOT_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_ENCODERS_LEN, MAVLINK_MSG_ID_ROBOT_ENCODERS_CRC);
}

/**
 * @brief Encode a robot_encoders struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param robot_encoders C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_robot_encoders_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_robot_encoders_t* robot_encoders)
{
    return mavlink_msg_robot_encoders_pack(system_id, component_id, msg, robot_encoders->id, robot_encoders->total_left, robot_encoders->total_right, robot_encoders->delta_left, robot_encoders->delta_right);
}

/**
 * @brief Encode a robot_encoders struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param robot_encoders C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_robot_encoders_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_robot_encoders_t* robot_encoders)
{
    return mavlink_msg_robot_encoders_pack_chan(system_id, component_id, chan, msg, robot_encoders->id, robot_encoders->total_left, robot_encoders->total_right, robot_encoders->delta_left, robot_encoders->delta_right);
}

/**
 * @brief Send a robot_encoders message
 * @param chan MAVLink channel to send the message
 *
 * @param id  Parameter identifier
 * @param total_left  Total value
 * @param total_right  Total value
 * @param delta_left  Increment from last message
 * @param delta_right  Increment from last message
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_robot_encoders_send(mavlink_channel_t chan, uint8_t id, uint32_t total_left, uint32_t total_right, int32_t delta_left, int32_t delta_right)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOT_ENCODERS_LEN];
    _mav_put_uint32_t(buf, 0, total_left);
    _mav_put_uint32_t(buf, 4, total_right);
    _mav_put_int32_t(buf, 8, delta_left);
    _mav_put_int32_t(buf, 12, delta_right);
    _mav_put_uint8_t(buf, 16, id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_ENCODERS, buf, MAVLINK_MSG_ID_ROBOT_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_ENCODERS_LEN, MAVLINK_MSG_ID_ROBOT_ENCODERS_CRC);
#else
    mavlink_robot_encoders_t packet;
    packet.total_left = total_left;
    packet.total_right = total_right;
    packet.delta_left = delta_left;
    packet.delta_right = delta_right;
    packet.id = id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_ENCODERS, (const char *)&packet, MAVLINK_MSG_ID_ROBOT_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_ENCODERS_LEN, MAVLINK_MSG_ID_ROBOT_ENCODERS_CRC);
#endif
}

/**
 * @brief Send a robot_encoders message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_robot_encoders_send_struct(mavlink_channel_t chan, const mavlink_robot_encoders_t* robot_encoders)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_robot_encoders_send(chan, robot_encoders->id, robot_encoders->total_left, robot_encoders->total_right, robot_encoders->delta_left, robot_encoders->delta_right);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_ENCODERS, (const char *)robot_encoders, MAVLINK_MSG_ID_ROBOT_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_ENCODERS_LEN, MAVLINK_MSG_ID_ROBOT_ENCODERS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ROBOT_ENCODERS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_robot_encoders_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t id, uint32_t total_left, uint32_t total_right, int32_t delta_left, int32_t delta_right)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, total_left);
    _mav_put_uint32_t(buf, 4, total_right);
    _mav_put_int32_t(buf, 8, delta_left);
    _mav_put_int32_t(buf, 12, delta_right);
    _mav_put_uint8_t(buf, 16, id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_ENCODERS, buf, MAVLINK_MSG_ID_ROBOT_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_ENCODERS_LEN, MAVLINK_MSG_ID_ROBOT_ENCODERS_CRC);
#else
    mavlink_robot_encoders_t *packet = (mavlink_robot_encoders_t *)msgbuf;
    packet->total_left = total_left;
    packet->total_right = total_right;
    packet->delta_left = delta_left;
    packet->delta_right = delta_right;
    packet->id = id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_ENCODERS, (const char *)packet, MAVLINK_MSG_ID_ROBOT_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_ENCODERS_LEN, MAVLINK_MSG_ID_ROBOT_ENCODERS_CRC);
#endif
}
#endif

#endif

// MESSAGE ROBOT_ENCODERS UNPACKING


/**
 * @brief Get field id from robot_encoders message
 *
 * @return  Parameter identifier
 */
static inline uint8_t mavlink_msg_robot_encoders_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field total_left from robot_encoders message
 *
 * @return  Total value
 */
static inline uint32_t mavlink_msg_robot_encoders_get_total_left(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field total_right from robot_encoders message
 *
 * @return  Total value
 */
static inline uint32_t mavlink_msg_robot_encoders_get_total_right(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field delta_left from robot_encoders message
 *
 * @return  Increment from last message
 */
static inline int32_t mavlink_msg_robot_encoders_get_delta_left(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field delta_right from robot_encoders message
 *
 * @return  Increment from last message
 */
static inline int32_t mavlink_msg_robot_encoders_get_delta_right(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Decode a robot_encoders message into a struct
 *
 * @param msg The message to decode
 * @param robot_encoders C-struct to decode the message contents into
 */
static inline void mavlink_msg_robot_encoders_decode(const mavlink_message_t* msg, mavlink_robot_encoders_t* robot_encoders)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    robot_encoders->total_left = mavlink_msg_robot_encoders_get_total_left(msg);
    robot_encoders->total_right = mavlink_msg_robot_encoders_get_total_right(msg);
    robot_encoders->delta_left = mavlink_msg_robot_encoders_get_delta_left(msg);
    robot_encoders->delta_right = mavlink_msg_robot_encoders_get_delta_right(msg);
    robot_encoders->id = mavlink_msg_robot_encoders_get_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROBOT_ENCODERS_LEN? msg->len : MAVLINK_MSG_ID_ROBOT_ENCODERS_LEN;
        memset(robot_encoders, 0, MAVLINK_MSG_ID_ROBOT_ENCODERS_LEN);
    memcpy(robot_encoders, _MAV_PAYLOAD(msg), len);
#endif
}
