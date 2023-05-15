/** @file
 *  @brief MAVLink comm protocol generated from yarrr.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_YARRR_H
#define MAVLINK_YARRR_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_YARRR.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#define MAVLINK_YARRR_XML_HASH 4319279412332935429

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{0, 57, 15, 15, 0, 0, 0}, {1, 17, 17, 17, 0, 0, 0}, {2, 91, 5, 5, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_YARRR

// ENUM DEFINITIONS



// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_robot_motors_pids.h"
#include "./mavlink_msg_robot_encoders.h"
#include "./mavlink_msg_robot_motors_speed.h"

// base include



#if MAVLINK_YARRR_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_ROBOT_MOTORS_PIDS, MAVLINK_MESSAGE_INFO_ROBOT_ENCODERS, MAVLINK_MESSAGE_INFO_ROBOT_MOTORS_SPEED}
# define MAVLINK_MESSAGE_NAMES {{ "ROBOT_ENCODERS", 1 }, { "ROBOT_MOTORS_PIDS", 0 }, { "ROBOT_MOTORS_SPEED", 2 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_YARRR_H
