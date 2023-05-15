/** @file
 *    @brief MAVLink comm protocol testsuite generated from yarrr.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef YARRR_TESTSUITE_H
#define YARRR_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_yarrr(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_yarrr(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_robot_motors_pids(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_robot_motors_pids_t packet_in = {
        17.0,45.0,73.0,17859,175
    };
    mavlink_robot_motors_pids_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.kp = packet_in.kp;
        packet1.ki = packet_in.ki;
        packet1.kd = packet_in.kd;
        packet1.sample_rate = packet_in.sample_rate;
        packet1.id = packet_in.id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_motors_pids_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_robot_motors_pids_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_motors_pids_pack(system_id, component_id, &msg , packet1.id , packet1.kp , packet1.ki , packet1.kd , packet1.sample_rate );
    mavlink_msg_robot_motors_pids_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_motors_pids_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.id , packet1.kp , packet1.ki , packet1.kd , packet1.sample_rate );
    mavlink_msg_robot_motors_pids_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_robot_motors_pids_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_motors_pids_send(MAVLINK_COMM_1 , packet1.id , packet1.kp , packet1.ki , packet1.kd , packet1.sample_rate );
    mavlink_msg_robot_motors_pids_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ROBOT_MOTORS_PIDS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ROBOT_MOTORS_PIDS) != NULL);
#endif
}

static void mavlink_test_robot_encoders(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ROBOT_ENCODERS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_robot_encoders_t packet_in = {
        963497464,963497672,963497880,963498088,53
    };
    mavlink_robot_encoders_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.total_left = packet_in.total_left;
        packet1.total_right = packet_in.total_right;
        packet1.delta_left = packet_in.delta_left;
        packet1.delta_right = packet_in.delta_right;
        packet1.id = packet_in.id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ROBOT_ENCODERS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ROBOT_ENCODERS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_encoders_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_robot_encoders_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_encoders_pack(system_id, component_id, &msg , packet1.id , packet1.total_left , packet1.total_right , packet1.delta_left , packet1.delta_right );
    mavlink_msg_robot_encoders_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_encoders_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.id , packet1.total_left , packet1.total_right , packet1.delta_left , packet1.delta_right );
    mavlink_msg_robot_encoders_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_robot_encoders_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_encoders_send(MAVLINK_COMM_1 , packet1.id , packet1.total_left , packet1.total_right , packet1.delta_left , packet1.delta_right );
    mavlink_msg_robot_encoders_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ROBOT_ENCODERS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ROBOT_ENCODERS) != NULL);
#endif
}

static void mavlink_test_robot_motors_speed(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_robot_motors_speed_t packet_in = {
        17235,17339,17
    };
    mavlink_robot_motors_speed_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.speed_left = packet_in.speed_left;
        packet1.speed_right = packet_in.speed_right;
        packet1.id = packet_in.id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_motors_speed_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_robot_motors_speed_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_motors_speed_pack(system_id, component_id, &msg , packet1.id , packet1.speed_left , packet1.speed_right );
    mavlink_msg_robot_motors_speed_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_motors_speed_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.id , packet1.speed_left , packet1.speed_right );
    mavlink_msg_robot_motors_speed_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_robot_motors_speed_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_motors_speed_send(MAVLINK_COMM_1 , packet1.id , packet1.speed_left , packet1.speed_right );
    mavlink_msg_robot_motors_speed_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ROBOT_MOTORS_SPEED") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ROBOT_MOTORS_SPEED) != NULL);
#endif
}

static void mavlink_test_yarrr(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_robot_motors_pids(system_id, component_id, last_msg);
    mavlink_test_robot_encoders(system_id, component_id, last_msg);
    mavlink_test_robot_motors_speed(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // YARRR_TESTSUITE_H
