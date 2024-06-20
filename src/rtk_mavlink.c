#include "rtk_mavlink.h"

#define HZ_1 1000000
#define HZ_10 100000
#define NUM_OF_POINTS 4

unsigned long t_last_heartbeat = 0;
uint8_t tx_msg_buffer[MAVLINK_MAX_PACKET_LEN];
int tx_msg_len = 0;
bool is_first_run = true;

FFI_PLUGIN_EXPORT send_msg request_mission_count(uint16_t mission_count)
{
    send_msg send;

    mavlink_msg_mission_count_pack_chan(sysid_apm, MAV_COMP_ID_MISSIONPLANNER, MAVLINK_COMM_1, &tx_msg, sysid_apm, compid_apm, mission_count, MAV_MISSION_TYPE_MISSION);
    tx_msg_len = mavlink_msg_to_send_buffer(tx_msg_buffer, &tx_msg);

    send.tx_msg_len = tx_msg_len;
    memcpy(send.tx_msg_buffer, tx_msg_buffer, tx_msg_len);

    return send;
}

FFI_PLUGIN_EXPORT send_msg request_mission_nav_waypoint(uint16_t seq, int32_t lat, int32_t lng, int32_t alt)
{
    send_msg send;

    mavlink_msg_mission_item_int_pack_chan(sysid_apm, MAV_COMP_ID_MISSIONPLANNER, MAVLINK_COMM_1, &tx_msg, sysid_apm, compid_apm, seq, MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_NAV_WAYPOINT,
                                           0, 1, 0, 0, 0, NAN, lat, lng, alt, MAV_MISSION_TYPE_MISSION);
    tx_msg_len = mavlink_msg_to_send_buffer(tx_msg_buffer, &tx_msg);

    send.tx_msg_len = tx_msg_len;
    memcpy(send.tx_msg_buffer, tx_msg_buffer, tx_msg_len);

    return send;
}

FFI_PLUGIN_EXPORT send_msg request_mission_nav_land(uint16_t seq, int32_t lat, int32_t lng, int32_t alt)
{
    send_msg send;

    mavlink_msg_mission_item_int_pack_chan(sysid_apm, MAV_COMP_ID_MISSIONPLANNER, MAVLINK_COMM_1, &tx_msg, sysid_apm, compid_apm, seq, MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_NAV_LAND,
                                           0, 1,
                                           0, PRECISION_LAND_MODE_DISABLED, 0, NAN, lat, lng, alt, MAV_MISSION_TYPE_MISSION);
    tx_msg_len = mavlink_msg_to_send_buffer(tx_msg_buffer, &tx_msg);

    send.tx_msg_len = tx_msg_len;
    memcpy(send.tx_msg_buffer, tx_msg_buffer, tx_msg_len);

    return send;
}

FFI_PLUGIN_EXPORT send_msg request_mission_nav_takeoff(uint16_t seq, int32_t lat, int32_t lng, int32_t alt)
{
    send_msg send;

    mavlink_msg_mission_item_int_pack_chan(sysid_apm, MAV_COMP_ID_MISSIONPLANNER, MAVLINK_COMM_1, &tx_msg, sysid_apm, compid_apm, seq, MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_NAV_TAKEOFF,
                                           0, 0, 0, 0, 0, NAN, lat, lng, alt, MAV_MISSION_TYPE_MISSION);
    tx_msg_len = mavlink_msg_to_send_buffer(tx_msg_buffer, &tx_msg);

    send.tx_msg_len = tx_msg_len;
    memcpy(send.tx_msg_buffer, tx_msg_buffer, tx_msg_len);

    return send;
}

FFI_PLUGIN_EXPORT send_msg request_mission_nav_return_to_launch(uint16_t seq)
{
    send_msg send;

    mavlink_msg_mission_item_int_pack_chan(sysid_apm, MAV_COMP_ID_MISSIONPLANNER, MAVLINK_COMM_1, &tx_msg, sysid_apm, compid_apm, seq, MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_NAV_RETURN_TO_LAUNCH,
                                           0, 0, 0, 0, 0, 0, 0, 0, 0, MAV_MISSION_TYPE_MISSION);
    tx_msg_len = mavlink_msg_to_send_buffer(tx_msg_buffer, &tx_msg);

    send.tx_msg_len = tx_msg_len;
    memcpy(send.tx_msg_buffer, tx_msg_buffer, tx_msg_len);

    return send;
}

FFI_PLUGIN_EXPORT send_msg request_mission_do_set_mode()
{
    send_msg send;

    mavlink_msg_command_long_pack_chan(sysid_apm, MAV_COMP_ID_MISSIONPLANNER, MAVLINK_COMM_1, &tx_msg, sysid_apm, compid_apm, MAV_CMD_DO_SET_MODE, 0, MAV_MODE_AUTO_ARMED, 0, 0, 0, 0, 0, 0);
    tx_msg_len = mavlink_msg_to_send_buffer(tx_msg_buffer, &tx_msg);

    send.tx_msg_len = tx_msg_len;
    memcpy(send.tx_msg_buffer, tx_msg_buffer, tx_msg_len);

    return send;
}

FFI_PLUGIN_EXPORT send_msg request_mission_start()
{
    send_msg send;

    mavlink_msg_command_long_pack_chan(sysid_apm, MAV_COMP_ID_MISSIONPLANNER, MAVLINK_COMM_1, &tx_msg, sysid_apm, compid_apm, MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0);
    tx_msg_len = mavlink_msg_to_send_buffer(tx_msg_buffer, &tx_msg);

    send.tx_msg_len = tx_msg_len;
    memcpy(send.tx_msg_buffer, tx_msg_buffer, tx_msg_len);

    return send;
}

FFI_PLUGIN_EXPORT send_msg request_cmd_arm_disarm(float arm)
{
    send_msg send;

    mavlink_msg_command_long_pack_chan(sysid_apm, MAV_COMP_ID_MISSIONPLANNER, MAVLINK_COMM_0, &tx_msg, sysid_apm, compid_apm, MAV_CMD_COMPONENT_ARM_DISARM, 0, arm, 0, 0, 0, 0, 0, 0);
    tx_msg_len = mavlink_msg_to_send_buffer(tx_msg_buffer, &tx_msg);

    send.tx_msg_len = tx_msg_len;
    memcpy(send.tx_msg_buffer, tx_msg_buffer, tx_msg_len);

    return send;
}

FFI_PLUGIN_EXPORT send_msg request_attitude()
{
    send_msg send;

    if (already_received_heartbeat)
    {
        mavlink_msg_command_long_pack_chan(sysid_apm, MAV_COMP_ID_ONBOARD_COMPUTER, MAVLINK_COMM_0, &tx_msg, sysid_apm, compid_apm,
                                           MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_ATTITUDE, HZ_1, 0, 0, 0, 0, 0);
        tx_msg_len = mavlink_msg_to_send_buffer(tx_msg_buffer, &tx_msg);

        send.tx_msg_len = tx_msg_len;
        memcpy(send.tx_msg_buffer, tx_msg_buffer, tx_msg_len);
    }

    return send;
}

FFI_PLUGIN_EXPORT send_msg request_sys_status()
{
    send_msg send;

    if (already_received_heartbeat)
    {
        mavlink_msg_command_long_pack_chan(sysid_apm, MAV_COMP_ID_ONBOARD_COMPUTER, MAVLINK_COMM_0, &tx_msg, sysid_apm, compid_apm,
                                           MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_SYS_STATUS, HZ_1, 0, 0, 0, 0, 0);
        tx_msg_len = mavlink_msg_to_send_buffer(tx_msg_buffer, &tx_msg);

        send.tx_msg_len = tx_msg_len;
        memcpy(send.tx_msg_buffer, tx_msg_buffer, tx_msg_len);
    }

    return send;
}

FFI_PLUGIN_EXPORT send_msg request_gps_status()
{
    send_msg send;

    if (already_received_heartbeat)
    {
        mavlink_msg_command_long_pack_chan(sysid_apm, MAV_COMP_ID_ONBOARD_COMPUTER, MAVLINK_COMM_0, &tx_msg, sysid_apm, compid_apm,
                                           MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_GPS_STATUS, HZ_1, 0, 0, 0, 0, 0);
        tx_msg_len = mavlink_msg_to_send_buffer(tx_msg_buffer, &tx_msg);

        send.tx_msg_len = tx_msg_len;
        memcpy(send.tx_msg_buffer, tx_msg_buffer, tx_msg_len);
    }

    return send;
}

FFI_PLUGIN_EXPORT send_msg request_global_position_int()
{
    send_msg send;

    if (already_received_heartbeat)
    {
        mavlink_msg_command_long_pack_chan(sysid_apm, MAV_COMP_ID_ONBOARD_COMPUTER, MAVLINK_COMM_0, &tx_msg, sysid_apm, compid_apm,
                                           MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, HZ_1, 0, 0, 0, 0, 0);
        tx_msg_len = mavlink_msg_to_send_buffer(tx_msg_buffer, &tx_msg);

        send.tx_msg_len = tx_msg_len;
        memcpy(send.tx_msg_buffer, tx_msg_buffer, tx_msg_len);
    }

    return send;
}

FFI_PLUGIN_EXPORT send_msg request_local_position_ned()
{
    send_msg send;

    if (already_received_heartbeat)
    {
        mavlink_msg_command_long_pack_chan(sysid_apm, MAV_COMP_ID_ONBOARD_COMPUTER, MAVLINK_COMM_0, &tx_msg, sysid_apm, compid_apm,
                                           MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_LOCAL_POSITION_NED, HZ_1, 0, 0, 0, 0, 0);
        tx_msg_len = mavlink_msg_to_send_buffer(tx_msg_buffer, &tx_msg);

        send.tx_msg_len = tx_msg_len;
        memcpy(send.tx_msg_buffer, tx_msg_buffer, tx_msg_len);
    }

    return send;
}

FFI_PLUGIN_EXPORT int update_data(uint8_t new_byte)
{
    if (is_first_run)
    {
        rx_mission_request_int.seq = -1;
        rx_mission_request.seq = -1;
        rx_mission_ack.type = -1;
        is_first_run = false;
    }

    uint8_t r_byte = new_byte;

    if (mavlink_parse_char(MAVLINK_COMM_0, r_byte, &rx_msg, &rx_status))
    {
        current_msg_id = rx_msg.msgid;

        switch (rx_msg.msgid)
        {
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            // is_heartbeat = 1;
            mavlink_msg_heartbeat_decode(&rx_msg, &rx_heartbeat);

            if (!already_received_heartbeat)
            {
                already_received_heartbeat = 1;
                sysid_apm = rx_msg.sysid;
                compid_apm = rx_msg.compid;
            }
            break;
        }
        case MAVLINK_MSG_ID_SYS_STATUS:
        {
            // is_sys_status = 1;
            mavlink_msg_sys_status_decode(&rx_msg, &rx_sys_status);
            break;
        }
        case MAVLINK_MSG_ID_GPS_STATUS:
        {
            // is_gps_status = 1;
            mavlink_msg_gps_status_decode(&rx_msg, &rx_gps_status);
            break;
        }
        case MAVLINK_MSG_ID_ATTITUDE:
        {
            // is_attitude = 1;
            mavlink_msg_attitude_decode(&rx_msg, &rx_attitude);
            break;
        }
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        {
            // is_global_position_int = 1;
            mavlink_msg_global_position_int_decode(&rx_msg, &rx_global_position_int);
            break;
        }
        case MAVLINK_MSG_ID_SYSTEM_TIME:
        {
            mavlink_msg_system_time_decode(&rx_msg, &rx_system_time);
            break;
        }
        case MAVLINK_MSG_ID_PING:
        {
            mavlink_msg_ping_decode(&rx_msg, &rx_ping);
            break;
        }
        case MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL:
        {
            mavlink_msg_change_operator_control_decode(&rx_msg, &rx_change_operator_control);
            break;
        }
        case MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK:
        {
            mavlink_msg_change_operator_control_ack_decode(&rx_msg, &rx_change_operator_control_ack);
            break;
        }
        case MAVLINK_MSG_ID_AUTH_KEY:
        {
            mavlink_msg_auth_key_decode(&rx_msg, &rx_auth_key);
            break;
        }
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        {
            mavlink_msg_param_request_read_decode(&rx_msg, &rx_param_request_read);
            break;
        }
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        {
            mavlink_msg_param_request_list_decode(&rx_msg, &rx_param_request_list);
            break;
        }
        case MAVLINK_MSG_ID_PARAM_VALUE:
        {
            mavlink_msg_param_value_decode(&rx_msg, &rx_param_value);
            break;
        }
        case MAVLINK_MSG_ID_PARAM_SET:
        {
            mavlink_msg_param_set_decode(&rx_msg, &rx_param_set);
            break;
        }
        case MAVLINK_MSG_ID_GPS_RAW_INT:
        {
            mavlink_msg_gps_raw_int_decode(&rx_msg, &rx_gps_raw_int);
            break;
        }
        case MAVLINK_MSG_ID_SCALED_IMU:
        {
            mavlink_msg_scaled_imu_decode(&rx_msg, &rx_scaled_imu);
            break;
        }
        case MAVLINK_MSG_ID_RAW_IMU:
        {
            mavlink_msg_raw_imu_decode(&rx_msg, &rx_raw_imu);
            break;
        }
        case MAVLINK_MSG_ID_RAW_PRESSURE:
        {
            mavlink_msg_raw_pressure_decode(&rx_msg, &rx_raw_pressure);
            break;
        }
        case MAVLINK_MSG_ID_SCALED_PRESSURE:
        {
            mavlink_msg_scaled_pressure_decode(&rx_msg, &rx_scaled_pressure);
            break;
        }
        case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
        {
            mavlink_msg_attitude_quaternion_decode(&rx_msg, &rx_attitude_quaternion);
            break;
        }
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        {
            mavlink_msg_local_position_ned_decode(&rx_msg, &rx_local_position_ned);
            break;
        }
        case MAVLINK_MSG_ID_RC_CHANNELS_SCALED:
        {
            mavlink_msg_rc_channels_scaled_decode(&rx_msg, &rx_rc_channels_scaled);
            break;
        }
        case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
        {
            mavlink_msg_rc_channels_raw_decode(&rx_msg, &rx_rc_channels_raw);
            break;
        }
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
        {
            mavlink_msg_servo_output_raw_decode(&rx_msg, &rx_servo_output_raw);
            break;
        }
        case MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST:
        {
            mavlink_msg_mission_request_partial_list_decode(&rx_msg, &rx_mission_request_partial_list);
            break;
        }
        case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
        {
            mavlink_msg_mission_write_partial_list_decode(&rx_msg, &rx_mission_write_partial_list);
            break;
        }
        case MAVLINK_MSG_ID_MISSION_CURRENT:
        {
            mavlink_msg_mission_current_decode(&rx_msg, &rx_mission_current);
            break;
        }
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
        {
            mavlink_msg_mission_request_list_decode(&rx_msg, &rx_mission_request_list);
            break;
        }
        case MAVLINK_MSG_ID_MISSION_COUNT:
        {
            mavlink_msg_mission_count_decode(&rx_msg, &rx_mission_count);
            break;
        }
        case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
        {
            mavlink_msg_mission_clear_all_decode(&rx_msg, &rx_mission_clear_all);
            break;
        }
        case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
        {
            mavlink_msg_mission_item_reached_decode(&rx_msg, &rx_mission_item_reached);
            break;
        }
        case MAVLINK_MSG_ID_MISSION_ACK:
        {
            mavlink_msg_mission_ack_decode(&rx_msg, &rx_mission_ack);
            is_mission_ack = 1;
            break;
        }
        case MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN:
        {
            mavlink_msg_set_gps_global_origin_decode(&rx_msg, &rx_set_gps_global_origin);
            break;
        }
        case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
        {
            mavlink_msg_gps_global_origin_decode(&rx_msg, &rx_gps_global_origin);
            break;
        }
        case MAVLINK_MSG_ID_PARAM_MAP_RC:
        {
            mavlink_msg_param_map_rc_decode(&rx_msg, &rx_param_map_rc);
            break;
        }
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
        {
            mavlink_msg_mission_request_int_decode(&rx_msg, &rx_mission_request_int);

            break;
        }
        case MAVLINK_MSG_ID_MISSION_REQUEST:
        {
            request_count++;
            current_seq = rx_msg.seq;
            is_mission_request = 1;
            mavlink_msg_mission_request_decode(&rx_msg, &rx_mission_request);
            return rx_msg.msgid;
        }
        case MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA:
        {
            mavlink_msg_safety_set_allowed_area_decode(&rx_msg, &rx_safety_set_allowed_area);
            break;
        }
        case MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA:
        {
            mavlink_msg_safety_allowed_area_decode(&rx_msg, &rx_safety_allowed_area);
            break;
        }
        case MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV:
        {
            mavlink_msg_attitude_quaternion_cov_decode(&rx_msg, &rx_attitude_quaternion_cov);
            break;
        }
        case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
        {
            mavlink_msg_nav_controller_output_decode(&rx_msg, &rx_nav_controller_output);
            break;
        }
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV:
        {
            mavlink_msg_global_position_int_cov_decode(&rx_msg, &rx_global_position_int_cov);
            break;
        }
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV:
        {
            mavlink_msg_local_position_ned_cov_decode(&rx_msg, &rx_local_position_ned_cov);
            break;
        }
        case MAVLINK_MSG_ID_RC_CHANNELS:
        {
            mavlink_msg_rc_channels_decode(&rx_msg, &rx_rc_channels);
            break;
        }
        case MAVLINK_MSG_ID_MANUAL_CONTROL:
        {
            mavlink_msg_manual_control_decode(&rx_msg, &rx_manual_control);
            break;
        }
        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
        {
            mavlink_msg_rc_channels_override_decode(&rx_msg, &rx_rc_channels_override);
            break;
        }
        case MAVLINK_MSG_ID_MISSION_ITEM_INT:
        {
            mavlink_msg_mission_item_int_decode(&rx_msg, &rx_mission_item_int);
            break;
        }
        case MAVLINK_MSG_ID_VFR_HUD:
        {
            mavlink_msg_vfr_hud_decode(&rx_msg, &rx_vfr_hud);
            break;
        }
        case MAVLINK_MSG_ID_COMMAND_INT:
        {
            mavlink_msg_command_int_decode(&rx_msg, &rx_command_int);
            break;
        }
        case MAVLINK_MSG_ID_COMMAND_LONG:
        {
            mavlink_msg_command_long_decode(&rx_msg, &rx_command_long);
            break;
        }
        case MAVLINK_MSG_ID_MANUAL_SETPOINT:
        {
            mavlink_msg_manual_setpoint_decode(&rx_msg, &rx_manual_setpoint);
            break;
        }
        case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
        {
            mavlink_msg_set_attitude_target_decode(&rx_msg, &rx_set_attitude_target);
            break;
        }
        case MAVLINK_MSG_ID_ATTITUDE_TARGET:
        {
            mavlink_msg_attitude_target_decode(&rx_msg, &rx_attitude_target);
            break;
        }
        case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
        {
            mavlink_msg_set_position_target_local_ned_decode(&rx_msg, &rx_set_position_target_local_ned);
            break;
        }
        case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
        {
            mavlink_msg_position_target_local_ned_decode(&rx_msg, &rx_position_target_local_ned);
            break;
        }
        case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
        {
            mavlink_msg_set_position_target_global_int_decode(&rx_msg, &rx_set_position_target_global_int);
            break;
        }
        case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
        {
            mavlink_msg_position_target_global_int_decode(&rx_msg, &rx_position_target_global_int);
            break;
        }
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET:
        {
            mavlink_msg_local_position_ned_system_global_offset_decode(&rx_msg, &rx_local_position_ned_system_global_offset);
            break;
        }
        case MAVLINK_MSG_ID_HIL_CONTROLS:
        {
            mavlink_msg_hil_controls_decode(&rx_msg, &rx_hil_controls);
            break;
        }
        case MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW:
        {
            mavlink_msg_hil_rc_inputs_raw_decode(&rx_msg, &rx_hil_rc_inputs_raw);
            break;
        }
        case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
        {
            mavlink_msg_hil_actuator_controls_decode(&rx_msg, &rx_hil_actuator_controls);
            break;
        }
        case MAVLINK_MSG_ID_OPTICAL_FLOW:
        {
            mavlink_msg_hil_optical_flow_decode(&rx_msg, &rx_hil_optical_flow);
            break;
        }
        case MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE:
        {
            mavlink_msg_global_vision_position_estimate_decode(&rx_msg, &rx_global_vision_position_estimate);
            break;
        }
        case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
        {
            mavlink_msg_vision_position_estimate_decode(&rx_msg, &rx_vision_position_estimate);
            break;
        }
        case MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE:
        {
            mavlink_msg_vision_speed_estimate_decode(&rx_msg, &rx_vision_speed_estimate);
            break;
        }
        case MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE:
        {
            mavlink_msg_vision_position_estimate_decode(&rx_msg, &rx_vision_position_estimate);
            break;
        }
        case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:
        {
            mavlink_msg_optical_flow_decode(&rx_msg, &rx_optical_flow);
            break;
        }
        case MAVLINK_MSG_ID_HIL_SENSOR:
        {
            mavlink_msg_hil_sensor_decode(&rx_msg, &rx_hil_sensor);
            break;
        }
        case MAVLINK_MSG_ID_SIM_STATE:
        {
            mavlink_msg_sim_state_decode(&rx_msg, &rx_sim_state);
            break;
        }
        case MAVLINK_MSG_ID_RADIO_STATUS:
        {
            mavlink_msg_radio_status_decode(&rx_msg, &rx_radio_status);
            break;
        }
        case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL:
        {
            mavlink_msg_file_transfer_protocol_decode(&rx_msg, &rx_file_transfer_protocol);
            break;
        }
        case MAVLINK_MSG_ID_TIMESYNC:
        {
            mavlink_msg_timesync_decode(&rx_msg, &rx_timesync);
            break;
        }
        case MAVLINK_MSG_ID_CAMERA_TRIGGER:
        {
            mavlink_msg_camera_trigger_decode(&rx_msg, &rx_camera_trigger);
            break;
        }
        case MAVLINK_MSG_ID_HIL_GPS:
        {
            mavlink_msg_hil_gps_decode(&rx_msg, &rx_hil_gps);
            break;
        }
        case MAVLINK_MSG_ID_HIL_OPTICAL_FLOW:
        {
            mavlink_msg_optical_flow_decode(&rx_msg, &rx_optical_flow);
            break;
        }
        case MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
        {
            mavlink_msg_hil_state_quaternion_decode(&rx_msg, &rx_hil_state_quaternion);
            break;
        }
        case MAVLINK_MSG_ID_SCALED_IMU2:
        {
            mavlink_msg_scaled_imu2_decode(&rx_msg, &rx_scaled_imu2);
            break;
        }
        case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
        {
            mavlink_msg_log_request_list_decode(&rx_msg, &rx_log_request_list);
            break;
        }
        case MAVLINK_MSG_ID_LOG_ENTRY:
        {
            mavlink_msg_log_entry_decode(&rx_msg, &rx_log_entry);
            break;
        }
        case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
        {
            mavlink_msg_log_request_data_decode(&rx_msg, &rx_log_request_data);
            break;
        }
        case MAVLINK_MSG_ID_LOG_DATA:
        {
            mavlink_msg_log_data_decode(&rx_msg, &rx_log_data);
            break;
        }
        case MAVLINK_MSG_ID_LOG_ERASE:
        {
            mavlink_msg_log_erase_decode(&rx_msg, &rx_log_erase);
            break;
        }
        case MAVLINK_MSG_ID_LOG_REQUEST_END:
        {
            mavlink_msg_log_request_end_decode(&rx_msg, &rx_log_request_end);
            break;
        }
        case MAVLINK_MSG_ID_GPS2_RAW:
        {
            mavlink_msg_gps2_raw_decode(&rx_msg, &rx_gps2_raw);
            break;
        }
        case MAVLINK_MSG_ID_POWER_STATUS:
        {
            mavlink_msg_power_status_decode(&rx_msg, &rx_power_status);
            break;
        }
        case MAVLINK_MSG_ID_SERIAL_CONTROL:
        {
            mavlink_msg_serial_control_decode(&rx_msg, &rx_serial_control);
            break;
        }
        case MAVLINK_MSG_ID_GPS_RTK:
        {
            mavlink_msg_gps_rtk_decode(&rx_msg, &rx_gps_rtk);
            break;
        }
        case MAVLINK_MSG_ID_GPS2_RTK:
        {
            mavlink_msg_gps2_rtk_decode(&rx_msg, &rx_gps2_rtk);
            break;
        }
        case MAVLINK_MSG_ID_SCALED_IMU3:
        {
            mavlink_msg_scaled_imu3_decode(&rx_msg, &rx_scaled_imu3);
            break;
        }
        case MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE:
        {
            mavlink_msg_data_transmission_handshake_decode(&rx_msg, &rx_data_transmission_handshake);
            break;
        }
        case MAVLINK_MSG_ID_ENCAPSULATED_DATA:
        {
            mavlink_msg_encapsulated_data_decode(&rx_msg, &rx_encapsulated_data);
            break;
        }
        case MAVLINK_MSG_ID_DISTANCE_SENSOR:
        {
            mavlink_msg_distance_sensor_decode(&rx_msg, &rx_distance_sensor);
            break;
        }
        case MAVLINK_MSG_ID_TERRAIN_REQUEST:
        {
            mavlink_msg_terrain_request_decode(&rx_msg, &rx_terrain_request);
            break;
        }
        case MAVLINK_MSG_ID_TERRAIN_DATA:
        {
            mavlink_msg_terrain_request_decode(&rx_msg, &rx_terrain_request);
            break;
        }
        case MAVLINK_MSG_ID_SCALED_PRESSURE2:
        {
            mavlink_msg_scaled_pressure2_decode(&rx_msg, &rx_scaled_pressure2);
            break;
        }
        case MAVLINK_MSG_ID_ATT_POS_MOCAP:
        {
            mavlink_msg_att_pos_mocap_decode(&rx_msg, &rx_att_pos_mocap);
            break;
        }
        case MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET:
        {
            mavlink_msg_set_actuator_control_target_decode(&rx_msg, &rx_set_actuator_control_target);
            break;
        }
        case MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET:
        {
            mavlink_msg_actuator_control_target_decode(&rx_msg, &rx_actuator_control_target);
            break;
        }
        case MAVLINK_MSG_ID_ALTITUDE:
        {
            mavlink_msg_altitude_decode(&rx_msg, &rx_altitude);
            break;
        }
        case MAVLINK_MSG_ID_RESOURCE_REQUEST:
        {
            mavlink_msg_resource_request_decode(&rx_msg, &rx_resource_request);
            break;
        }
        case MAVLINK_MSG_ID_SCALED_PRESSURE3:
        {
            mavlink_msg_scaled_pressure3_decode(&rx_msg, &rx_scaled_pressure3);
            break;
        }
        case MAVLINK_MSG_ID_FOLLOW_TARGET:
        {
            mavlink_msg_follow_target_decode(&rx_msg, &rx_follow_target);
            break;
        }
        case MAVLINK_MSG_ID_CONTROL_SYSTEM_STATE:
        {
            mavlink_msg_control_system_state_decode(&rx_msg, &rx_control_system_state);
            break;
        }
        case MAVLINK_MSG_ID_BATTERY_STATUS:
        {
            mavlink_msg_battery_status_decode(&rx_msg, &rx_battery_status);
            break;
        }
        case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
        {
            mavlink_msg_autopilot_version_decode(&rx_msg, &rx_autopilot_version);
            break;
        }
        case MAVLINK_MSG_ID_LANDING_TARGET:
        {
            mavlink_msg_landing_target_decode(&rx_msg, &rx_landing_target);
            break;
        }
        case MAVLINK_MSG_ID_FENCE_STATUS:
        {
            mavlink_msg_fence_status_decode(&rx_msg, &rx_fence_status);
            break;
        }
        case MAVLINK_MSG_ID_MAG_CAL_REPORT:
        {
            mavlink_msg_mag_cal_report_decode(&rx_msg, &rx_mag_cal_report);
            break;
        }
        case MAVLINK_MSG_ID_EFI_STATUS:
        {
            mavlink_msg_efi_status_decode(&rx_msg, &rx_efi_status);
            break;
        }
        case MAVLINK_MSG_ID_ESTIMATOR_STATUS:
        {
            mavlink_msg_estimator_status_decode(&rx_msg, &rx_estimator_status);
            break;
        }
        case MAVLINK_MSG_ID_WIND_COV:
        {
            mavlink_msg_wind_cov_decode(&rx_msg, &rx_wind_cov);
            break;
        }
        case MAVLINK_MSG_ID_GPS_INPUT:
        {
            mavlink_msg_gps_input_decode(&rx_msg, &rx_gps_input);
            break;
        }
        case MAVLINK_MSG_ID_GPS_RTCM_DATA:
        {
            mavlink_msg_gps_rtcm_data_decode(&rx_msg, &rx_gps_rtcm_data);
            break;
        }
        case MAVLINK_MSG_ID_HIGH_LATENCY2:
        {
            mavlink_msg_high_latency2_decode(&rx_msg, &rx_high_latency2);
            break;
        }
        case MAVLINK_MSG_ID_VIBRATION:
        {
            mavlink_msg_vibration_decode(&rx_msg, &rx_vibration);
            break;
        }
        case MAVLINK_MSG_ID_HOME_POSITION:
        {
            mavlink_msg_home_position_decode(&rx_msg, &rx_home_position);
            break;
        }
        case MAVLINK_MSG_ID_MESSAGE_INTERVAL:
        {
            mavlink_msg_message_interval_decode(&rx_msg, &rx_message_interval);
            break;
        }
        case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
        {
            mavlink_msg_extended_sys_state_decode(&rx_msg, &rx_extended_sys_state);
            break;
        }
        case MAVLINK_MSG_ID_ADSB_VEHICLE:
        {
            mavlink_msg_adsb_vehicle_decode(&rx_msg, &rx_adsb_vehicle);
            break;
        }
        case MAVLINK_MSG_ID_COLLISION:
        {
            mavlink_msg_collision_decode(&rx_msg, &rx_collision);
            break;
        }
        case MAVLINK_MSG_ID_V2_EXTENSION:
        {
            mavlink_msg_v2_extension_decode(&rx_msg, &rx_v2_extension);
            break;
        }
        case MAVLINK_MSG_ID_MEMORY_VECT:
        {
            mavlink_msg_memory_vect_decode(&rx_msg, &rx_memory_vect);
            break;
        }
        case MAVLINK_MSG_ID_DEBUG_VECT:
        {
            mavlink_msg_debug_vect_decode(&rx_msg, &rx_debug_vect);
            break;
        }
        case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
        {
            mavlink_msg_named_value_float_decode(&rx_msg, &rx_named_value_float);
            break;
        }
        case MAVLINK_MSG_ID_NAMED_VALUE_INT:
        {
            mavlink_msg_named_value_int_decode(&rx_msg, &rx_named_value_int);
            break;
        }
        case MAVLINK_MSG_ID_STATUSTEXT:
        {
            mavlink_msg_statustext_decode(&rx_msg, &rx_statustext);
            break;
        }
        case MAVLINK_MSG_ID_DEBUG:
        {
            mavlink_msg_debug_decode(&rx_msg, &rx_debug);
            break;
        }
        case MAVLINK_MSG_ID_SETUP_SIGNING:
        {
            mavlink_msg_setup_signing_decode(&rx_msg, &rx_setup_signing);
            break;
        }
        case MAVLINK_MSG_ID_BUTTON_CHANGE:
        {
            mavlink_msg_button_change_decode(&rx_msg, &rx_button_change);
            break;
        }
        case MAVLINK_MSG_ID_CAMERA_INFORMATION:
        {
            mavlink_msg_camera_information_decode(&rx_msg, &rx_camera_information);
            break;
        }
        case MAVLINK_MSG_ID_CAMERA_SETTINGS:
        {
            mavlink_msg_camera_settings_decode(&rx_msg, &rx_camera_settings);
            break;
        }
        case MAVLINK_MSG_ID_STORAGE_INFORMATION:
        {
            mavlink_msg_storage_information_decode(&rx_msg, &rx_storage_information);
            break;
        }
        case MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS:
        {
            mavlink_msg_camera_capture_status_decode(&rx_msg, &rx_camera_capture_status);
            break;
        }
        case MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED:
        {
            mavlink_msg_camera_image_captured_decode(&rx_msg, &rx_camera_image_captured);
            break;
        }
        case MAVLINK_MSG_ID_FLIGHT_INFORMATION:
        {
            mavlink_msg_flight_information_decode(&rx_msg, &rx_flight_information);
            break;
        }
        case MAVLINK_MSG_ID_LOGGING_DATA:
        {
            mavlink_msg_logging_data_decode(&rx_msg, &rx_logging_data);
            break;
        }
        case MAVLINK_MSG_ID_LOGGING_DATA_ACKED:
        {
            mavlink_msg_logging_data_acked_decode(&rx_msg, &rx_logging_data_acked);
            break;
        }
        case MAVLINK_MSG_ID_LOGGING_ACK:
        {
            mavlink_msg_logging_ack_decode(&rx_msg, &rx_logging_ack);
            break;
        }
        case MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION:
        {
            mavlink_msg_video_stream_information_decode(&rx_msg, &rx_video_stream_information);
            break;
        }
        case MAVLINK_MSG_ID_VIDEO_STREAM_STATUS:
        {
            mavlink_msg_video_stream_status_decode(&rx_msg, &rx_video_stream_status);
            break;
        }
        case MAVLINK_MSG_ID_CAMERA_FOV_STATUS:
        {
            mavlink_msg_camera_fov_status_decode(&rx_msg, &rx_camera_fov_status);
            break;
        }
        case MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS:
        {
            mavlink_msg_camera_tracking_image_status_decode(&rx_msg, &rx_camera_tracking_image_status);
            break;
        }
        case MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS:
        {
            mavlink_msg_camera_tracking_geo_status_decode(&rx_msg, &rx_camera_tracking_geo_status);
            break;
        }
        case MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION:
        {
            mavlink_msg_gimbal_manager_information_decode(&rx_msg, &rx_gimbal_manager_information);
            break;
        }
        case MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS:
        {
            mavlink_msg_gimbal_manager_status_decode(&rx_msg, &rx_gimbal_manager_status);
            break;
        }
        case MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE:
        {
            mavlink_msg_gimbal_manager_set_attitude_decode(&rx_msg, &rx_gimbal_manager_set_attitude);
            break;
        }
        case MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION:
        {
            mavlink_msg_gimbal_device_information_decode(&rx_msg, &rx_gimbal_device_information);
            break;
        }
        case MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE:
        {
            mavlink_msg_gimbal_device_set_attitude_decode(&rx_msg, &rx_gimbal_device_set_attitude);
            break;
        }
        case MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS:
        {
            mavlink_msg_gimbal_device_attitude_status_decode(&rx_msg, &rx_gimbal_device_attitude_status);
            break;
        }
        case MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE:
        {
            mavlink_msg_autopilot_state_for_gimbal_device_decode(&rx_msg, &rx_autopilot_state_for_gimbal_device);
            break;
        }
        case MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW:
        {
            mavlink_msg_gimbal_manager_set_pitchyaw_decode(&rx_msg, &rx_gimbal_manager_set_pitchyaw);
            break;
        }
        case MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_MANUAL_CONTROL:
        {
            mavlink_msg_gimbal_manager_set_manual_control_decode(&rx_msg, &rx_gimbal_manager_set_manual_control);
            break;
        }
        case MAVLINK_MSG_ID_WIFI_CONFIG_AP:
        {
            mavlink_msg_wifi_config_ap_decode(&rx_msg, &rx_wifi_config_ap);
            break;
        }
        case MAVLINK_MSG_ID_AIS_VESSEL:
        {
            mavlink_msg_ais_vessel_decode(&rx_msg, &rx_ais_vessel);
            break;
        }
        case MAVLINK_MSG_ID_UAVCAN_NODE_STATUS:
        {
            mavlink_msg_uavcan_node_status_decode(&rx_msg, &rx_uavcan_node_status);
            break;
        }
        case MAVLINK_MSG_ID_UAVCAN_NODE_INFO:
        {
            mavlink_msg_uavcan_node_info_decode(&rx_msg, &rx_uavcan_node_info);
            break;
        }
        case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ:
        {
            mavlink_msg_param_ext_request_read_decode(&rx_msg, &rx_param_ext_request_read);
            break;
        }
        case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST:
        {
            mavlink_msg_param_ext_request_list_decode(&rx_msg, &rx_param_ext_request_list);
            break;
        }
        case MAVLINK_MSG_ID_PARAM_EXT_VALUE:
        {
            mavlink_msg_param_ext_value_decode(&rx_msg, &rx_param_ext_value);
            break;
        }
        case MAVLINK_MSG_ID_PARAM_EXT_SET:
        {
            mavlink_msg_param_ext_set_decode(&rx_msg, &rx_param_ext_set);
            break;
        }
        case MAVLINK_MSG_ID_PARAM_EXT_ACK:
        {
            mavlink_msg_param_ext_ack_decode(&rx_msg, &rx_param_ext_ack);
            break;
        }
        case MAVLINK_MSG_ID_OBSTACLE_DISTANCE:
        {
            mavlink_msg_obstacle_distance_decode(&rx_msg, &rx_obstacle_distance);
            break;
        }
        case MAVLINK_MSG_ID_ODOMETRY:
        {
            mavlink_msg_odometry_decode(&rx_msg, &rx_odometry);
            break;
        }
        case MAVLINK_MSG_ID_ISBD_LINK_STATUS:
        {
            mavlink_msg_isbd_link_status_decode(&rx_msg, &rx_isbd_link_status);
            break;
        }
        case MAVLINK_MSG_ID_RAW_RPM:
        {
            mavlink_msg_raw_rpm_decode(&rx_msg, &rx_raw_rpm);
            break;
        }
        case MAVLINK_MSG_ID_UTM_GLOBAL_POSITION:
        {
            mavlink_msg_utm_global_position_decode(&rx_msg, &rx_utm_global_position);
            break;
        }
        case MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY:
        {
            mavlink_msg_debug_float_array_decode(&rx_msg, &rx_debug_float_array);
            break;
        }
        case MAVLINK_MSG_ID_GENERATOR_STATUS:
        {
            mavlink_msg_generator_status_decode(&rx_msg, &rx_generator_status);
            break;
        }
        case MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS:
        {
            mavlink_msg_actuator_output_status_decode(&rx_msg, &rx_actuator_output_status);
            break;
        }
        case MAVLINK_MSG_ID_TUNNEL:
        {
            mavlink_msg_tunnel_decode(&rx_msg, &rx_tunnel);
            break;
        }
        case MAVLINK_MSG_ID_CAN_FRAME:
        {
            mavlink_msg_can_frame_decode(&rx_msg, &rx_can_frame);
            break;
        }
        case MAVLINK_MSG_ID_CANFD_FRAME:
        {
            mavlink_msg_canfd_frame_decode(&rx_msg, &rx_canfd_frame);
            break;
        }
        case MAVLINK_MSG_ID_CAN_FILTER_MODIFY:
        {
            mavlink_msg_can_filter_modify_decode(&rx_msg, &rx_can_filter_modify);
            break;
        }
        case MAVLINK_MSG_ID_WHEEL_DISTANCE:
        {
            mavlink_msg_wheel_distance_decode(&rx_msg, &rx_wheel_distance);
            break;
        }
        case MAVLINK_MSG_ID_WINCH_STATUS:
        {
            mavlink_msg_winch_status_decode(&rx_msg, &rx_winch_status);
            break;
        }
        case MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID:
        {
            mavlink_msg_open_drone_id_basic_id_decode(&rx_msg, &rx_open_drone_id_basic_id);
            break;
        }
        case MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION:
        {
            mavlink_msg_open_drone_id_location_decode(&rx_msg, &rx_open_drone_id_location);
            break;
        }
        case MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION:
        {
            mavlink_msg_open_drone_id_authentication_decode(&rx_msg, &rx_open_drone_id_authentication);
            break;
        }
        case MAVLINK_MSG_ID_OPEN_DRONE_ID_SELF_ID:
        {
            mavlink_msg_open_drone_id_self_id_decode(&rx_msg, &rx_open_drone_id_self_id);
            break;
        }
        case MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM:
        {
            mavlink_msg_open_drone_id_system_decode(&rx_msg, &rx_open_drone_id_system);
            break;
        }
        case MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID:
        {
            mavlink_msg_open_drone_id_operator_id_decode(&rx_msg, &rx_open_drone_id_operator_id);
            break;
        }
        case MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK:
        {
            mavlink_msg_open_drone_id_message_pack_decode(&rx_msg, &rx_open_drone_id_message_pack);
            break;
        }
        case MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS:
        {
            mavlink_msg_open_drone_id_arm_status_decode(&rx_msg, &rx_open_drone_id_arm_status);
            break;
        }
        case MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE:
        {
            mavlink_msg_open_drone_id_system_update_decode(&rx_msg, &rx_open_drone_id_system_update);
            break;
        }
        case MAVLINK_MSG_ID_HYGROMETER_SENSOR:
        {
            mavlink_msg_hygrometer_sensor_decode(&rx_msg, &rx_hygrometer_sensor);
            break;
        }
        default:
        {
            // custom_seq++;
            return -1;
            break;
        }
        }
        return rx_msg.msgid;
    }
}