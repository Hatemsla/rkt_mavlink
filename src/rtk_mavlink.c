#include "rtk_mavlink.h"

#define HZ_1 1000000
#define HZ_10 100000
#define NUM_OF_POINTS 4

bool already_received_heartbeat = false; // первое сообщение heartbeat принято
unsigned long t_last_heartbeat = 0;
uint8_t tx_msg_buffer[MAVLINK_MAX_PACKET_LEN];
int tx_msg_len = 0;


FFI_PLUGIN_EXPORT send_msg request_attitude(){
    send_msg send;

    if(already_received_heartbeat){
        mavlink_msg_command_long_pack_chan(sysid_apm, MAV_COMP_ID_ONBOARD_COMPUTER, MAVLINK_COMM_0, &tx_msg, sysid_apm, compid_apm,
                                             MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_ATTITUDE, HZ_1, 0, 0, 0, 0, 0);
        tx_msg_len = mavlink_msg_to_send_buffer(tx_msg_buffer, &tx_msg);
        
        send.tx_msg_len = tx_msg_len;
        memcpy(send.tx_msg_buffer, tx_msg_buffer, tx_msg_len);
    }

    return send;
}

FFI_PLUGIN_EXPORT send_msg request_sys_status(){
    send_msg send;

    if(already_received_heartbeat){
        mavlink_msg_command_long_pack_chan(sysid_apm, MAV_COMP_ID_ONBOARD_COMPUTER, MAVLINK_COMM_0, &tx_msg, sysid_apm, compid_apm,
                                             MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_SYS_STATUS, HZ_1, 0, 0, 0, 0, 0);
        tx_msg_len = mavlink_msg_to_send_buffer(tx_msg_buffer, &tx_msg);
        
        send.tx_msg_len = tx_msg_len;
        memcpy(send.tx_msg_buffer, tx_msg_buffer, tx_msg_len);
    }

    return send;
}

FFI_PLUGIN_EXPORT send_msg request_gps_status(){
    send_msg send;

    if(already_received_heartbeat){
        mavlink_msg_command_long_pack_chan(sysid_apm, MAV_COMP_ID_ONBOARD_COMPUTER, MAVLINK_COMM_0, &tx_msg, sysid_apm, compid_apm,
                                             MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_GPS_STATUS, HZ_1, 0, 0, 0, 0, 0);
        tx_msg_len = mavlink_msg_to_send_buffer(tx_msg_buffer, &tx_msg);
        
        send.tx_msg_len = tx_msg_len;
        memcpy(send.tx_msg_buffer, tx_msg_buffer, tx_msg_len);
    }

    return send;
}

FFI_PLUGIN_EXPORT send_msg request_global_position_int(){
    send_msg send;

    if(already_received_heartbeat){
        mavlink_msg_command_long_pack_chan(sysid_apm, MAV_COMP_ID_ONBOARD_COMPUTER, MAVLINK_COMM_0, &tx_msg, sysid_apm, compid_apm,
                                             MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, HZ_1, 0, 0, 0, 0, 0);
        tx_msg_len = mavlink_msg_to_send_buffer(tx_msg_buffer, &tx_msg);
        
        send.tx_msg_len = tx_msg_len;
        memcpy(send.tx_msg_buffer, tx_msg_buffer, tx_msg_len);
    }

    return send;
}

FFI_PLUGIN_EXPORT char* update_data(uint8_t new_byte){
    uint8_t r_byte = new_byte;

    if (mavlink_parse_char(MAVLINK_COMM_0, r_byte, &rx_msg, &rx_status)){
        switch (rx_msg.msgid){
            case MAVLINK_MSG_ID_HEARTBEAT:
            {
                const char *prefix = "autopilot: ";
                size_t prefix_length = strlen(prefix);

                mavlink_msg_heartbeat_decode(&rx_msg, &rx_heartbeat);

                size_t result_length = prefix_length + 1;
                if (rx_heartbeat.autopilot == 8) {
                    result_length += strlen("NO AUTOPILOT");
                } else {
                    result_length += strlen("HAVE AUTOPILOT");
                }

                heartbeat_str_result = (char *)malloc(result_length);

                strcpy(heartbeat_str_result, prefix);

                if (rx_heartbeat.type == 2)
                {
                    strcat(heartbeat_str_result, "NO AUTOPILOT");
                }
                else
                {
                    strcat(heartbeat_str_result, "HAVE AUTOPILOT");
                }

                if(!already_received_heartbeat){
                    already_received_heartbeat = true;
                    sysid_apm = rx_msg.sysid;
                    compid_apm = rx_msg.compid;
                }

                return heartbeat_str_result;
            }
            case MAVLINK_MSG_ID_SYS_STATUS:
            {
                const char *prefix = "voltage_battery: ";
                size_t prefix_length = strlen(prefix);

                mavlink_msg_sys_status_decode(&rx_msg, &rx_sys_status);

                size_t result_length = prefix_length + 10;

                sys_status_str_result = (char *)malloc(result_length);
                strcpy(sys_status_str_result, prefix);

                char voltage_str[10];
                sprintf(voltage_str, "%d", rx_sys_status.voltage_battery);
                strcat(sys_status_str_result, voltage_str);

                return sys_status_str_result;
            }
            case MAVLINK_MSG_ID_GPS_STATUS:
            {
                const char *prefix = "satellites_visible: ";
                size_t prefix_length = strlen(prefix);

                mavlink_msg_gps_status_decode(&rx_msg, &rx_gps_status);

                size_t result_length = prefix_length + 10;

                gps_status_str_result = (char *)malloc(result_length);
                strcpy(gps_status_str_result, prefix);

                char satellites_visible_str[10];
                sprintf(satellites_visible_str, "%d", rx_gps_status.satellites_visible);
                strcat(gps_status_str_result, satellites_visible_str);
                
                return gps_status_str_result;
            }
            case MAVLINK_MSG_ID_ATTITUDE:
            {
                const char *roll_prefix = "roll: ";
                const char *pitch_prefix = "pitch: ";
                const char *yaw_prefix = "yaw: ";
                const char *roll_speed_prefix = "roll_speed: ";
                const char *pitch_speed_prefix = "pitch_speed: ";
                const char *yaw_speed_prefix = "yaw_speed: ";

                size_t roll_prefix_length = strlen(roll_prefix);
                size_t pitch_prefix_length = strlen(pitch_prefix);
                size_t yaw_prefix_length = strlen(yaw_prefix);
                size_t roll_speed_prefix_length = strlen(roll_speed_prefix);
                size_t pitch_speed_prefix_length = strlen(pitch_speed_prefix);
                size_t yaw_speed_prefix_length = strlen(yaw_speed_prefix);

                mavlink_msg_attitude_decode(&rx_msg, &rx_attitude);

                size_t result_length = roll_prefix_length + 8 + pitch_prefix_length + 8 + yaw_prefix_length + 8 + roll_speed_prefix_length + 8 + pitch_speed_prefix_length + 8 + yaw_speed_prefix_length + 8;
                attitude_str_result = (char *)malloc(result_length);

                strcpy(attitude_str_result, roll_prefix);
                char roll_str[8];
                sprintf(roll_str, "%.2f ", rx_attitude.roll);
                strcat(attitude_str_result, roll_str);

                strcat(attitude_str_result, pitch_prefix);
                char pitch_str[8];
                sprintf(pitch_str, "%.2f ", rx_attitude.pitch);
                strcat(attitude_str_result, pitch_str);

                strcat(attitude_str_result, yaw_prefix);
                char yaw_str[8];
                sprintf(yaw_str, "%.2f ", rx_attitude.yaw);
                strcat(attitude_str_result, yaw_str);

                strcat(attitude_str_result, roll_speed_prefix);
                char roll_speed_str[8];
                sprintf(roll_speed_str, "%.2f ", rx_attitude.rollspeed);
                strcat(attitude_str_result, roll_speed_str);

                strcat(attitude_str_result, pitch_speed_prefix);
                char pitch_speed_str[8];
                sprintf(pitch_speed_str, "%.2f ", rx_attitude.pitchspeed);
                strcat(attitude_str_result, pitch_speed_str);

                strcat(attitude_str_result, yaw_speed_prefix);
                char yaw_speed_str[8];
                sprintf(yaw_speed_str, "%.2f ", rx_attitude.yawspeed);
                strcat(attitude_str_result, yaw_speed_str);

                return attitude_str_result;
            }
            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            {
                const char *lat_prefix = "lat: ";
                const char *lon_prefix = "lon: ";
                const char *alt_prefix = "alt: ";

                size_t lat_prefix_length = strlen(lat_prefix);
                size_t lon_prefix_length = strlen(lon_prefix);
                size_t alt_prefix_length = strlen(alt_prefix);

                mavlink_msg_global_position_int_decode(&rx_msg, &rx_global_position_int);

                size_t result_length = lat_prefix_length + 4 + lon_prefix_length + 4 + alt_prefix_length + 4;
                global_position_int_str_result = (char *)malloc(result_length);

                strcpy(global_position_int_str_result, lat_prefix);
                char lat_str[4];
                sprintf(lat_str, "%d ", rx_global_position_int.lat);
                strcat(global_position_int_str_result, lat_str);

                strcat(global_position_int_str_result, lon_prefix);
                char lon_str[4];
                sprintf(lon_str, "%d ", rx_global_position_int.lon);
                strcat(global_position_int_str_result, lon_str);

                strcat(global_position_int_str_result, alt_prefix);
                char alt_str[4];
                sprintf(alt_str, "%d ", rx_global_position_int.alt);
                strcat(global_position_int_str_result, alt_str);

                return global_position_int_str_result;
            }
            default: {
                return "";
            }
        }
    } else {
        return "";
    }
}

void free_data(){
    if(heartbeat_str_result != NULL)
        free(heartbeat_str_result);
    
    if(sys_status_str_result != NULL)
        free(sys_status_str_result);

    if(gps_status_str_result != NULL)
        free(gps_status_str_result);

    if(attitude_str_result != NULL)
        free(attitude_str_result);

    if(global_position_int_str_result != NULL)
        free(global_position_int_str_result);
}