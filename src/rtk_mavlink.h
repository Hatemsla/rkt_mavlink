#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "generated-lib/ardupilotmega/mavlink.h"
#include "generated-lib/mavlink_helpers.h"

#if _WIN32
#include <windows.h>
#else
#include <pthread.h>
#include <unistd.h>
#include <string.h>
#endif

#if _WIN32
#define FFI_PLUGIN_EXPORT __declspec(dllexport)
#else
#define FFI_PLUGIN_EXPORT
#endif

mavlink_message_t tx_msg;
mavlink_message_t rx_msg;
mavlink_status_t rx_status;
mavlink_heartbeat_t rx_heartbeat;
mavlink_sys_status_t rx_sys_status;
mavlink_gps_status_t rx_gps_status;
mavlink_attitude_t rx_attitude;
mavlink_global_position_int_t rx_global_position_int;

char* heartbeat_str_result;
char* sys_status_str_result;
char* gps_status_str_result;
char* attitude_str_result;
char* global_position_int_str_result;

uint8_t sysid_apm;                       // id дрона
uint8_t compid_apm;                      // id автопилота

FFI_PLUGIN_EXPORT void update_data(uint8_t new_byte);