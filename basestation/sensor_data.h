/*
 * Pull in the data types we expect to receive from the sensors
 *
 * Steve Barnett October 2018
 *
 */

#pragma once

#include <stdint.h>

// TODO: Restructure these types so they can be included with less of a cludge
#define ESP_NOW_ETH_ALEN 6
typedef int esp_now_send_status_t;

#include "../sensor/main/espnow_types.h"

