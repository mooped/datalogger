/*
 * Database interface for ESP datalogger
 *
 * Steve Barnett October 2018
 *
 */

#pragma once

#include <time.h>

#include "sensor_data.h"

void db_connect(void);
void db_disconnect(void);
void db_submit_record(time_t time, espnow_sensor_data_t* data);

