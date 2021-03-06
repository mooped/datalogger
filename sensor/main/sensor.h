/*
 * Sensors for ESP datalogger
 *
 * Steve Barnett 2018-2019
 *
 */

#pragma once

// Initialise any attached sensors
void sensor_init(void);

// ESP32 internal temperature sensor
uint8_t sensor_internal_temperature(void);

// Si7007 temperature/humidity sensor
typedef struct
{
  float temp;
  float humidity;
} si7007_data_t;

si7007_data_t sensor_si7007_read(void);

// CCS811 CO2/VOC sensor
typedef struct
{
  uint16_t co2_ppm;
  uint16_t voc_ppb;
  uint8_t raw_data[8];
  uint16_t baseline;
  uint16_t flags;
} ccs811_data_t;

void sensor_ccs811_write_env_data(float temperature, float humidity);
ccs811_data_t sensor_ccs811_read(void);
uint16_t sensor_ccs811_read_baseline();
void sensor_ccs811_write_baseline(uint16_t baseline);

// Thermistor
int sensor_thermistor_read(void);

