/*
 * Sensor library for ESP datalogger
 * 
 * Steve Barnett 2018
 *
*/

#include <string.h>
#include <math.h>

#include "driver/i2c.h"

#include "freertos/FreeRTOS.h"

#include "esp_log.h"

#include "sensor.h"

// Tag for logging
const static char *TAG = "Sensor";

// Declare undocumented temperature logger API
// Calibration is very inconsistent between devices but could be interesting
// to see how it correlates with the other sensors and it gives an otherwise
// unpopulated board something to send
uint8_t temprature_sens_read(void);

uint8_t sensor_internal_temperature(void)
{
  return temprature_sens_read();
}

// Nasty globals to store sensor readings
static unsigned int temp_high = 0;
static unsigned int temp_low = 0;

static unsigned int humidity_high = 0;
static unsigned int humidity_low = 0;

static float real_temp = -1.f;
static float real_humidity = -1.f;

static int co2_reading = -1;
static int voc_reading = -1;
static uint8_t raw_reading[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

// Si7007 support code
static void read_temp_humidity(void)
{
    // Read temperature and humidity data from Si7007 connected to pins IO21 and IO23
    temp_high = 0;
    temp_low = 0;
    humidity_high = 0;
    humidity_low = 0;

    ESP_LOGI(TAG, "Sampling...");

    for (int i = 0; i < 1000000; ++i)
    {
      // Sample temperature
      if (gpio_get_level(21))
      {
        ++temp_high;
      }
      else
      {
        ++temp_low;
      }

      // Sample humidity
      if (gpio_get_level(23))
      {
        ++humidity_high;
      }
      else
      {
        ++humidity_low;
      }
    }

    // Log data
    ESP_LOGI(TAG, "Si7007 raw PWM H:%d %d T:%d %d", humidity_high, humidity_low, temp_high, temp_low);

    if (temp_high > 0 && temp_low > 0 && humidity_high > 0 && humidity_low > 0)
    {
      // If the temperature/humidity data looks good convert and store for compensation
      real_temp = -46.85f + 175.72f *((float)(temp_high) / 1000000.f);
      real_humidity = -6.f + 125.f *((float)(humidity_high) / 1000000.f);
    }
    else
    {
      real_temp = -1.f;
      real_humidity = -1.f;
    }

    ESP_LOGI(TAG, "Temperature: %f Humidity: %f", real_temp, real_humidity);
}

si7007_data_t sensor_si7007_read(void)
{
  si7007_data_t data;
  read_temp_humidity();
  data.temp = real_temp;
  data.humidity = real_humidity;
  return data;
}

// CCS811 support code
#define WRITE_BIT  I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT   I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */

#define CCS811_ADDR 0x5a

static void i2c_init(i2c_port_t i2c_num)
{
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = GPIO_NUM_19;//19;
  conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
  conf.scl_io_num = GPIO_NUM_18;//18;
  conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
  conf.master.clk_speed = 100000;
  i2c_param_config(i2c_num, &conf);
  i2c_driver_install(i2c_num, conf.mode, 0, 0, 0);
}

static esp_err_t i2c_poke(i2c_port_t i2c_num, uint8_t reg)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( CCS811_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_write(i2c_port_t i2c_num, uint8_t reg, uint8_t* data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( CCS811_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read(i2c_port_t i2c_num, uint8_t reg, uint8_t* data_rd, size_t size)
{
    esp_err_t poke_err = i2c_poke(i2c_num, reg);
    if (poke_err != ESP_OK)
    {
      return poke_err;
    }
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( CCS811_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static uint8_t ccs811_init()
{
  esp_err_t err;

  uint8_t status = 0x00;
  err = i2c_read(I2C_NUM_0, 0x00, &status, 1);
  ESP_LOGI(TAG, "Initial Status: [%x]", status);
  if ((status & 0x90) == 0x90)
  {
    ESP_LOGI(TAG, "Already Initialised!");
    return 1;
  }

  uint8_t sw_reset[] = { 0x11, 0xe5, 0x72, 0x8a };
  err = i2c_write(I2C_NUM_0, 0xff, sw_reset, 4);
  if (err != ESP_OK)
  {
    ESP_LOGI(TAG, "Software Reset failed! [%x]", err);
    return 0;
  }

  vTaskDelay(1000.f / 10.f);

  err = i2c_poke(I2C_NUM_0, 0xf4);
  if (err != ESP_OK)
  {
    ESP_LOGI(TAG, "App Start failed! [%x]", err);
    return 0;
  }

  vTaskDelay(1000.f / 10.f);

  status = 0x00;
  err = i2c_read(I2C_NUM_0, 0x00, &status, 1);
  if ((status & 0x90) == 0x90)
  {
    ESP_LOGI(TAG, "CCS811 Initialised. Status: [%x]", status);
  }
  else
  {
    ESP_LOGI(TAG, "Read Status failed! Status: [%x] Error: [%x]", status, err);
    return 0;
  }

  uint8_t drive_mode = (0x01) << 4;
  err = i2c_write(I2C_NUM_0, 0x01, &drive_mode, 1);
  if (err != ESP_OK)
  {
    ESP_LOGI(TAG, "Set Drive Mode failed! Error: [%x]", err);
    return 0;
  }

  return 1;
}

void ccs811_write_env_data(float temperature, float humidity)
{
  esp_err_t err;

  uint8_t env_data[4];
  uint16_t humidity_word = (uint16_t)(humidity * 512.f); // Humidity is 16 bits 1/512%RH
  uint16_t temperature_word = (uint16_t)((temperature + 25.f) * 512.f);  // Temperature is 16 bits 1/512 degrees C, but the scale starts at -25 degrees C

  // Write the environment data into a buffer and fix endianness
  // (ESP32 is little endian, the ccs811 expects big endian data)
  env_data[0] = (humidity_word >> 8) & 0xff;
  env_data[1] = humidity_word & 0xff;
  env_data[2] = (temperature_word >> 8) & 0xff;
  env_data[3] = temperature_word & 0xff;

  err = i2c_write(I2C_NUM_0, 0x05, env_data, 4);
  if (err == ESP_OK)
  {
    ESP_LOGI(TAG, "Write ENV_DATA: H: [%.4x] T: [%.4x] Packet: [%.2x%.2x%.2x%.2x]", humidity_word, temperature_word, env_data[0], env_data[1], env_data[2], env_data[3]);
  }
  else
  {
    ESP_LOGI(TAG, "Writing ENV_DATA failed. Error: [%x]", err);
    return;
  }
}

static void ccs811_sample(uint8_t* data/*[8]*/)
{
  esp_err_t err;

  // Wait for a sample
  uint8_t ready = 0;
  while (!ready)
  {
    uint8_t status = 0x00;
    err = i2c_read(I2C_NUM_0, 0x00, &status, 1);
    if ((status & 0x90) == 0x90)
    {
      if (status & 0x08)
      {
        ESP_LOGI(TAG, "Data Ready. Status: [%x]", status);
        ready = 1;
      }
      else
      {
        ESP_LOGI(TAG, "No data. Status: [%x]", status);
        vTaskDelay(1000.f / 10.f);
      }
    }
    else
    {
      ESP_LOGI(TAG, "Read Status failed! Status: [%x] Error: [%x]", status, err);
      return;
    }
  }

  // Read data
  err = i2c_read(I2C_NUM_0, 0x02, data, 8);
  if (err != ESP_OK )
  {
    ESP_LOGI(TAG, "Read Alg Result failed! Result: [CO2: 0x%x 0x%x VOC: 0x%x 0x%x Status: 0x%x Error: 0x%x Raw: 0x%x 0x%x] Error: %x", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], err);
    return;
  }

  ESP_LOGI(TAG, "Read Alg Result: [CO2: 0x%x 0x%x VOC: 0x%x 0x%x Status: 0x%x Error: 0x%x Raw: 0x%x 0x%x]", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
}

static void read_co2_vocs(void)
{
    // Read CO2 and VOC data from CCS811 if present
    ESP_LOGI(TAG, "Attempting to read CCS811...");
    if (ccs811_init())
    {
      // If the temperature/humidity data looks good use it for compensation
      if (real_temp > 0.f && real_humidity > 0.f)
      {
        ccs811_write_env_data(real_temp, real_humidity);
      }

      while (1)
      {
      ccs811_sample(raw_reading);
      co2_reading = ((uint16_t)(raw_reading[0]) << 8) + raw_reading[1];
      voc_reading = ((uint16_t)(raw_reading[2]) << 8) + raw_reading[3];
      ESP_LOGI(TAG, "CO2 PPM: %i VOC PPB: %i.", co2_reading, voc_reading);
      }
    }
    else
    {
      ESP_LOGI(TAG, "No CCS811 present.");
      co2_reading = -1;
      voc_reading = -1;
    }
}

ccs811_data_t sensor_ccs811_read(void)
{
  ccs811_data_t data;

  read_co2_vocs();

  data.co2_ppm = co2_reading;
  data.voc_ppb = voc_reading;
  memcpy(&data.raw_data, raw_reading, sizeof(data.raw_data));

  return data;
}

void sensor_init(void)
{
    // Configure input pins
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = ((1 << 21) | (1 << 23));
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // Configure output pins
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = ((1 << 25) | (1 << 27));
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // Set /wak low, and /rst high
    gpio_set_level(25, 1);
    gpio_set_level(27, 0);

    // Initialise the CCS811
    i2c_init(I2C_NUM_0);
    ccs811_init();
}
