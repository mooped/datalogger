/*
 * ESP32 data logger main program
 *
 * Steve Barnett 2018-2019
*/

#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_event.h"
#include "tcpip_adapter.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_now.h"
#include "esp32/rom/ets_sys.h"
#include "esp32/rom/crc.h"

#include "espnow_types.h"
#include "led.h"
#include "sensor.h"

// IS_BASESTATION = 0 - read sensors, transmit data
// IS_BASESTATION = 1 - wait for packets, forward packets over UART to basestation software running on a host with network access
#define IS_BASESTATION 0

// SHOULD_SLEEP = 0 - continuously transmit data
// SHOULD_SLEEP = 1 - sleep between data transmissions
#define SHOULD_SLEEP 1

static const char *TAG = "datalogger";

static xQueueHandle espnow_queue;

static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static void espnow_deinit(espnow_state_t *state);

/* WiFi should start before using ESPNOW */
static void wifi_init(void)
{
  tcpip_adapter_init();
  ESP_ERROR_CHECK( esp_event_loop_create_default());
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
  ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
  ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
  ESP_ERROR_CHECK( esp_wifi_start());
  ESP_ERROR_CHECK( esp_wifi_set_promiscuous(1));


  /* In order to simplify example, channel is set after WiFi started.
   * This is not necessary in real application if the two devices have
   * been already on the same channel.
   */
  ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, 0) );
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  espnow_event_t evt;
  espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

  if (mac_addr == NULL)
  {
    ESP_LOGE(TAG, "Send cb arg error");
    return;
  }

  evt.id = ESPNOW_SEND_CB;
  memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
  send_cb->status = status;
  if (xQueueSend(espnow_queue, &evt, portMAX_DELAY) != pdTRUE)
  {
    ESP_LOGW(TAG, "Send send queue fail");
  }
}

static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
  espnow_event_t evt;
  espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

  if (mac_addr == NULL || data == NULL || len <= 0)
  {
    ESP_LOGE(TAG, "Receive cb arg error");
    return;
  }

  evt.id = ESPNOW_RECV_CB;
  memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
  recv_cb->data = malloc(len);
  if (recv_cb->data == NULL)
  {
    ESP_LOGE(TAG, "Malloc receive data fail");
    return;
  }
  memcpy(recv_cb->data, data, len);
  recv_cb->data_len = len;
  if (xQueueSend(espnow_queue, &evt, portMAX_DELAY) != pdTRUE)
  {
    ESP_LOGW(TAG, "Send receive queue fail");
    free(recv_cb->data);
  }
}

static char hex[] = "0123456789abcdef";

/* Dump packet to UART */
void data_dump(uint8_t* data, uint16_t data_len)
{
  uint16_t byte_index = 0;
  putchar('\r');
  putchar('\n');
  while (byte_index < data_len)
  {
    for (uint16_t line_char = 0; line_char < 16 && byte_index < data_len; ++line_char, ++byte_index)
    {
      putchar(hex[(data[byte_index] & 0xf0) >> 4]);
      putchar(hex[(data[byte_index] & 0x0f)     ]);
    }
  }
  putchar('\r');
  putchar('\n');
}

/* Parse received ESPNOW data. */
int espnow_data_parse(uint8_t *data, uint16_t data_len)
{
  espnow_data_t *buf = (espnow_data_t*)data;
  uint16_t crc, crc_cal = 0;

  // Check there is enough data to process the header
  if (data_len < sizeof(espnow_data_t))
  {
      ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
      return -1;
  }

  // Read packet size
  size_t packet_size = sizeof(espnow_data_t) + buf->payload_len;

  // Check CRC now we know the size of the packet
  crc = buf->crc;
  buf->crc = 0; // Zero CRC so we can recalculate correctly
  crc_cal = crc16_le(UINT16_MAX, (uint8_t const *)buf, packet_size);
  if (crc_cal == crc)
  {
      buf->crc = crc_cal; // Replace CRC for UART transmission

      // Invoke the appropriate handler for the packet
      switch (buf->type)
      {
        case PT_StationInfo:
        {
          assert(0 && "Station Info packets not yet implemented!");
        } break;
        case PT_Data:
        {
          // Just write out the data
          data_dump(data, packet_size);
        } break;
        default:
        {
          assert(0 && "Unknown packet type!");
        } break;
      }

      return 0;
  }

  return -1;
}

/* Prepare sensor data packet to be sent. */
espnow_packet_param_t espnow_build_sensor_data_packet(espnow_state_t* state)
{
  espnow_sensor_data_t *buf = (espnow_sensor_data_t *)state->buffer;

  // Create packet params
  espnow_packet_param_t packet_params;
  packet_params.len = sizeof(espnow_data_t) + sizeof(espnow_sensor_data_t);
  packet_params.buffer = (uint8_t*)buf;

  // LED indicates that we're waiting to transmit something
  led_set(1);

  // Fill buffer
  memset(buf, 0, packet_params.len);
  buf->header.type = PT_Data;
  buf->header.payload_len = sizeof(espnow_sensor_data_t) - sizeof(espnow_data_t);
  buf->internal_temperature = sensor_internal_temperature();
  buf->si7007_data = sensor_si7007_read();
  buf->ccs811_data = sensor_ccs811_read();
  ESP_LOGI(TAG, "Thermistor reading: %d", sensor_thermistor_read());

  assert(state->len >= sizeof(espnow_sensor_data_t));

  esp_read_mac(buf->header.sender_mac, ESP_MAC_WIFI_STA);
  buf->header.crc = 0;

  buf->header.crc = crc16_le(UINT16_MAX, (uint8_t const *)buf, sizeof(espnow_sensor_data_t));

  return packet_params;
}

esp_err_t espnow_send_sensor_data(espnow_state_t* state)
{
  espnow_packet_param_t packet = espnow_build_sensor_data_packet(state);

  return esp_now_send(state->dest_mac, packet.buffer, packet.len);
}

static void espnow_task(void *pvParameter)
{
  espnow_event_t evt;

  espnow_state_t *state = (espnow_state_t *)pvParameter;

#if !IS_BASESTATION
  /* If we're a sensor, start sending packets */
  ESP_LOGI(TAG, "Start sending broadcast data");
  if (espnow_send_sensor_data(state) != ESP_OK)
  {
      ESP_LOGE(TAG, "Send error");
      espnow_deinit(state);
      vTaskDelete(NULL);
  }
#else
  ESP_LOGI(TAG, "Waiting for data");
#endif // !IS_BASESTATION

  while (xQueueReceive(espnow_queue, &evt, portMAX_DELAY) == pdTRUE)
  {
    switch (evt.id)
    {
      case ESPNOW_SEND_CB:
      {
        espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

        // Last packet sent - turn off the LED
        led_set(0);

        /* Data is sent - sleep for 2 mins then wake up and send some more */
#if SHOULD_SLEEP
        ESP_LOGI(TAG, "Sleep");
        esp_sleep_enable_timer_wakeup(120 * 1000000);
        esp_deep_sleep_start();
        ESP_LOGI(TAG, "Wake");
        // Reboot
        esp_restart();
#else
        vTaskDelay(120000 / portTICK_PERIOD_MS);
#endif

        /* Send more data now the previous data is sent. */
        ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(send_cb->mac_addr));
        if (espnow_send_sensor_data(state) != ESP_OK)
        {
            ESP_LOGE(TAG, "Send error");
            espnow_deinit(state);
            vTaskDelete(NULL);
        }
        break;
      }
      case ESPNOW_RECV_CB:
      {
        espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

        if (espnow_data_parse(recv_cb->data, recv_cb->data_len) < 0)
        {
          ESP_LOGI(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
        }
        free(recv_cb->data);
        break;
      }
      default:
        ESP_LOGE(TAG, "Callback type error: %d", evt.id);
        break;
    }
  }
}

static esp_err_t espnow_init(void)
{
    espnow_state_t *state;

    espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (espnow_queue == NULL)
    {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(espnow_recv_cb) );

    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL)
    {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    /* Initialize sending parameters. */
    state = malloc(sizeof(espnow_state_t));
    memset(state, 0, sizeof(espnow_state_t));
    if (state == NULL)
    {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vSemaphoreDelete(espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    state->len = CONFIG_ESPNOW_SEND_LEN;
    state->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    if (state->buffer == NULL)
    {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(state);
        vSemaphoreDelete(espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }

    /* Start out by sending broadcast packets */
    memcpy(state->dest_mac, broadcast_mac, ESP_NOW_ETH_ALEN);

    /* Create the task to send and receive packets */
    xTaskCreate(espnow_task, "espnow_task", 4096, state, 4, NULL);

    return ESP_OK;
}

static void espnow_deinit(espnow_state_t *state)
{
    free(state->buffer);
    free(state);
    vSemaphoreDelete(espnow_queue);
    esp_now_deinit();
}

void app_main()
{
  // Initialise and light LED
  led_init();
  led_set(1);

  // Initialise sensors
  sensor_init();

  // Initialise ESPNOW
  wifi_init();
  espnow_init();
}

