/*
 * CoAP based data logger
 * 
 * Steve Barnett 2018
 *
 * Largely based on the lib-coap example from esp-idf
 * https://github.com/espressif/esp-idf/tree/master/examples/protocols/coap_client
 *
 * Currently still a bunch of example code mashed together!
 *
*/

#include <string.h>
#include <sys/socket.h>
#include <netdb.h>

#include "driver/i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_deep_sleep.h"

#include "nvs_flash.h"

#include "coap.h"

// Declare undocumented temperature logger API
// Not particularly useful (the calibration is very inconsistent between devices
// But I'm interested in seeing how closely it matches my temperature sensor
uint8_t temprature_sens_read();

// Nasty globals to store sensor readings
unsigned int temp_high = 0;
unsigned int temp_low = 0;

unsigned int humidity_high = 0;
unsigned int humidity_low = 0;

int co2_reading = -1;
int voc_reading = -1;
uint8_t raw_reading[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD

#define COAP_DEFAULT_TIME_SEC 5
#define COAP_DEFAULT_TIME_USEC 0

/* The examples use uri "coap://californium.eclipse.org" that
   you can set via 'make menuconfig'.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define COAP_DEFAULT_DEMO_URI "coap://californium.eclipse.org"
*/
#define COAP_DEFAULT_DEMO_URI CONFIG_TARGET_DOMAIN_URI

static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const static int CONNECTED_BIT = BIT0;

const static char *TAG = "CoAP_client";

static void message_handler(struct coap_context_t *ctx, const coap_endpoint_t *local_interface, const coap_address_t *remote,
              coap_pdu_t *sent, coap_pdu_t *received,
                const coap_tid_t id)
{
    unsigned char* data = NULL;
    size_t data_len;
    if (COAP_RESPONSE_CLASS(received->hdr->code) == 2) {
        if (coap_get_data(received, &data_len, &data)) {
            printf("Received: %s\n", data);
        }
    }
}

static void coap_example_task(void *p)
{
    struct hostent *hp;
    struct ip4_addr *ip4_addr;

    coap_context_t*   ctx = NULL;
    coap_address_t    dst_addr, src_addr;
    static coap_uri_t uri;
    fd_set            readfds;
    struct timeval    tv;
    int flags, result;
    coap_pdu_t*       request = NULL;
    const char*       server_uri = COAP_DEFAULT_DEMO_URI;
    //uint8_t     get_method = 1;
    uint8_t     put_method = 3;

    char buffer[256];

    while (1) {
        ESP_LOGI(TAG, "Trying to connect");
        /* Wait for the callback to set the CONNECTED_BIT in the
           event group.
        */
        xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                            false, true, portMAX_DELAY);
        ESP_LOGI(TAG, "Connected to AP");

        if (coap_split_uri((const uint8_t *)server_uri, strlen(server_uri), &uri) == -1) {
            ESP_LOGE(TAG, "CoAP server uri error");
            break;
        }

        hp = gethostbyname((const char *)uri.host.s);

        if (hp == NULL) {
            ESP_LOGE(TAG, "DNS lookup failed");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        /* Code to print the resolved IP.

           Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
        ip4_addr = (struct ip4_addr *)hp->h_addr;
        ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*ip4_addr));

        coap_address_init(&src_addr);
        src_addr.addr.sin.sin_family      = AF_INET;
        src_addr.addr.sin.sin_port        = htons(0);
        src_addr.addr.sin.sin_addr.s_addr = INADDR_ANY;

        ctx = coap_new_context(&src_addr);
        if (ctx) {
            coap_address_init(&dst_addr);
            dst_addr.addr.sin.sin_family      = AF_INET;
            dst_addr.addr.sin.sin_port        = htons(COAP_DEFAULT_PORT);
            dst_addr.addr.sin.sin_addr.s_addr = ip4_addr->addr;

            request            = coap_new_pdu();
            if (request){
                request->hdr->type = COAP_MESSAGE_CON;
                request->hdr->id   = coap_new_message_id(ctx);
                request->hdr->code = put_method;
                coap_add_option(request, COAP_OPTION_URI_PATH, 4, (const unsigned char*)"temp");
                coap_add_option(request, COAP_OPTION_URI_QUERY, sizeof((unsigned char*)CONFIG_NODE_NAME) + 1, (const unsigned char*)CONFIG_NODE_NAME);

                // Build temperature message
                sprintf(buffer, "{\"core_temp\":%d,\"temp_pwm\":%d,\"humidity_pwm\":%d, \"co2_ppm\":%d, \"voc_ppb\":%d, \"ccs811_raw\":[%d, %d, %d, %d, %d, %d, %d, %d]}", temprature_sens_read(), temp_high, humidity_high, co2_reading, voc_reading, raw_reading[0], raw_reading[1], raw_reading[2], raw_reading[3], raw_reading[4], raw_reading[5], raw_reading[6], raw_reading[7]);

                coap_add_data(request, strlen(buffer), (unsigned char*)buffer);

                coap_register_response_handler(ctx, message_handler);
                coap_send_confirmed(ctx, ctx->endpoint, &dst_addr, request);

                flags = fcntl(ctx->sockfd, F_GETFL, 0);
                fcntl(ctx->sockfd, F_SETFL, flags|O_NONBLOCK);

                tv.tv_usec = COAP_DEFAULT_TIME_USEC;
                tv.tv_sec = COAP_DEFAULT_TIME_SEC;

                for(;;) {
                    FD_ZERO(&readfds);
                    FD_CLR( ctx->sockfd, &readfds );
                    FD_SET( ctx->sockfd, &readfds );
                    result = select( FD_SETSIZE, &readfds, 0, 0, &tv );
                    if (result > 0) {
                        if (FD_ISSET( ctx->sockfd, &readfds ))
                            coap_read(ctx);
                    } else if (result < 0) {
                        break;
                    } else {
                        ESP_LOGE(TAG, "select timeout");
                        break;
                    }
                }
            }
            coap_free_context(ctx);
        }

        ESP_LOGE(TAG, "Sleep");
        esp_deep_sleep_enable_timer_wakeup(52 * 1000000);
        esp_deep_sleep_start();
        ESP_LOGE(TAG, "Wake");
    }

    vTaskDelete(NULL);
}

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void wifi_conn_init(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(wifi_event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

#define WRITE_BIT  I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT   I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */

#define CCS811_ADDR 0x5a

void i2c_init()
{
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = 19;
  conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
  conf.scl_io_num = 18;
  conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
  conf.master.clk_speed = 100000;
  i2c_param_config(1, &conf);
  i2c_driver_install(1, conf.mode, 0, 0, 0);
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
  i2c_init();

  uint8_t status = 0x00;
  err = i2c_read(1, 0x00, &status, 1);
  ESP_LOGI(TAG, "Initial Status: [%x]", status);
  if ((status & 0x90) == 0x90)
  {
    ESP_LOGI(TAG, "Already Initialised!");
    return 1;
  }

  uint8_t sw_reset[] = { 0x11, 0xe5, 0x72, 0x8a };
  err = i2c_write(1, 0xff, sw_reset, 4);
  if (err != ESP_OK)
  {
    ESP_LOGI(TAG, "Software Reset failed! [%x]", err);
    return 0;
  }

  vTaskDelay(1000.f / 10.f);

  err = i2c_poke(1, 0xf4);
  if (err != ESP_OK)
  {
    ESP_LOGI(TAG, "App Start failed! [%x]", err);
    return 0;
  }

  vTaskDelay(1000.f / 10.f);

  status = 0x00;
  err = i2c_read(1, 0x00, &status, 1);
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
  err = i2c_write(1, 0x01, &drive_mode, 1);
  if (err != ESP_OK)
  {
    ESP_LOGI(TAG, "Set Drive Mode failed! Error: [%x]", err);
    return 0;
  }

  return 1;
}

static void ccs811_sample(uint8_t* data/*[8]*/)
{
  esp_err_t err;

  // Wait for a sample
  uint8_t ready = 0;
  while (!ready)
  {
    uint8_t status = 0x00;
    err = i2c_read(1, 0x00, &status, 1);
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
  err = i2c_read(1, 0x02, data, 8);
  if (err != ESP_OK )
  {
    ESP_LOGI(TAG, "Read Alg Result failed! Result: [CO2: 0x%x 0x%x VOC: 0x%x 0x%x Status: 0x%x Error: 0x%x Raw: 0x%x 0x%x] Error: %x", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], err);
    return;
  }

  ESP_LOGI(TAG, "Read Alg Result: [CO2: 0x%x 0x%x VOC: 0x%x 0x%x Status: 0x%x Error: 0x%x Raw: 0x%x 0x%x]", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
}

void app_main(void)
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    wifi_conn_init();

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
    gpio_set_level(26, 1);
    gpio_set_level(27, 0);

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

    ESP_LOGI(TAG, "Sampled...");

    // Read CO2 and VOC data from CCS811 if present
    ESP_LOGI(TAG, "Attempting to read CCS811...");
    if (ccs811_init())
    {
      do
      {
        ccs811_sample(raw_reading);
        co2_reading = ((uint16_t)(raw_reading[0]) << 8) + raw_reading[1];
        voc_reading = ((uint16_t)(raw_reading[2]) << 8) + raw_reading[3];
        ESP_LOGI(TAG, "CO2 PPM: %i VOC PPB: %i.", co2_reading, voc_reading);
      }
      while (co2_reading <= 0 || voc_reading <= 0);
    }
    else
    {
      ESP_LOGI(TAG, "No CCS811 present.");
      co2_reading = -1;
      voc_reading = -1;
    }

    // Log data
    ESP_LOGI(TAG, "H:%d %d T:%d %d", humidity_high, humidity_low, temp_high, temp_low);

    // On with the show
    xTaskCreate(coap_example_task, "coap", 2048, NULL, 5, NULL);
}

