/*
 * Base station for ESP datalogger
 *
 * Currently an ugly hack to get it working quickly
 *
 * Steve Barnett October 2018
 *
 */

#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>

#include "db.h"
#include "serial.h"
#include "sensor_data.h"

#ifndef min
#define min(a,b) ((a) < (b) ? (a) : (b))
#endif

#define BUFFER_SIZE 256

static int quit = 0;

#ifdef __GNUC__
#define UNUSED_PARAM __attribute__ ((unused))
#else /* not a GCC */
#define UNUSED_PARAM
#endif /* GCC */

/* SIGINT handler: set quit to 1 for graceful termination */
static void
handle_sigint(int signum UNUSED_PARAM)
{
  quit = 1;
}

char hex_decode_digit(char digit)
{
  // TODO: Check this generates something performant, or rewrite in less of a hurry
  switch (digit)
  {
    case '0': return 0x00;
    case '1': return 0x01;
    case '2': return 0x02;
    case '3': return 0x03;
    case '4': return 0x04;
    case '5': return 0x05;
    case '6': return 0x06;
    case '7': return 0x07;
    case '8': return 0x08;
    case '9': return 0x09;
    case 'a':
    case 'A': return 0x0a;
    case 'b':
    case 'B': return 0x0b;
    case 'c':
    case 'C': return 0x0c;
    case 'd':
    case 'D': return 0x0d;
    case 'e':
    case 'E': return 0x0e;
    case 'f':
    case 'F': return 0x0f;
    default: return -1;
  }
}

// Assumes input buffer is null terminated and output buffer is correctly sized
// Returns message length or -1 on failure to decode
int hex_decode(const char* input, char* output)
{
  const char* input_ptr = input;
  char* output_ptr = output;
  int size;

  // Assume a hex encoded packet and start decoding
  do
  {
    char high = hex_decode_digit(*input_ptr++);
    char low = hex_decode_digit(*input_ptr++);
    if (high == -1 || low == -1)
    {
      return -1;
    }
    *output_ptr++ = (high << 4) | low;
    ++size;
  }
  while (*input_ptr != '\0');

  return size;
}

// Process a message containing sensor data
void handle_sensor_data(espnow_sensor_data_t* data)
{
  // If payload is 4 bytes shorter than expected it's an old unit without CCS811 baseline and flags fields - zero these
  if (data->header.payload_len == ((sizeof(espnow_sensor_data_t) - 4) - sizeof(espnow_data_t)))
  {
    printf("Payload byte matches an old style packet. Zeroed CCS811 baseline and flags fields.\n");
    data->ccs811_data.baseline = 0;
    data->ccs811_data.flags = 0;
  }
  else
  // Check payload length
  if (data->header.payload_len != sizeof(espnow_sensor_data_t) - sizeof(espnow_data_t))
  {
    printf("Payload length did not match expected packet size.\n");
    return;
  }

  printf("Got sensor data:\n"
    "\tMAC: %2.x:%2.x:%2.x:%2.x:%2.x:%2.x\n"
    "\tCRC: %x\n"
    "\tCore Temp: %d\n"
    "\tsi7007 -\n"
    "\t\tTemperature: %f\n"
    "\t\tHumidity: %f\n"
    "\tccs811 -\n"
    "\t\tCO2 PPM: %d\n"
    "\t\tVOC PPB: %d\n"
    "\t\tBaseline: %4.4x\n"
    "\t\tFlags: %4.4x\n",
    data->header.sender_mac[0],
    data->header.sender_mac[1],
    data->header.sender_mac[2],
    data->header.sender_mac[3],
    data->header.sender_mac[4],
    data->header.sender_mac[5],
    data->header.crc,
    data->internal_temperature,
    data->si7007_data.temp,
    data->si7007_data.humidity,
    data->ccs811_data.co2_ppm,
    data->ccs811_data.co2_ppm,
    data->ccs811_data.baseline,
    data->ccs811_data.flags
  );

  // TODO: Check CRC

  // Get the time
  time_t time_now = time(NULL);

  // Submit to database
  db_submit_record(time_now, data);
}

// Figure out the type of a message and decide what to do with it
void route_message(espnow_data_t* message)
{
  switch (message->type)
  {
    case PT_Data:
    {
      handle_sensor_data((espnow_sensor_data_t*)message);
    } break;
    case PT_StationInfo:
    default:
    {
      printf("Ignoring unknown message\n");
    } break;
  }
}

static char decode_buffer[BUFFER_SIZE];
static const size_t decode_buffer_size = sizeof(decode_buffer) / sizeof(decode_buffer[0]);

// Reconstruct a message into memory and pass it on to the message router
void parse_message(const char* const message)
{
  const size_t message_len = strlen(message);

  // Message is an odd number of bytes - not hex encoded!
  if (message_len % 2 != 0)
  {
    printf("Odd sized Message: %s\n", message);
  }
  // Message is too long!
  if (message_len > decode_buffer_size * 2)
  {
    printf("Over sized Message: %s\n", message);
    return;
  }

  int size = hex_decode(message, decode_buffer);
  if (size < 0)
  {
    printf("Malformed Message: %s\n", message);
    return;
  }

  route_message((espnow_data_t*)decode_buffer);
}

char message_buffer[BUFFER_SIZE];
const size_t message_len = sizeof(message_buffer) / sizeof(message_buffer[0]);

int main(int argc, char **argv)
{
  db_connect();
  serial_t* serial = serial_init("/dev/ttyUSB0");
  if (!serial)
  {
    return -1;
  }

  signal(SIGINT, handle_sigint);

  // Read data from the serial port, submit valid packets to the database
  while (!quit)
  {
    int len = serial_read_line(serial, message_buffer, message_len - 1);
    if (len > 0)
    {
      message_buffer[len] = '\0';
      printf("Message[%d]: %s\n", len, message_buffer);

      parse_message(message_buffer);
    }

    usleep(10000);
  }

  serial_deinit(serial);
  db_disconnect();

  return 0;
}

