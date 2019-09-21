/*
 * Serial port handler
 *
 * Steve Barnett 2018
 *
 */

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

typedef struct
{
  int fd;
} serial_t;

serial_t* serial_init(const char* const device)
{
  serial_t* handle = (serial_t*)malloc(sizeof(serial_t));
  if (!handle)
  {
    perror("unable to allocate serial handle");
    return NULL;
  }

  handle->fd = open(device, O_RDWR);
  if (handle->fd < 0)
  {
    perror("error opening serial device");
    free(handle);
    return NULL;
  }

  // Set non-blocking mode on the serial device
  if (fcntl(handle->fd, F_SETFL, fcntl(handle->fd, F_GETFL) | O_NONBLOCK) < 0)
  {
    perror("error setting O_NONBLOCK on serial device");
    close(handle->fd);
    free(handle);
    return NULL;
  }

  return handle;
}

void serial_deinit(serial_t* handle)
{
  if (!handle)
  {
    return;
  }

  if (handle->fd >= 0)
  {
    close(handle->fd);
  }
  else
  {
    perror("expected a valid fd handle to close");
  }

  free(handle);
}

int serial_read_line(serial_t* handle, char* buffer, size_t buffer_size)
{
  if (!handle)
  {
    perror("expected a serial object to read from");
  }

  if (handle->fd < 0)
  {
    perror("invalid handle");
    return 0;
  }

  // Anything to read?
  int count = 0;
  char* out_ptr = buffer;
  ssize_t len = read(handle->fd, out_ptr, 1);
  // TODO: Keep a buffer of previous reads so we don't have to read single bytes to avoid overshooting
  if (len > 0)
  {
    // Keep reading until we find a newline
    while (len < buffer_size)
    {
      // We're done when we find a new line, but don't include it in the message length
      if (*out_ptr == '\n')
      {
        return count;
      }

      // Increment pointers, strip out carriage returns
      if (*out_ptr != '\r')
      {
        out_ptr += len;
        count += len;
      }

      // Read some more
      len = read(handle->fd, out_ptr, 1);
    }

    // Did we run out of buffer? Throw away the rest of the packet...
    if (len >= buffer_size)
    {
      out_ptr = buffer;
      while (*out_ptr != '\n')
      {
        read(handle->fd, out_ptr, 1);
      }
    }
  }

  return 0;
}

