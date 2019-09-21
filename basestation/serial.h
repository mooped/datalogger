/*
 * Serial port handler
 *
 * Steve Barnett 2018
 *
 */

// Opaque handle type
typedef void serial_t;

serial_t* serial_init(const char* const device);
void serial_deinit(serial_t* handle);
int serial_read_line(serial_t* handle, char* buffer, size_t buffer_size);

