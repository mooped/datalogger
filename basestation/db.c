#include "db.h"

#include <string.h>
#include <stdio.h>
#include <time.h>

#include <mongoc.h>
#include <bson.h>
#include <stdint.h>

#include "credentials.h"

static mongoc_client_t *client = NULL;
static mongoc_database_t *database = NULL;
static mongoc_collection_t *data_collection = NULL;
static mongoc_collection_t *idmapping_collection = NULL;

void db_connect(void)
{
  const char *uri_str = MONGO_URI;

  /*
  * Required to initialize libmongoc's internals
  */
  mongoc_init();

  /*
   * Create a new client instance
   */
  client = mongoc_client_new(uri_str);

  /*
   * Register the application name so we can track it in the profile logs
   * on the server. This can also be done from the URI (see other examples).
   */
  mongoc_client_set_appname(client, "datalogger-server");

  /*
   * Get a handle on the database "db_name" and collection "coll_name"
   */
  database = mongoc_client_get_database(client, "climate");
  data_collection = mongoc_client_get_collection(client, "climate", "v0_3");
  idmapping_collection = mongoc_client_get_collection(client, "climate", "v0_3_idmapping");
}

void db_disconnect(void)
{
  /*
   * Release our handles and clean up libmongoc
   */
  mongoc_collection_destroy(data_collection);
  mongoc_collection_destroy(idmapping_collection);
  mongoc_database_destroy(database);
  mongoc_client_destroy(client);
  mongoc_cleanup();
}

void db_submit_record(time_t time, espnow_sensor_data_t* data)
{
  bson_t* doc;
  bson_oid_t oid;
  bson_error_t error;
  bson_iter_t iter;
  bson_t* core;
  //bson_t* thermistor;
  bson_t* si7007;
  bson_t* ccs811;

  // Generate MAC address string
  char* mac_buffer[13];
  snprintf((char*)mac_buffer, 13, "%2.x%2.x%2.x%2.x%2.x%2.x",
    data->header.sender_mac[0],
    data->header.sender_mac[1],
    data->header.sender_mac[2],
    data->header.sender_mac[3],
    data->header.sender_mac[4],
    data->header.sender_mac[5]
  );

  // Build document for the internal temperature sensor
  core = bson_new();
  BSON_APPEND_INT32(core, "internal_temperature", data->internal_temperature);

  // Build document for the si7007
  si7007 = bson_new();
  BSON_APPEND_DOUBLE(si7007, "temperature", data->si7007_data.temp);
  BSON_APPEND_DOUBLE(si7007, "humidity", data->si7007_data.humidity);

  // Build document for the ccs811
  ccs811 = bson_new();
  BSON_APPEND_DOUBLE(ccs811, "co2_ppm", data->ccs811_data.co2_ppm);
  BSON_APPEND_DOUBLE(ccs811, "voc_ppb", data->ccs811_data.voc_ppb);
  BSON_APPEND_INT32(ccs811, "ccs811_baseline", data->ccs811_data.baseline);
  BSON_APPEND_INT32(ccs811, "ccs811_flags", data->ccs811_data.flags);

  // Build the root document
  doc = bson_new();
  
  bson_oid_init(&oid, NULL);
  BSON_APPEND_OID(doc, "_id", &oid);
  BSON_APPEND_INT64(doc, "timestamp", time);
  BSON_APPEND_UTF8(doc, "sensor_mac", (const char*)mac_buffer);
  BSON_APPEND_DOCUMENT(doc, "core", core);
  //BSON_APPEND_DOCUMENT(doc, "thermistor", thermistor);
  BSON_APPEND_DOCUMENT(doc, "si7007", si7007);
  if (data->ccs811_data.co2_ppm > 0 || data->ccs811_data.voc_ppb > 0)
  {
    BSON_APPEND_DOCUMENT(doc, "ccs811", ccs811);
  }

  // Submit it
  if (!mongoc_collection_insert(data_collection, MONGOC_INSERT_NONE, doc, NULL, &error))
  {
    fprintf(stderr, "%s\n", error.message);
  }

  bson_destroy(doc);
  bson_destroy(core);
  //bson_destroy(thermistor);
  bson_destroy(si7007);
  //bson_destroy(ccs811);
}

