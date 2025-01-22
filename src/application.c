/* Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <helpers/nrfx_reset_reason.h>
#include <date_time.h>
#include <stdio.h>
#include <net/nrf_cloud.h>
#include <net/nrf_cloud_codec.h>
#include <net/nrf_cloud_log.h>
#include <net/nrf_cloud_alert.h>
#if defined(CONFIG_NRF_CLOUD_COAP)
#include <net/nrf_cloud_coap.h>
#endif
#if defined(CONFIG_LOCATION_TRACKING)
#include <modem/location.h>
#include "location_tracking.h"
#endif
#include <dk_buttons_and_leds.h>
#include "application.h"
#include "temperature.h"
#include "cloud_connection.h"
#include "message_queue.h"
#include "led_control.h"
#include "at_commands.h"
#include "shadow_config.h"
#include <zephyr/drivers/sensor.h>

#include <zephyr/posix/arpa/inet.h>
#include <zephyr/posix/netdb.h>
#include <zephyr/posix/sys/socket.h>
#include <zephyr/posix/poll.h>
#include <nrf_modem_at.h>

#define IMEI_LEN 15

LOG_MODULE_REGISTER(application, CONFIG_MULTI_SERVICE_LOG_LEVEL);

/* Timer used to time the sensor sampling rate. */
static K_TIMER_DEFINE(sensor_sample_timer, NULL, NULL);

/* AT command request error handling */
#define AT_CMD_REQUEST_ERR_FORMAT "Error while processing AT command request: %d"
#define AT_CMD_REQUEST_ERR_MAX_LEN (sizeof(AT_CMD_REQUEST_ERR_FORMAT) + 20)
BUILD_ASSERT(CONFIG_AT_CMD_REQUEST_RESPONSE_BUFFER_LENGTH >= AT_CMD_REQUEST_ERR_MAX_LEN,
	     "Not enough AT command response buffer for printing error events.");

/* Temperature alert limits. */
#define TEMP_ALERT_LIMIT ((double)CONFIG_TEMP_ALERT_LIMIT)
#define TEMP_ALERT_HYSTERESIS 1.5
#define TEMP_ALERT_LOWER_LIMIT (TEMP_ALERT_LIMIT - TEMP_ALERT_HYSTERESIS)

#define MAX_BUFFER_SIZE 100  // Define a maximum buffer size based on available memory
#define NUM_SAMPLES 4

#define IMEI_LEN 15
#define AT_RESP_LEN 64

static char device_id_str[IMEI_LEN + 5]; // "nrf-" + 15 digits + null terminator

typedef struct {
    char sensor_name[32];
    double value;
    int64_t timestamp;
} sensor_data_t;

sensor_data_t sensor_buffer[MAX_BUFFER_SIZE];
size_t buffer_index = 0;


struct sockaddr_storage server = { 0 };
struct coap_client coap_client = { 0 };
int sock;

void buffer_sensor_data(const char *sensor, double value) {
    if (buffer_index < MAX_BUFFER_SIZE) {
        int err;
        int64_t timestamp;

        // Acquire timestamp using date_time_now()
        err = date_time_now(&timestamp);
        if (err) {
            LOG_ERR("Failed to obtain current time, error %d", err);
            timestamp = k_uptime_get();  // Fallback to uptime if necessary
        }

        // Store sensor data and timestamp in buffer
        strncpy(sensor_buffer[buffer_index].sensor_name, sensor, sizeof(sensor_buffer[buffer_index].sensor_name));
        sensor_buffer[buffer_index].value = value;
        sensor_buffer[buffer_index].timestamp = timestamp;
        buffer_index++;
    } else {
        LOG_ERR("Sensor buffer is full!");
    }
}

/* State of the test counter. This can be changed using the configuration in the shadow */
static bool test_counter_enabled;

/**
 * @brief Construct a device message object with automatically generated timestamp
 *
 * The resultant JSON object will be conformal to the General Message Schema described in the
 * application-protocols repo:
 *
 * https://github.com/nRFCloud/application-protocols
 *
 * @param msg - The object to contain the message
 * @param appid - The appId for the device message
 * @param msg_type - The messageType for the device message
 * @return int - 0 on success, negative error code otherwise.
 */
static int create_timestamped_device_message(struct nrf_cloud_obj *const msg,
					     const char *const appid,
					     const char *const msg_type)
{
	int err;
	int64_t timestamp;

	/* Acquire timestamp */
	err = date_time_now(&timestamp);
	if (err) {
		LOG_ERR("Failed to obtain current time, error %d", err);
		return -ETIME;
	}

	/* Create message object */
	err = nrf_cloud_obj_msg_init(msg, appid,
				     IS_ENABLED(CONFIG_NRF_CLOUD_COAP) ? NULL : msg_type);
	if (err) {
		LOG_ERR("Failed to initialize message with appid %s and msg type %s",
			appid, msg_type);
		return err;
	}

	/* Add timestamp to message object */
	err = nrf_cloud_obj_ts_add(msg, timestamp);
	if (err) {
		LOG_ERR("Failed to add timestamp to data message with appid %s and msg type %s",
			appid, msg_type);
		nrf_cloud_obj_free(msg);
		return err;
	}

	return 0;
}

int create_timestamped_device_message_with_ts(struct nrf_cloud_obj *const msg,
                                              const char *const appid,
                                              const char *const msg_type,
                                              int64_t timestamp) {
    int err;

    // Create message object
    err = nrf_cloud_obj_msg_init(msg, appid,
                                 IS_ENABLED(CONFIG_NRF_CLOUD_COAP) ? NULL : msg_type);
    if (err) {
        LOG_ERR("Failed to initialize message with appid %s and msg type %s",
                appid, msg_type);
        return err;
    }

    // Add timestamp to message object
    err = nrf_cloud_obj_ts_add(msg, timestamp);
    if (err) {
        LOG_ERR("Failed to add timestamp to data message with appid %s and msg type %s",
                appid, msg_type);
        nrf_cloud_obj_free(msg);
        return err;
    }

    return 0;
}

/**
 * @brief Transmit a collected sensor sample to nRF Cloud.
 *
 * @param sensor - The name of the sensor which was sampled.
 * @param value - The sampled sensor value.
 * @return int - 0 on success, negative error code otherwise.
 */
static int send_sensor_sample(const char *const sensor, double value)
{
	int ret;

	MSG_OBJ_DEFINE(msg_obj);

	/* Create a timestamped message container object for the sensor sample. */
	ret = create_timestamped_device_message(&msg_obj, sensor,
						NRF_CLOUD_JSON_MSG_TYPE_VAL_DATA);
	if (ret) {
		return -EINVAL;
	}

	/* Populate the container object with the sensor value. */
	LOG_INF("Add obj to single send: %f", value);
	ret = nrf_cloud_obj_num_add(&msg_obj, NRF_CLOUD_JSON_DATA_KEY, value, false);
	if (ret) {
		LOG_ERR("Failed to append value to %s sample container object ",
			sensor);
		nrf_cloud_obj_free(&msg_obj);
		return -ENOMEM;
	}

	/* Send the sensor sample container object as a device message. */
	return send_device_message(&msg_obj);
}

static void response_cb(int16_t code, size_t offset,
                        const uint8_t *payload, size_t len,
                        bool last_block, void *user_data)
{	
	LOG_INF("CoAP RESPONSE CALLBACK");
    if (code >= 0) {
        LOG_INF("CoAP response: code: 0x%x", code);

        /* Only log payload if non-empty. */
        if (len > 0 && payload != NULL) {
            /* Log it as a string with explicit length to avoid reading beyond the buffer. */
            LOG_INF("Payload (%u bytes): %.*s", (unsigned int)len, (int)len, payload);
        } else {
            LOG_INF("No payload in response");
        }
    } else {
        LOG_INF("Response received with error code: %d", code);
    }
}


static int server_resolve(struct sockaddr_storage *server)
{
	int err;
	struct addrinfo *result;
	struct addrinfo hints = {
		.ai_family = AF_INET,
		.ai_socktype = SOCK_DGRAM
	};
	char ipv4_addr[NET_IPV4_ADDR_LEN];

	err = getaddrinfo(CONFIG_COAP_SAMPLE_SERVER_HOSTNAME, NULL, &hints, &result);
	if (err) {
		LOG_ERR("getaddrinfo, error: %d", err);
		return err;
	}

	if (result == NULL) {
		LOG_ERR("Address not found");
		return -ENOENT;
	}

	/* IPv4 Address. */
	struct sockaddr_in *server4 = ((struct sockaddr_in *)server);

	server4->sin_addr.s_addr = ((struct sockaddr_in *)result->ai_addr)->sin_addr.s_addr;
	server4->sin_family = AF_INET;
	server4->sin_port = htons(CONFIG_COAP_SAMPLE_SERVER_PORT);

	inet_ntop(AF_INET, &server4->sin_addr.s_addr, ipv4_addr, sizeof(ipv4_addr));

	LOG_INF("IPv4 Address found %s", ipv4_addr);

	/* Free the address. */
	freeaddrinfo(result);

	return 0;
}

struct reading_item {
    const char *appId;
    double data;
    long long ts;
};

struct group {
    long long earliest_ts;
    cJSON *values; // JSON object holding { "appId": data, ... }
};

/* 
 * Convert and group the original JSON array into 
 * [ { "ts": X, "values": { "ID1": val1, "ID2": val2, ... } }, ... ]
 * where all readings within 1 second of the earliest reading get grouped.
 */
static cJSON *build_thingsboard_telemetry_array_one_second_group(const cJSON *original_array,
                                                                 const char *device_id)
{
    if (!cJSON_IsArray(original_array)) {
        LOG_ERR("Input is not a JSON array");
        return NULL;
    }

    int count = cJSON_GetArraySize(original_array);
    if (count <= 0) {
        LOG_WRN("Empty JSON array");
        // Return an empty top-level object with just the device_id
        cJSON *top_obj = cJSON_CreateObject();
        cJSON_AddStringToObject(top_obj, "device_id", device_id);
        cJSON_AddItemToObject(top_obj, "data", cJSON_CreateArray());
        return top_obj;
    }

    /* Allocate array for the raw readings */
    struct reading_item *items = k_calloc(count, sizeof(struct reading_item));
    if (!items) {
        LOG_ERR("Failed to allocate memory for reading items");
        return NULL;
    }

    /* Extract items from JSON into the array */
    int idx = 0;
    cJSON *elem = NULL;
    cJSON_ArrayForEach(elem, original_array) {
        if (idx >= count) {
            break;
        }

        cJSON *appId_obj = cJSON_GetObjectItem(elem, "appId");
        cJSON *ts_obj    = cJSON_GetObjectItem(elem, "ts");
        cJSON *data_obj  = cJSON_GetObjectItem(elem, "data");
        if (cJSON_IsString(appId_obj) && cJSON_IsNumber(ts_obj) && cJSON_IsNumber(data_obj)) {
            items[idx].appId = appId_obj->valuestring;
            items[idx].ts    = (long long)ts_obj->valuedouble;
            items[idx].data  = data_obj->valuedouble;
            idx++;
        }
    }
    int valid_count = idx;

    // Sort the items by timestamp ascending
    int cmp_func(const void *a, const void *b) {
        const struct reading_item *ra = (const struct reading_item *)a;
        const struct reading_item *rb = (const struct reading_item *)b;
        if (ra->ts < rb->ts) return -1;
        if (ra->ts > rb->ts) return  1;
        return 0;
    }
    qsort(items, valid_count, sizeof(struct reading_item), cmp_func);

    // Allocate groups (worst case: each reading is its own group)
    struct group *groups = k_calloc(valid_count, sizeof(struct group));
    if (!groups) {
        LOG_ERR("Failed to allocate memory for groups");
        k_free(items);
        return NULL;
    }

    int group_index = 0;
    if (valid_count > 0) {
        // Start first group with the first reading
        groups[0].earliest_ts = items[0].ts;
        groups[0].values = cJSON_CreateObject();
        cJSON_AddNumberToObject(groups[0].values, items[0].appId, items[0].data);
        group_index = 1;
    }

    // Group subsequent readings
    for (int i = 1; i < valid_count; i++) {
        long long current_ts = items[i].ts;
        struct group *last_grp = &groups[group_index - 1];

        if ((current_ts - last_grp->earliest_ts) < 1000) {
            // Same group
            cJSON_AddNumberToObject(last_grp->values, items[i].appId, items[i].data);
        } else {
            // New group
            groups[group_index].earliest_ts = current_ts;
            groups[group_index].values = cJSON_CreateObject();
            cJSON_AddNumberToObject(groups[group_index].values, items[i].appId, items[i].data);
            group_index++;
        }
    }

    // Build final JSON array of groups
    cJSON *tb_array = cJSON_CreateArray();
    if (!tb_array) {
        LOG_ERR("Failed to create TB array JSON");
        // Cleanup
        for (int g = 0; g < group_index; g++) {
            cJSON_Delete(groups[g].values);
        }
        k_free(groups);
        k_free(items);
        return NULL;
    }

    // Convert each group into { "ts": X, "values": {...} }
    for (int g = 0; g < group_index; g++) {
        cJSON *group_obj = cJSON_CreateObject();
        if (!group_obj) {
            LOG_ERR("Failed to create group object");
            cJSON_Delete(tb_array);
            // Cleanup
            for (int h = 0; h < group_index; h++) {
                cJSON_Delete(groups[h].values);
            }
            k_free(groups);
            k_free(items);
            return NULL;
        }

        cJSON_AddNumberToObject(group_obj, "ts", (double)groups[g].earliest_ts);
        cJSON_AddItemToObject(group_obj, "values", groups[g].values);
        cJSON_AddItemToArray(tb_array, group_obj);
    }

    // Cleanup intermediate data
    k_free(groups);
    k_free(items);

    // ===== Now wrap tb_array in a top-level object with the device ID =====
    cJSON *top_obj = cJSON_CreateObject();
    if (!top_obj) {
        LOG_ERR("Failed to create top-level JSON object");
        cJSON_Delete(tb_array);
        return NULL;
    }

    cJSON_AddStringToObject(top_obj, "device_id", device_id);
    cJSON_AddItemToObject(top_obj, "data", tb_array);

    return top_obj;
}


#define NRF_CLOUD_ENC_SRC_PRE_ENCODED 1

static int encode_nrf_cloud_obj(struct nrf_cloud_obj *obj)
{
    if (!obj || obj->type != NRF_CLOUD_OBJ_TYPE_JSON || !obj->json) {
        LOG_ERR("Invalid object or JSON");
        return -EINVAL;
    }

    /* Convert + group readings into TB-friendly array */
    cJSON *tb_array = build_thingsboard_telemetry_array_one_second_group(obj->json,device_id_str);
    if (!tb_array) {
        LOG_ERR("Failed to build TB telemetry array");
        return -ENOMEM;
    }

    /* Print the new array as unformatted JSON */
    char *json_str = cJSON_PrintUnformatted(tb_array);
    cJSON_Delete(tb_array);  // free the cJSON structure

    if (!json_str) {
        LOG_ERR("Failed to stringify TB array");
        return -ENOMEM;
    }

    /* Populate the encoded_data fields. Use k_malloc if you want Zephyr memory tracking. */
    obj->encoded_data.ptr = json_str;
    obj->encoded_data.len = strlen(json_str);
    obj->enc_src = NRF_CLOUD_ENC_SRC_PRE_ENCODED;

    LOG_INF("Encoded grouped TB JSON: %s", json_str);
    return 0;
}

int send_device_message_localcoap(struct nrf_cloud_obj *const msg_obj)
{
    if (!msg_obj) {
        return -EINVAL;
    }

    /* Ensure the object is encoded */
    if (!msg_obj->encoded_data.ptr || msg_obj->encoded_data.len == 0) {
        int err = encode_nrf_cloud_obj(msg_obj);
        if (err) {
            return err;
        }
    }

    struct coap_client_request req = {
        .method       = COAP_METHOD_POST,
        .confirmable  = true,
        .fmt          = COAP_CONTENT_FORMAT_APP_JSON, // sending JSON
        .payload      = (const uint8_t *)msg_obj->encoded_data.ptr,
        .len          = msg_obj->encoded_data.len,
        .cb           = response_cb, // your CoAP response callback
        .path         = CONFIG_COAP_SAMPLE_RESOURCE, // "api/v1/<token>/telemetry"
    };

    int err = coap_client_req(&coap_client, sock,
                              (struct sockaddr *)&server, &req, NULL);
    if (err) {
        LOG_ERR("Failed to send POST: %d", err);
        return err;
    }

    LOG_INF("CoAP POST sent. Payload: %s",
            (char *)msg_obj->encoded_data.ptr);

    return 0;
}

#if defined(CONFIG_JSON_DIY_COAP)
void send_buffered_data() {
    int ret;
    NRF_CLOUD_OBJ_JSON_DEFINE(bulk_obj);
    NRF_CLOUD_OBJ_JSON_DEFINE(msg_obj);
	LOG_INF("Initializing bulk message");
    ret = nrf_cloud_obj_bulk_init(&bulk_obj);
    if (ret) {
        LOG_ERR("Failed to initialize bulk message: %d", ret);
        return;
    }

    for (size_t i = 0; i < buffer_index; i++) {
        ret = create_timestamped_device_message_with_ts(&msg_obj,
                                                        sensor_buffer[i].sensor_name,
                                                        NRF_CLOUD_JSON_MSG_TYPE_VAL_DATA,
                                                        sensor_buffer[i].timestamp);
        if (ret) {
            LOG_ERR("Failed to create message object: %d", ret);
            continue;
        }

        ret = nrf_cloud_obj_num_add(&msg_obj, NRF_CLOUD_JSON_DATA_KEY, sensor_buffer[i].value, false);
		LOG_INF("Add obj to bulk send: %f", sensor_buffer[i].value);
        if (ret) {
            LOG_ERR("Failed to add data to message object: %d", ret);
            nrf_cloud_obj_free(&msg_obj);
            continue;
        }

        ret = nrf_cloud_obj_bulk_add(&bulk_obj, &msg_obj);
        if (ret) {
            LOG_ERR("Failed to add message to bulk object: %d", ret);
            nrf_cloud_obj_free(&msg_obj);
            continue;
        }
        nrf_cloud_obj_reset(&msg_obj);
    }
	LOG_INF("Send bulk message");
    ret = send_device_message_localcoap(&bulk_obj);
	
    if (ret) {
        LOG_ERR("Failed to send bulk message: %d", ret);
    }
    // Clear the buffer
    buffer_index = 0;
    

    nrf_cloud_obj_free(&bulk_obj);
}
#endif

#if defined(CONFIG_NRF_CLOUD_COAP) && !defined(CONFIG_JSON_DIY_COAP)
void send_buffered_data() {
    int ret;
    NRF_CLOUD_OBJ_JSON_DEFINE(msg_obj);

    LOG_INF("Sending buffered data as individual CoAP messages");

    for (size_t i = 0; i < buffer_index; i++) {
        // Initialize the message object
        ret = create_timestamped_device_message_with_ts(&msg_obj,
                                                        sensor_buffer[i].sensor_name,
                                                        NRF_CLOUD_JSON_MSG_TYPE_VAL_DATA,
                                                        sensor_buffer[i].timestamp);
        if (ret) {
            LOG_ERR("Failed to create message object: %d", ret);
            continue;
        }

        // Add the sensor value to the message
        ret = nrf_cloud_obj_num_add(&msg_obj, NRF_CLOUD_JSON_DATA_KEY, sensor_buffer[i].value, false);
        if (ret) {
            LOG_ERR("Failed to add data to message object: %d", ret);
            nrf_cloud_obj_free(&msg_obj);
            continue;
        }

        // Send the message
        LOG_INF("Sending message for sensor %s with value %f", sensor_buffer[i].sensor_name, sensor_buffer[i].value);
        ret = send_device_message(&msg_obj);
        if (ret) {
            LOG_ERR("Failed to send message: %d", ret);
            nrf_cloud_obj_free(&msg_obj);
            continue;
        }

        // Reset the message object for reuse
        nrf_cloud_obj_reset(&msg_obj);
    }

    // Clear the buffer after sending all messages
    LOG_INF("All buffered messages sent. Clearing buffer.");
    buffer_index = 0;
}
#endif

#if defined(CONFIG_NRF_CLOUD_MQTT)
void send_buffered_data() {
    int ret;
    NRF_CLOUD_OBJ_JSON_DEFINE(bulk_obj);
    NRF_CLOUD_OBJ_JSON_DEFINE(msg_obj);
	LOG_INF("Initializing bulk message");
    ret = nrf_cloud_obj_bulk_init(&bulk_obj);
    if (ret) {
        LOG_ERR("Failed to initialize bulk message: %d", ret);
        return;
    }

    for (size_t i = 0; i < buffer_index; i++) {
        ret = create_timestamped_device_message_with_ts(&msg_obj,
                                                        sensor_buffer[i].sensor_name,
                                                        NRF_CLOUD_JSON_MSG_TYPE_VAL_DATA,
                                                        sensor_buffer[i].timestamp);
        if (ret) {
            LOG_ERR("Failed to create message object: %d", ret);
            continue;
        }

        ret = nrf_cloud_obj_num_add(&msg_obj, NRF_CLOUD_JSON_DATA_KEY, sensor_buffer[i].value, false);
		LOG_INF("Add obj to bulk send: %f", sensor_buffer[i].value);
        if (ret) {
            LOG_ERR("Failed to add data to message object: %d", ret);
            nrf_cloud_obj_free(&msg_obj);
            continue;
        }

        ret = nrf_cloud_obj_bulk_add(&bulk_obj, &msg_obj);
        if (ret) {
            LOG_ERR("Failed to add message to bulk object: %d", ret);
            nrf_cloud_obj_free(&msg_obj);
            continue;
        }
        nrf_cloud_obj_reset(&msg_obj);
    }
	LOG_INF("Send bulk message");
    ret = send_device_message(&bulk_obj);
	
    if (ret) {
        LOG_ERR("Failed to send bulk message: %d", ret);
    } else {
        // Clear the buffer after successful send
        buffer_index = 0;
    }

    nrf_cloud_obj_free(&bulk_obj);
}
#endif

#if defined(CONFIG_LOCATION_TRACKING)
/**
 * @brief Transmit a collected GNSS sample to nRF Cloud.
 *
 * @param loc_gnss - GNSS location data.
 * @return int - 0 on success, negative error code otherwise.
 */
static int send_gnss(const struct location_event_data * const loc_gnss)
{
	int ret;

	if (!loc_gnss || (loc_gnss->method != LOCATION_METHOD_GNSS)) {
		return -EINVAL;
	}

	struct nrf_cloud_gnss_data gnss_pvt = {
		.type = NRF_CLOUD_GNSS_TYPE_PVT,
		.ts_ms = NRF_CLOUD_NO_TIMESTAMP,
		.pvt = {
			.lon		= loc_gnss->location.longitude,
			.lat		= loc_gnss->location.latitude,
			.accuracy	= loc_gnss->location.accuracy,
			.has_alt	= 0,
			.has_speed	= 0,
			.has_heading	= 0
		}
	};
	MSG_OBJ_DEFINE(msg_obj);

	/* Add the timestamp */
	(void)date_time_now(&gnss_pvt.ts_ms);

	/* Encode the location data into a device message */
	ret = nrf_cloud_obj_gnss_msg_create(&msg_obj, &gnss_pvt);

	if (ret == 0) {
		/* Send the location message */
		ret = send_device_message(&msg_obj);
	}

	return ret;
}

/**
 * @brief Callback to receive periodic location updates from location_tracking.c and forward them
 * to nRF Cloud.
 *
 * Note that cellular positioning (MCELL/Multi-Cell and SCELL/Single-Cell) is sent to nRF
 * Cloud automatically (since the Location library and nRF Cloud must work together to
 * determine them in the first place). GNSS positions, on the other hand, must be
 * sent manually, since they are determined entirely on-device.
 *
 * @param location_data - The received location update.
 *
 */
static void on_location_update(const struct location_event_data * const location_data)
{
	LOG_INF("Location Updated: %.06f N %.06f W, accuracy: %.01f m, Method: %s",
		location_data->location.latitude,
		location_data->location.longitude,
		(double)location_data->location.accuracy,
		location_method_str(location_data->method));

	buffer_sensor_data("LAT", location_data->location.latitude);
	buffer_sensor_data("LON", location_data->location.longitude);
	buffer_sensor_data("ACC", location_data->location.accuracy);
	buffer_sensor_data("MET", location_data->method);

	/* If the position update was derived using GNSS, send it onward to nRF Cloud. */
	if (location_data->method == LOCATION_METHOD_GNSS) {
		LOG_INF("GNSS Position Update! Sending to nRF Cloud...");
		send_gnss(location_data);
	}
}
#endif /* CONFIG_LOCATION_TRACKING */

/**
 * @brief Receives general device messages from nRF Cloud, checks if they are AT command requests,
 * and performs them if so, transmitting the modem response back to nRF Cloud.
 *
 * Try sending {"appId":"MODEM", "messageType":"CMD", "data":"AT+CGMR"}
 * in the nRF Cloud Portal Terminal card.
 *
 * @param msg - The device message to check.
 */
static void handle_at_cmd_requests(const struct nrf_cloud_data *const dev_msg)
{
	char *cmd;
	struct nrf_cloud_obj msg_obj;
	int err = nrf_cloud_obj_input_decode(&msg_obj, dev_msg);

	if (err) {
		/* The message isn't JSON or otherwise couldn't be parsed. */
		LOG_DBG("A general topic device message of length %d could not be parsed.",
			dev_msg->len);
		return;
	}

	/* Confirm app ID and message type */
	err = nrf_cloud_obj_msg_check(&msg_obj, NRF_CLOUD_JSON_APPID_VAL_MODEM,
				      NRF_CLOUD_JSON_MSG_TYPE_VAL_CMD);
	if (err) {
		goto cleanup;
	}

	/* Get the command string */
	err = nrf_cloud_obj_str_get(&msg_obj, NRF_CLOUD_JSON_DATA_KEY, &cmd);
	if (err) {
		/* Missing or invalid command value will be treated as a blank command */
		cmd = "";
	}

	/* Execute the command and receive the result */
	char *response = execute_at_cmd_request(cmd);

	/* To re-use msg_obj for the response message we must first free its memory and
	 * reset its state.
	 * The cmd string will no longer be valid after msg_obj is freed.
	 */
	cmd = NULL;
	/* Free the object's allocated memory */
	err = nrf_cloud_obj_free(&msg_obj);
	if (err) {
		LOG_ERR("Failed to free AT CMD request");
		return;
	}

	/* Reset the object's state */
	err = nrf_cloud_obj_reset(&msg_obj);
	if (err) {
		LOG_ERR("Failed to reset AT CMD request message object for reuse");
		return;
	}

	err = create_timestamped_device_message(&msg_obj, NRF_CLOUD_JSON_APPID_VAL_MODEM,
						NRF_CLOUD_JSON_MSG_TYPE_VAL_DATA);
	if (err) {
		return;
	}

	/* Populate response with command result */
	err = nrf_cloud_obj_str_add(&msg_obj, NRF_CLOUD_JSON_DATA_KEY, response, false);
	if (err) {
		LOG_ERR("Failed to populate AT CMD response with modem response data");
		goto cleanup;
	}

	/* Send the response */
	err = send_device_message(&msg_obj);
	if (err) {
		LOG_ERR("Failed to send AT CMD request response, error: %d", err);
	}

	return;

cleanup:
	(void)nrf_cloud_obj_free(&msg_obj);
}

/** @brief Check whether temperature is acceptable.
 * If the device exceeds a temperature limit, send the temperature alert one time.
 * Once the temperature falls below a lower limit, re-enable the temperature alert
 * so it will be sent if limit is exceeded again.
 *
 * The difference between the two limits should be sufficient to prevent sending
 * new alerts if the temperature value oscillates between two nearby values.
 *
 * @param temp - The current device temperature.
 */
static void monitor_temperature(double temp)
{
	static bool temperature_alert_active;

	if ((temp > TEMP_ALERT_LIMIT) && !temperature_alert_active) {
		temperature_alert_active = true;
		(void)nrf_cloud_alert_send(ALERT_TYPE_TEMPERATURE, (float)temp,
					   "Temperature over limit!");
		LOG_INF("Temperature limit %f C exceeded: now %f C.",
			TEMP_ALERT_LIMIT, temp);
	} else if ((temp < TEMP_ALERT_LOWER_LIMIT) && temperature_alert_active) {
		temperature_alert_active = false;
		LOG_INF("Temperature now below limit: %f C.", temp);
	}
}

/** @brief Send an incrementing test counter message to nRF Cloud.
 * If CONFIG_TEST_COUNTER_MULTIPLIER is greater than 1, the counter will be incremented
 * that many times in a row, and a separate test counter message will be sent for each increment.
 */
static void test_counter_send(void)
{
	static int counter;

	for (int i = 0; i < CONFIG_TEST_COUNTER_MULTIPLIER; i++) {
		LOG_INF("Sent test counter = %d", counter);
		(void)send_sensor_sample("COUNT", counter++);
	}
}

static void button_handler(uint32_t button_state, uint32_t has_changed)
{
	if (has_changed & DK_BTN1_MSK) {
		if ((button_state & DK_BTN1_MSK) == DK_BTN1_MSK) {
			LOG_INF("Button pressed");
			(void)nrf_cloud_alert_send(ALERT_TYPE_MSG, 0, "Button pressed");
		}
	}
}

static void print_reset_reason(void)
{
	uint32_t reset_reason;

	reset_reason = nrfx_reset_reason_get();
	LOG_INF("Reset reason: 0x%x", reset_reason);
}

static void report_startup(void)
{
	if (IS_ENABLED(CONFIG_SEND_ONLINE_ALERT)) {
		uint32_t reset_reason;

		reset_reason = nrfx_reset_reason_get();
		nrfx_reset_reason_clear(reset_reason);
		(void)nrf_cloud_alert_send(ALERT_TYPE_DEVICE_NOW_ONLINE, reset_reason, NULL);
	}
}

int fetch_device_id(void)
{
    char at_resp[AT_RESP_LEN] = {0};
    int err = nrf_modem_at_cmd(
        at_resp,
        sizeof(at_resp),
        "AT+CGSN=1"
    );

    if (err) {
        LOG_ERR("Failed to read IMEI (err %d)", err);
        return err;
    }

    /* Typical response might look like:
     *   +CGSN: "351901936599939"
     * So we need to find the quoted IMEI substring.
     */
    char *start = strchr(at_resp, '"');  // find the first quote
    if (!start) {
        LOG_ERR("Could not find starting quote in CGSN response: %s", at_resp);
        return -EINVAL;
    }
    start++; // move past the first quote

    // Copy up to 15 digits (IMEI is always 15 digits), or until next quote/end
    char imei_buf[IMEI_LEN + 1] = {0};
    memcpy(imei_buf, start, IMEI_LEN);
    imei_buf[IMEI_LEN] = '\0'; // null-terminate

    /* If you want a prefix like "nrf-", prepend it: */
    snprintf(device_id_str, sizeof(device_id_str), "nrf-%s", imei_buf);

    LOG_INF("DEVICEID: %s", device_id_str);
    return 0;
}

void main_application_thread_fn(void)
{
	print_reset_reason();

	if (IS_ENABLED(CONFIG_AT_CMD_REQUESTS)) {
		/* Register with connection.c to receive general device messages and check them for
		 * AT command requests.
		 */
		register_general_dev_msg_handler(handle_at_cmd_requests);
	}

	dk_buttons_init(button_handler);

	/* Wait for first connection before starting the application. */
	(void)await_cloud_ready(K_FOREVER);

	report_startup();
	(void)nrf_cloud_alert_send(ALERT_TYPE_DEVICE_NOW_ONLINE, 0, NULL);

	/* Wait for the date and time to become known.
	 * This is needed both for location services and for sensor sample timestamping.
	 */
	LOG_INF("Waiting for modem to determine current date and time");
	if (!await_date_time_known(K_SECONDS(CONFIG_DATE_TIME_ESTABLISHMENT_TIMEOUT_SECONDS))) {
		LOG_WRN("Failed to determine valid date time. Proceeding anyways");
	} else {
		LOG_INF("Current date and time determined");
	}

	const char *protocol = "";

	if (IS_ENABLED(CONFIG_NRF_CLOUD_MQTT)) {
		protocol = "MQTT";
	} else if (IS_ENABLED(CONFIG_NRF_CLOUD_COAP)) {
		protocol = "CoAP";
	}
	nrf_cloud_log_init();
	nrf_cloud_log_control_set(CONFIG_NRF_CLOUD_LOG_OUTPUT_LEVEL);
	/* Send a direct log to the nRF Cloud web portal indicating the sample has started up. */
	(void)nrf_cloud_log_send(LOG_LEVEL_INF,
				 "nRF Cloud multi-service sample has started, "
				 "version: %s, protocol: %s",
				 CONFIG_APP_VERSION, protocol);

#if defined(CONFIG_LOCATION_TRACKING)
	/* Begin tracking location at the configured interval. */
	(void)start_location_tracking(on_location_update,
					CONFIG_LOCATION_TRACKING_SAMPLE_INTERVAL_SECONDS);
#endif
	int err;
	/* Resolve hostname -> IP */
    err = server_resolve(&server);
    if (err) {
        LOG_ERR("Failed to resolve server name");
    }

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        LOG_ERR("Failed to create CoAP socket: %d.", -errno);
    }

    LOG_INF("Initializing CoAP client new code");
    err = coap_client_init(&coap_client, NULL);
    if (err) {
        LOG_ERR("Failed to initialize CoAP client: %d", err);
    }
    fetch_device_id();
	int counter = 0;
	int sendcounter = 3;
	/* Begin sampling sensors. */
	while (true) {
		/* Start the sensor sample interval timer.
		 * We use a timer here instead of merely sleeping the thread, because the
		 * application thread can be preempted by other threads performing long tasks
		 * (such as periodic location acquisition), and we want to account for these
		 * delays when metering the sample send rate.
		 */
		k_timer_start(&sensor_sample_timer,
			K_SECONDS(CONFIG_SENSOR_SAMPLE_INTERVAL_SECONDS), K_FOREVER);

		if (test_counter_enable_get()) {
			LOG_INF("Sent test counter = %d", counter);
            buffer_sensor_data("COUNT", counter++);
        	
		}

		if (IS_ENABLED(CONFIG_TEMP_TRACKING)) {
			double temp = -1;
			double gas = -1;
			double humidity = -1;
			double pressure = -1;
			double voltage = -1;

			if (get_all_data(&temp, &humidity, &pressure, &gas)== 0) {
				buffer_sensor_data("T", temp);
				monitor_temperature(temp);
				LOG_INF("Temperature is %f degrees C", temp);
				buffer_sensor_data("O", gas);
				LOG_INF("Gas Resistance is %d OHM", (int)gas);
				buffer_sensor_data("P", pressure);
				LOG_INF("pressure is %f pascal", pressure);
				buffer_sensor_data("H", humidity);
				LOG_INF("Humidity is %f %%", humidity);
			}
			else{
				LOG_ERR("Failed to get sensor data");
			}
            
			if (++sendcounter > NUM_SAMPLES) {
				if(get_voltage(&voltage) == 0){
					buffer_sensor_data("VBT", voltage);
					LOG_INF("Voltage is %f V", voltage);
				}
				else{
					LOG_ERR("Failed to get voltage data");
				}

				LOG_INF("Sending data %d datapoints", (int)buffer_index);
            	send_buffered_data();
				sendcounter = 0;
			}

		}
		/* Wait out any remaining time on the sample interval timer. */
		k_timer_status_sync(&sensor_sample_timer);

		/* If cloud stops being ready due to network trouble or device being deleted
		 * from the cloud, turn off sensors and wait, then restart sensors.
		 */
		if (!await_cloud_ready(K_NO_WAIT) && is_device_deleted()) {
#if defined(CONFIG_LOCATION_TRACKING)
			(void)location_request_cancel();
#endif
			LOG_INF("Cloud not ready. Pausing sensors.");
			(void)await_cloud_ready(K_FOREVER);
			LOG_INF("Cloud is ready. Enabling sensors.");
#if defined(CONFIG_LOCATION_TRACKING)
			/* Begin tracking location at the configured interval. */
			(void)start_location_tracking(on_location_update,
						  CONFIG_LOCATION_TRACKING_SAMPLE_INTERVAL_SECONDS);
#endif
		}
	}
}

void test_counter_enable_set(const bool enable)
{
	if (IS_ENABLED(CONFIG_TEST_COUNTER)) {
		LOG_DBG("CONFIG_TEST_COUNTER is enabled, ignoring state change request");
	} else {
		LOG_DBG("Test counter %s", enable ? "enabled" : "disabled");
		test_counter_enabled = enable;
	}
}

bool test_counter_enable_get(void)
{
	/* When CONFIG_TEST_COUNTER is enabled the test counter is always enabled */
	return (test_counter_enabled || IS_ENABLED(CONFIG_TEST_COUNTER));
}
