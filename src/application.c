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

#include <zcbor_common.h>
#include <zcbor_encode.h>

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



#define IMEI_LEN 15
#define AT_RESP_LEN 64

static char device_id_str[IMEI_LEN + sizeof(CONFIG_NRF_CLOUD_CLIENT_ID_PREFIX)]; // "nrf-" + 15 digits + null terminator

#define MAX_BUFFER_SIZE 35  // Define a maximum buffer size based on available memory

typedef struct {
    char sensor_name[4];
    float value1;
	float value2;
	float value3;
	float value4;
    int64_t timestamp;
} sensor_data_t;

sensor_data_t sensor_buffer[MAX_BUFFER_SIZE];
size_t buffer_index = 0;

struct sockaddr_storage server = { 0 };
struct coap_client coap_client = { 0 };
int sock;

#define ENCODED_BUFFER_SIZE    1024
#define ZCBOR_ENCODER_STATE_COUNT 4
uint8_t cbor_buffer[ENCODED_BUFFER_SIZE];
size_t encoded_len = 0;

/**
 * @brief Encode an array of sensor_data_t into CBOR using zCBOR.
 *
 * @param sensor_data       Pointer to the sensor_data_t array
 * @param data_count        Number of valid elements in the array
 * @param cbor_out          Pointer to the buffer to store the resulting CBOR
 * @param cbor_out_size     Size of the cbor_out buffer in bytes
 * @param cbor_encoded_len  Output: how many bytes were written
 *
 * @return true if encoding succeeded, false otherwise
 */
bool encode_sensor_data_cbor(const sensor_data_t *sensor_data,
                             size_t data_count,
                             uint8_t *cbor_out,
                             size_t cbor_out_size,
                             size_t *cbor_encoded_len)
{
    /* Create an array of zcbor_state_t structures. */
    zcbor_state_t zstate[ZCBOR_ENCODER_STATE_COUNT];

    /* Initialize your zCBOR encoder.
     * NOTE: The signature here is:
     *   zcbor_new_state(state_array, n_states, payload, payload_len,
     *                   elem_count, flags, flags_bytes)
     * Make sure this order matches exactly your local zcbor library.
     */
    zcbor_new_state(zstate, ZCBOR_ENCODER_STATE_COUNT,
                    cbor_out, cbor_out_size,
                    0,        /* elem_count */
                    NULL,     /* flags pointer */
                    0);       /* flags_bytes */

    LOG_DBG("encode_sensor_data_cbor: data_count=%zu, out_size=%zu",
            data_count, cbor_out_size);

    bool ret = true;

    /* Encode the outer array (list) with `data_count` elements. */
    ret &= zcbor_list_start_encode(zstate, data_count+1);
    if (!ret) {
        LOG_ERR("Failed to start outer list. Possibly out of buffer space?");
        goto done;
    }
	char name[4]="NAME";
	//Start with the device ID
	ret &= zcbor_list_start_encode(zstate, 2);
	if (!ret) { LOG_ERR("Failed list_start_encode(3) at insert name"); goto done; }

	ret &= zcbor_tstr_encode_ptr(zstate, name, sizeof(name));
	ret &= zcbor_tstr_encode_ptr(zstate, device_id_str, sizeof(device_id_str));
	
	ret &= zcbor_list_end_encode(zstate, 2);
	if (!ret) { LOG_ERR("Failed list_start_encode(3) at insert name"); goto done; }

    for (size_t i = 0; i < data_count && ret; i++) {
        LOG_DBG("Encoding item %zu: name='%.*s', value1=%f,value2=%f,value3=%f,value4=%f, timestamp=%lld",
                i,
                (int)sizeof(sensor_data[i].sensor_name),
                sensor_data[i].sensor_name,
                (double)sensor_data[i].value1,
				(double)sensor_data[i].value2,
				(double)sensor_data[i].value3,
				(double)sensor_data[i].value4,
                (long long)sensor_data[i].timestamp);

        /* Each struct is an array of 3 fields:
         *   [ sensor_name, value, timestamp ]
         */
        ret &= zcbor_list_start_encode(zstate, 6);
        if (!ret) { LOG_ERR("Failed list_start_encode(3) at idx=%zu", i); goto done; }

        /* 1) Encode sensor_name (3 chars). */
        ret &= zcbor_tstr_encode_ptr(zstate,
                                     sensor_data[i].sensor_name,
                                     sizeof(sensor_data[i].sensor_name));
        if (!ret) { LOG_ERR("Failed tstr_encode_ptr at idx=%zu", i); goto done; }

        /* 2) Encode float (32 bits). */
        ret &= zcbor_float32_encode(zstate, &sensor_data[i].value1);
        if (!ret) { LOG_ERR("Failed float32_encode at idx=%zu", i); goto done; }

		/* 2) Encode float (32 bits). */
        ret &= zcbor_float32_encode(zstate, &sensor_data[i].value2);
        if (!ret) { LOG_ERR("Failed float32_encode at idx=%zu", i); goto done; }

		/* 2) Encode float (32 bits). */
        ret &= zcbor_float32_encode(zstate, &sensor_data[i].value3);
        if (!ret) { LOG_ERR("Failed float32_encode at idx=%zu", i); goto done; }

		/* 2) Encode float (32 bits). */
        ret &= zcbor_float32_encode(zstate, &sensor_data[i].value4);
        if (!ret) { LOG_ERR("Failed float32_encode at idx=%zu", i); goto done; }

        /* 3) Encode timestamp (int64). Must pass pointer for `_encode()` variant. */
        ret &= zcbor_int64_encode(zstate, &sensor_data[i].timestamp);
        if (!ret) { LOG_ERR("Failed int64_encode at idx=%zu", i); goto done; }

        ret &= zcbor_list_end_encode(zstate, 6);
        if (!ret) { LOG_ERR("Failed list_end_encode(3) at idx=%zu", i); goto done; }
    }

    /* Close the outer array. */
    ret &= zcbor_list_end_encode(zstate, data_count+1);
    if (!ret) {
        LOG_ERR("Failed to close outer list of size=%zu", data_count);
        goto done;
    }

done:
    if (ret) {
        /* If all encoding succeeded, compute how many bytes we wrote.
         * This version of zcbor doesn't track `payload_bak`, so we do pointer math:
         */
        size_t used = (size_t)(zstate[0].payload - cbor_out);
        *cbor_encoded_len = used;

        LOG_DBG("encode_sensor_data_cbor: success, encoded_len=%zu", used);
    } else {
        *cbor_encoded_len = 0;
        LOG_ERR("encode_sensor_data_cbor: failed somewhere!");
    }

    return ret;
}


void buffer_sensor_data(const char *sensor, double value1,double value2,double value3,double value4) {
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
        sensor_buffer[buffer_index].value1 = value1;
		sensor_buffer[buffer_index].value2 = value2;
		sensor_buffer[buffer_index].value3 = value3;
		sensor_buffer[buffer_index].value4 = value4;
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

int send_device_message_localcoap(void)
{
    LOG_INF("Send bulk message LOCALCOAP");

    /* Encode CBOR data. */
    bool success = encode_sensor_data_cbor(sensor_buffer, buffer_index,
                                           cbor_buffer, sizeof(cbor_buffer),
                                           &encoded_len);
    if (!success) {
        LOG_ERR("Failed to encode sensor data");
        return -EINVAL;
    }

    /* Because your struct has "size_t len;" (not a pointer), just assign the value. */
    struct coap_client_request req = {
        .method       = COAP_METHOD_POST,
        .confirmable  = true,
        .fmt          = COAP_CONTENT_FORMAT_APP_CBOR,
        .payload      = cbor_buffer,
        .len          = encoded_len,  // <- Use the value, not &encoded_len
        .cb           = response_cb,
        .path         = CONFIG_COAP_SAMPLE_RESOURCE,
        /* .options     = ... if needed */
        /* .num_options = ... if needed */
        /* .user_data   = ... if needed */
    };

    LOG_INF("Encoded message length: %zu", encoded_len);

    /* Send the CoAP request. */
    int err = coap_client_req(&coap_client, sock,
                              (struct sockaddr *)&server, &req, NULL);
    if (err) {
        LOG_ERR("Failed to send POST: %d", err);
		err = server_resolve(&server);
		if (err) {
			LOG_ERR("Failed to resolve server name");
		}
		err = coap_client_req(&coap_client, sock,
			(struct sockaddr *)&server, &req, NULL);
        return err;
    }

    return 0;  // success
}

#if defined(CONFIG_JSON_DIY_COAP)
void send_buffered_data() {
    int ret;
    ret = send_device_message_localcoap();
	
    if (ret) {
        LOG_ERR("Failed to send bulk message: %d", ret);
    }
    // Clear the buffer
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

	buffer_sensor_data("LOAM", location_data->location.latitude,location_data->location.longitude,location_data->location.accuracy,location_data->method);

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
    snprintf(device_id_str, sizeof(device_id_str), "%s%s",CONFIG_NRF_CLOUD_CLIENT_ID_PREFIX, imei_buf);

    LOG_INF("DEVICEID: %s", device_id_str);
    return 0;
}

static double temp_sum = 0;
static double humidity_sum = 0;
static double pressure_sum = 0;
static double gas_sum = 0;
static int samples_collected = 0;

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
	int sendcounter = 0;
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
            //buffer_sensor_data("CNT", counter++);
        	
		}

		if (IS_ENABLED(CONFIG_TEMP_TRACKING)) {
			double temp = -1;
			double gas = -1;
			double humidity = -1;
			double pressure = -1;
			double voltage = -1;
			double current= -1;
			double tempCharger= -1;
			double chargestat= -1;

			if (get_all_data(&temp, &humidity, &pressure, &gas)== 0) {
				/* Accumulate the readings */
				temp_sum += temp;
				humidity_sum += humidity;
				pressure_sum += pressure;
				gas_sum += gas;
				samples_collected++;

				LOG_DBG("Temperature is %f degrees C", temp);
				LOG_DBG("Gas Resistance is %d OHM", (int)gas);
				LOG_DBG("pressure is %f pascal", pressure);
				LOG_DBG("Humidity is %f %%", humidity);
				LOG_DBG("Samples collected: %d", samples_collected);
				//Log that we took a reading, idx no
				/* Once we have enough samples, compute the average and buffer it */
				if (samples_collected >= CONFIG_NUM_SAMPLES_AVERAGED) {
					double avg_temp = temp_sum / samples_collected;
					double avg_humidity = humidity_sum / samples_collected;
					double avg_pressure = pressure_sum / samples_collected;
					double avg_gas = gas_sum / samples_collected;

					LOG_DBG("Averaged Temperature: %f degC", avg_temp);
					LOG_DBG("Averaged Humidity: %f %%",     avg_humidity);
					LOG_DBG("Averaged Pressure: %f Pa",     avg_pressure);
					LOG_DBG("Averaged Gas: %d OHM",         (int)avg_gas);

					/* Buffer the averaged data (instead of the single-sample data) */
					buffer_sensor_data("THPG", avg_temp, avg_humidity, avg_pressure, avg_gas);
					LOG_INF("Reading %d", sendcounter);
					/* Reset accumulators and counter */
					temp_sum = 0;
					humidity_sum = 0;
					pressure_sum = 0;
					gas_sum = 0;
					samples_collected = 0;
					sendcounter++;
				}
			}
			else{
				LOG_ERR("Failed to get sensor data");
			}
            
			if (sendcounter > CONFIG_NUM_SAMPLES_PER_SEND) {
				if(get_charger(&voltage,&current,&tempCharger,&chargestat) == 0){
					buffer_sensor_data("VCtS", voltage,current,temp,chargestat);
					LOG_INF("Voltage is %f V", voltage);
					LOG_INF("Current is %f A", current);
					LOG_INF("Temperature is %f degrees C", tempCharger);
					LOG_INF("Charging status is %d", chargestat);
				}
				else{
					LOG_ERR("Failed to get voltage data");
				}

				LOG_INF("Sending data %d datapoints", (int)buffer_index);
            	send_buffered_data();
				buffer_index = 0;
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
