/* Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <zephyr/logging/log.h>
#include "application.h"
#include "cloud_connection.h"
#include "message_queue.h"
#include "led_control.h"
#include "fota_support_coap.h"
#include "shadow_support_coap.h"

LOG_MODULE_REGISTER(main, CONFIG_MULTI_SERVICE_LOG_LEVEL);

/* Here, we start the various threads that our application will run in */

#ifndef CONFIG_LED_INDICATION_DISABLED
/* Define, and automatically start the LED animation thread. See led_control.c */
K_THREAD_DEFINE(led_thread, CONFIG_LED_THREAD_STACK_SIZE, led_animation_thread_fn,
		NULL, NULL, NULL, 0, 0, 0);
#endif

/* Define, and automatically start the main application thread. See application.c */
K_THREAD_DEFINE(app_thread, CONFIG_APPLICATION_THREAD_STACK_SIZE, main_application_thread_fn,
		NULL, NULL, NULL, 0, 0, 0);

/* Define, and automatically start the message queue thread. See message_queue.c */
K_THREAD_DEFINE(msg_thread, CONFIG_MESSAGE_THREAD_STACK_SIZE, message_queue_thread_fn,
		NULL, NULL, NULL, 0, 0, 0);

/* Define, and automatically start the cloud connection thread. See cloud_connection.c */
K_THREAD_DEFINE(con_thread, CONFIG_CONNECTION_THREAD_STACK_SIZE, cloud_connection_thread_fn,
		NULL, NULL, NULL, 0, 0, 0);

#if defined(CONFIG_NRF_CLOUD_COAP)
#if defined(CONFIG_COAP_FOTA)
/* Define, and automatically start the CoAP FOTA check thread. See fota_support_coap.c */
K_THREAD_DEFINE(coap_fota, CONFIG_COAP_FOTA_THREAD_STACK_SIZE, coap_fota_thread_fn,
		NULL, NULL, NULL, 0, 0, 0);
#endif

#if defined(CONFIG_COAP_SHADOW)
/* Define, and automatically start the CoAP shadow check thread. See shadow_support_coap.c */
K_THREAD_DEFINE(coap_shadow, CONFIG_COAP_SHADOW_THREAD_STACK_SIZE, coap_shadow_thread_fn,
		NULL, NULL, NULL, 0, 0, 0);
#endif
#endif /* CONFIG_NRF_CLOUD_COAP */

/* main() is called from the main thread, which defaults to priority zero,
 * but for illustrative purposes we don't use it. main_application() could be called directly
 * from this function, rather than given its own dedicated thread.
 */
int main(void)
{	
#if defined(CONFIG_TEMP_DATA_USE_SENSOR)

	int32_t Voltage;

	if(device_is_ready(buck2) == false)
    {
		LOG_ERR("Regulator device is not ready!");
		return -ENODEV;
	}

	if (!device_is_ready(charger)) {
		printk("Charger device not ready.\n");
		return 0;
	}

	regulator_get_voltage(buck1, &Voltage);
	LOG_INF("BUCK1 Voltage: %i", Voltage);
	regulator_get_voltage(buck2, &Voltage);
	LOG_INF("BUCK2 Voltage: %i", Voltage);

	regulator_set_voltage(buck2, 1800000, 1800000);


	LOG_INF("V: %d.%03d ", volt.val1, volt.val2 / 1000);

	LOG_INF("I: %s%d.%04d ", ((current.val1 < 0) || (current.val2 < 0)) ? "-" : "",
	       abs(current.val1), abs(current.val2) / 100);

	LOG_INF("T: %d.%02d\n", temp.val1, temp.val2 / 10000);

	LOG_INF("Charger Status: %d, Error: %d\n", status.val1, error.val1);
#endif
	LOG_INF("Resource: %s", CONFIG_COAP_SAMPLE_RESOURCE);
    LOG_INF("Hostname: %s", CONFIG_COAP_SAMPLE_SERVER_HOSTNAME);
    LOG_INF("Port: %d", CONFIG_COAP_SAMPLE_SERVER_PORT);
    LOG_INF("Interval: %d", CONFIG_COAP_SAMPLE_REQUEST_INTERVAL_SECONDS);
	const char *protocol;

	if (IS_ENABLED(CONFIG_NRF_CLOUD_MQTT)) {
		protocol = "MQTT";
	} else if (IS_ENABLED(CONFIG_NRF_CLOUD_COAP)) {
		protocol = "CoAP";
	}

	LOG_INF("nRF Cloud multi-service sample has started, version: %s, protocol: %s",
		CONFIG_APP_VERSION, protocol);

	return 0;
}
