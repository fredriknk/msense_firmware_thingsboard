/* Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#if CONFIG_TEMP_DATA_USE_SENSOR

#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm1300_charger.h>
#else /* CONFIG_TEMP_DATA_USE_SENSOR */

#include <zephyr/random/random.h>

#endif /* CONFIG_TEMP_DATA_USE_SENSOR */

#include "temperature.h"

LOG_MODULE_REGISTER(temperature, CONFIG_MULTI_SERVICE_LOG_LEVEL);

#if CONFIG_TEMP_DATA_USE_SENSOR

static const struct device *temp_sensor = DEVICE_DT_GET(DT_ALIAS(temp_sensor));
static const struct device *charger = DEVICE_DT_GET(DT_NODELABEL(npm1300_ek_charger));

int get_temperature(double *temp)
{
	int err;
	struct sensor_value data = {0};

	/* Fetch all data the sensor supports. */
	err = sensor_sample_fetch_chan(temp_sensor, SENSOR_CHAN_ALL);
	if (err) {
		LOG_ERR("Failed to sample sensor, error %d", err);
		return -ENODATA;
	}

	/* Pick out the ambient temperature data. */
	err = sensor_channel_get(temp_sensor, SENSOR_CHAN_AMBIENT_TEMP, &data);
	if (err) {
		LOG_ERR("Failed to read temperature, error %d", err);
		return -ENODATA;
	}

	*temp = sensor_value_to_double(&data);

	return 0;
}

int get_voltage(double *voltage)
{
	int err;
	struct sensor_value data = {0};

	/* Fetch all data the sensor supports. */
	err = sensor_sample_fetch_chan(charger, SENSOR_CHAN_ALL);
	if (err) {
		LOG_ERR("Failed to sample sensor, error %d", err);
		return -ENODATA;
	}

	/* Pick out the voltage data. */
	err = sensor_channel_get(charger, SENSOR_CHAN_GAUGE_VOLTAGE, &data);
	if (err) {
		LOG_ERR("Failed to read voltage, error %d", err);
		return -ENODATA;
	}

	*voltage = sensor_value_to_double(&data);

	return 0;
}

int get_charger(double *voltage,double *current,double *temp,double *chargestat)
{
	int err;
	struct sensor_value data = {0};

	/* Fetch all data the sensor supports. */
	err = sensor_sample_fetch_chan(charger, SENSOR_CHAN_ALL);
	if (err) {
		LOG_ERR("Failed to sample sensor, error %d", err);
		return -ENODATA;
	}

	/* Pick out the voltage data. */
	err = sensor_channel_get(charger, SENSOR_CHAN_GAUGE_VOLTAGE, &data);
	if (err) {
		LOG_ERR("Failed to read voltage, error %d", err);
		return -ENODATA;
	}
	*voltage = sensor_value_to_double(&data); 

	err = sensor_channel_get(charger, SENSOR_CHAN_GAUGE_AVG_CURRENT, &data);
	if (err) {
		LOG_ERR("Failed to read current, error %d", err);
		return -ENODATA;
	}
	*current = sensor_value_to_double(&data);

	sensor_channel_get(charger, SENSOR_CHAN_GAUGE_TEMP, &data);
	if (err) {
		LOG_ERR("Failed to read temperature, error %d", err);
		return -ENODATA;
	}
	*temp = sensor_value_to_double(&data);

	sensor_channel_get(charger, SENSOR_CHAN_NPM1300_CHARGER_STATUS, &data);
	if (err) {
		LOG_ERR("Failed to read charger status, error %d", err);
		return -ENODATA;
	}
	*chargestat = sensor_value_to_double(&data);

	return 0;
}

int get_data(double *value, int DEFINED_TYPE)
{
	int err;
	struct sensor_value data = {0};

	/* Fetch all data the sensor supports. */
	err = sensor_sample_fetch_chan(temp_sensor, SENSOR_CHAN_ALL);
	if (err) {
		LOG_ERR("Failed to sample sensor, error %d", err);
		return -ENODATA;
	}

	/* Pick out the defined type data. */
	err = sensor_channel_get(temp_sensor, DEFINED_TYPE, &data);
	if (err) {
		LOG_ERR("Failed to read temperature, error %d", err);
		return -ENODATA;
	}

	*value = sensor_value_to_double(&data);

	return 0;
}

int get_all_data(double *temp_variable, double *hum_variable, double *pressure_variable,double *gas_variable )
{
	int err;
	struct sensor_value data = {0};

	/* Fetch all data the sensor supports. */
	err = sensor_sample_fetch_chan(temp_sensor, SENSOR_CHAN_ALL);
	if (err) {
		LOG_ERR("Failed to sample sensor, error %d", err);
		return -ENODATA;
	}

	/* Pick out the defined type data. */
	err = sensor_channel_get(temp_sensor, SENSOR_CHAN_AMBIENT_TEMP, &data);
	if (err) {
		LOG_ERR("Failed to read temperature, error %d", err);
	}
	*temp_variable = sensor_value_to_double(&data);

	err = sensor_channel_get(temp_sensor, SENSOR_CHAN_HUMIDITY, &data);
	if (err) {
		LOG_ERR("Failed to read humidity, error %d", err);
	}
	*hum_variable = sensor_value_to_double(&data);

	err = sensor_channel_get(temp_sensor, SENSOR_CHAN_PRESS, &data);
	if (err) {
		LOG_ERR("Failed to read pressure, error %d", err);
	}
	*pressure_variable = sensor_value_to_double(&data);

	err = sensor_channel_get(temp_sensor, SENSOR_CHAN_GAS_RES, &data);
	if (err) {
		LOG_ERR("Failed to read gasres, error %d", err);
	}
	*gas_variable = sensor_value_to_double(&data);

	return 0;
}

#else /* CONFIG_TEMP_DATA_USE_SENSOR */

int get_temperature(double *temp)
{
	*temp = 22.0 + (sys_rand32_get() % 100) / 40.0;
	return 0;
}

int get_all_data(double *temp_variable, double *hum_variable, double *pressure_variable,double *gas_variable )
{
	*temp_variable = 22.0 + (sys_rand32_get() % 100) / 40.0;
	*hum_variable = 30.0 + (sys_rand32_get() % 100) / 40.0;
	*pressure_variable = 100.0 + (sys_rand32_get() % 100) / 40.0;
	*gas_variable = 70000.0 + (sys_rand32_get() % 100) / 40.0;
	return 0;
}

int get_voltage(double *voltage)
{
	*voltage = 3.7 + (sys_rand32_get() % 100) / 40.0;
	return 0;
}

int get_charger(double *voltage,double *current,double *temp,double *chargestat)
{
	*voltage = 3.7 + (sys_rand32_get() % 100) / 40.0;
	*current = 0.5 + (sys_rand32_get() % 100) / 40.0;
	*temp = 25.0 + (sys_rand32_get() % 100) / 40.0;
	*chargestat = 1 + (sys_rand32_get() % 100) / 40.0;
	return 0;
}

int get_data(double *value, int DEFINED_TYPE)
{
	*value = 22.0 + (sys_rand32_get() % 100) / 40.0;
	return 0;
}

#endif /* CONFIG_TEMP_DATA_USE_SENSOR */
