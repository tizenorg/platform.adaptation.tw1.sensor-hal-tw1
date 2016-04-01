/*
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <linux/input.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <math.h>

#include <util.h>
#include <sensor_common.h>
#include <sensor_log.h>
#include <sensor_config.h>
#include "pressure_device.h"

#define SEA_LEVEL_RESOLUTION 0.01
#define SEA_LEVEL_PRESSURE 101325.0

#define SENSOR_NAME "PRESSURE_SENSOR"
#define SENSOR_TYPE_PRESSURE "PRESSURE"

#define ELEMENT_TEMPERATURE_RESOLUTION	"TEMPERATURE_RESOLUTION"
#define ELEMENT_TEMPERATURE_OFFSET		"TEMPERATURE_OFFSET"

#define UNKNOWN_NAME "UNKNOWN"

#define INPUT_NAME "pressure_sensor"
#define PRESSURE_SENSORHUB_POLL_NODE_NAME "pressure_poll_delay"

static sensor_info_t sensor_info = {
	id: 0x1,
	name: SENSOR_NAME,
	type: SENSOR_DEVICE_PRESSURE,
	event_type: (SENSOR_DEVICE_PRESSURE << SENSOR_EVENT_SHIFT) | RAW_DATA_EVENT,
	model_name: UNKNOWN_NAME,
	vendor: UNKNOWN_NAME,
	min_range: 0,
	max_range: 0,
	resolution: 0,
	min_interval: 0,
	max_batch_count: 0,
	wakeup_supported: false
};

std::vector<uint32_t> pressure_device::event_ids;

pressure_device::pressure_device()
: m_node_handle(-1)
, m_pressure(0)
, m_resolution(0)
, m_temperature(0)
, m_sea_level_pressure(SEA_LEVEL_PRESSURE)
, m_temperature_resolution(0)
, m_temperature_offset(0)
, m_polling_interval(1000)
, m_fired_time(0)
, m_sensorhub_controlled(false)
{
	const std::string sensorhub_interval_node_name = PRESSURE_SENSORHUB_POLL_NODE_NAME;
	config::sensor_config &config = config::sensor_config::get_instance();

	node_info_query query;
	node_info info;

	if (!util::find_model_id(SENSOR_TYPE_PRESSURE, m_model_id)) {
		_E("Failed to find model id");
		throw ENXIO;

	}

	query.sensorhub_controlled = m_sensorhub_controlled = util::is_sensorhub_controlled(sensorhub_interval_node_name);
	query.sensor_type = SENSOR_TYPE_PRESSURE;
	query.key = INPUT_NAME;
	query.iio_enable_node_name = "pressure_enable";
	query.sensorhub_interval_node_name = sensorhub_interval_node_name;

	if (!util::get_node_info(query, info)) {
		_E("Failed to get node info");
		throw ENXIO;
	}

	util::show_node_info(info);

	m_method = info.method;
	m_data_node = info.data_node_path;
	m_enable_node = info.enable_node_path;
	m_interval_node = info.interval_node_path;

	if (!config.get(SENSOR_TYPE_PRESSURE, m_model_id, ELEMENT_VENDOR, m_vendor)) {
		_E("[VENDOR] is empty\n");
		throw ENXIO;
	}

	_I("m_vendor = %s", m_vendor.c_str());

	if (!config.get(SENSOR_TYPE_PRESSURE, m_model_id, ELEMENT_NAME, m_chip_name)) {
		_E("[NAME] is empty\n");
		throw ENXIO;
	}

	_I("m_chip_name = %s", m_chip_name.c_str());

	double min_range;

	if (!config.get(SENSOR_TYPE_PRESSURE, m_model_id, ELEMENT_MIN_RANGE, min_range)) {
		_E("[MIN_RANGE] is empty\n");
		throw ENXIO;
	}

	m_min_range = (float)min_range;
	_I("m_min_range = %f\n",m_min_range);

	double max_range;

	if (!config.get(SENSOR_TYPE_PRESSURE, m_model_id, ELEMENT_MAX_RANGE, max_range)) {
		_E("[MAX_RANGE] is empty\n");
		throw ENXIO;
	}

	m_max_range = (float)max_range;
	_I("m_max_range = %f\n",m_max_range);

	double raw_data_unit;

	if (!config.get(SENSOR_TYPE_PRESSURE, m_model_id, ELEMENT_RAW_DATA_UNIT, raw_data_unit)) {
		_E("[RAW_DATA_UNIT] is empty\n");
		throw ENXIO;
	}

	m_raw_data_unit = (float)(raw_data_unit);
	_I("m_raw_data_unit = %f\n", m_raw_data_unit);

	double temperature_resolution;
	if (!config.get(SENSOR_TYPE_PRESSURE, m_model_id, ELEMENT_TEMPERATURE_RESOLUTION, temperature_resolution)) {
		ERR("[TEMPERATURE_RESOLUTION] is empty\n");
		throw ENXIO;
	}

	m_temperature_resolution = (float)temperature_resolution;
	INFO("m_temperature_resolution = %f\n", m_temperature_resolution);

	double temperature_offset;

	if (!config.get(SENSOR_TYPE_PRESSURE, m_model_id, ELEMENT_TEMPERATURE_OFFSET, temperature_offset)) {
		ERR("[TEMPERATURE_OFFSET] is empty\n");
		throw ENXIO;
	}

	m_temperature_offset = (float)temperature_offset;
	INFO("m_temperature_offset = %f\n", m_temperature_offset);

	m_node_handle = open(m_data_node.c_str(), O_RDWR);

	if (m_node_handle < 0) {
		_ERRNO(errno, _E, "pressure handle open fail for pressure sensor");
		throw ENXIO;
	}

	if ((m_node_handle = open(m_data_node.c_str(),O_RDWR)) < 0) {
		_E("Failed to open handle(%d)", m_node_handle);
		throw ENXIO;
	}

	if (m_method == INPUT_EVENT_METHOD) {
		int clockId = CLOCK_MONOTONIC;
		if (ioctl(m_node_handle, EVIOCSCLOCKID, &clockId) != 0) {
			_E("Fail to set monotonic timestamp for %s", m_data_node.c_str());
			throw ENXIO;
		}

		update_value = [=]() {
			return this->update_value_input_event();
		};
	}

	_I("pressure_device is created!\n");
}

pressure_device::~pressure_device()
{
	close(m_node_handle);
	m_node_handle = -1;

	_I("pressure_device is destroyed!\n");
}

int pressure_device::get_poll_fd(void)
{
	return m_node_handle;
}

int pressure_device::get_sensors(const sensor_info_t **sensors)
{
	sensor_info.model_name = m_chip_name.c_str();
	sensor_info.vendor = m_vendor.c_str();
	sensor_info.min_range = m_min_range;
	sensor_info.max_range = m_max_range;
	sensor_info.resolution = m_raw_data_unit;
	sensor_info.min_interval = 1;
	sensor_info.max_batch_count = 0;
	*sensors = &sensor_info;

	return 1;
}

bool pressure_device::enable(uint32_t id)
{
	util::set_enable_node(m_enable_node, m_sensorhub_controlled, true, SENSORHUB_PRESSURE_ENABLE_BIT);
	set_interval(id, m_polling_interval);

	m_fired_time = 0;
	INFO("Enable pressure sensor");
	return true;
}

bool pressure_device::disable(uint32_t id)
{
	util::set_enable_node(m_enable_node, m_sensorhub_controlled, false, SENSORHUB_PRESSURE_ENABLE_BIT);

	INFO("Disable pressure sensor");
	return true;

}

bool pressure_device::set_interval(uint32_t id, unsigned long val)
{
	unsigned long long polling_interval_ns;

	polling_interval_ns = ((unsigned long long)(val) * 1000llu * 1000llu);

	if (!util::set_node_value(m_interval_node, polling_interval_ns)) {
		ERR("Failed to set polling resource: %s\n", m_interval_node.c_str());
		return false;
	}

	INFO("Interval is changed from %dms to %dms", m_polling_interval, val);
	m_polling_interval = val;
	return true;
}

bool pressure_device::update_value_input_event(void)
{
	int pressure_raw[3] = {0,};
	bool pressure = false;
	bool sea_level = false;
	bool temperature = false;
	int read_input_cnt = 0;
	const int INPUT_MAX_BEFORE_SYN = 10;
	unsigned long long fired_time = 0;
	bool syn = false;

	struct input_event pressure_event;
	_D("pressure event detection!");

	while ((syn == false) && (read_input_cnt < INPUT_MAX_BEFORE_SYN)) {
		int len = read(m_node_handle, &pressure_event, sizeof(pressure_event));
		if (len != sizeof(pressure_event)) {
			_E("pressure_file read fail, read_len = %d\n",len);
			return false;
		}

		++read_input_cnt;

		if (pressure_event.type == EV_REL) {
			switch (pressure_event.code) {
				case REL_HWHEEL:
					pressure_raw[0] = (int)pressure_event.value;
					pressure = true;
					break;
				case REL_DIAL:
					pressure_raw[1] = (int)pressure_event.value;
					sea_level = true;
					break;
				case REL_WHEEL:
					pressure_raw[2] = (int)pressure_event.value;
					temperature = true;
					break;
				default:
					_E("pressure_event event[type = %d, code = %d] is unknown.", pressure_event.type, pressure_event.code);
					return false;
					break;
			}
		} else if (pressure_event.type == EV_SYN) {
			syn = true;
			fired_time = util::get_timestamp(&pressure_event.time);
		} else {
			_E("pressure_event event[type = %d, code = %d] is unknown.", pressure_event.type, pressure_event.code);
			return false;
		}
	}

	if (syn == false) {
		_E("EV_SYN didn't come until %d inputs had come", read_input_cnt);
		return false;
	}

	if (pressure)
		m_pressure = pressure_raw[0];
	if (sea_level)
		m_sea_level_pressure = pressure_raw[1];
	if (temperature)
		m_temperature = pressure_raw[2];

	m_fired_time = fired_time;

	_D("m_pressure = %d, sea_level = %d, temperature = %d, time = %lluus", m_pressure, m_sea_level_pressure, m_temperature, m_fired_time);

	return true;
}

int pressure_device::read_fd(uint32_t **ids)
{
	if (!update_value()) {
		DBG("Failed to update value");
		return false;
	}

	event_ids.clear();
	event_ids.push_back(sensor_info.id);

	*ids = &event_ids[0];

	return event_ids.size();
}

int pressure_device::get_data(uint32_t id, sensor_data_t **data, int *length)
{
	int remains = 1;
	sensor_data_t *sensor_data;
	sensor_data = (sensor_data_t *)malloc(sizeof(sensor_data_t));
	retvm_if(!sensor_data, -ENOMEM, "Memory allocation failed");

	sensor_data->accuracy = SENSOR_ACCURACY_GOOD;
	sensor_data->timestamp = m_fired_time;
	sensor_data->value_count = 3;
	sensor_data->values[0] = m_pressure;
	sensor_data->values[1] = (float)m_sea_level_pressure;
	sensor_data->values[2] = m_temperature;

	raw_to_base(sensor_data);

	*data = sensor_data;
	*length = sizeof(sensor_data_t);

	return --remains;
}

void pressure_device::raw_to_base(sensor_data_t *data)
{
	data->values[0] = data->values[0] * m_raw_data_unit;
	m_sea_level_pressure = data->values[1] * SEA_LEVEL_RESOLUTION;
	data->values[1] = pressure_to_altitude(data->values[0]);
	data->values[2] = data->values[2] * m_temperature_resolution + m_temperature_offset;
}

float pressure_device::pressure_to_altitude(float pressure)
{
	/* TODO: use m_sea_level_pressure instread of SEA_LEVEL_PRESSURE */
	return 44330.0f * (1.0f - pow(pressure/SEA_LEVEL_PRESSURE, 1.0f/5.255f));
}

