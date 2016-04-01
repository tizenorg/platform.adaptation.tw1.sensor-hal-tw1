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

#include <util.h>
#include <sensor_common.h>
#include <sensor_log.h>
#include <sensor_config.h>
#include "proxi_device.h"

#define UNKNOWN_NAME "UNKNOWN"
#define SENSOR_TYPE_PROXI		"PROXI"

#define INPUT_NAME	"proximity_sensor"
#define PROXI_SENSORHUB_POLL_NODE_NAME "prox_poll_delay"

static sensor_info_t sensor_info = {
	id: 0x1,
	name: "Proximity Sensor",
	type: SENSOR_DEVICE_PROXIMITY,
	event_type: (SENSOR_DEVICE_PROXIMITY << SENSOR_EVENT_SHIFT) | RAW_DATA_EVENT,
	model_name: UNKNOWN_NAME,
	vendor: UNKNOWN_NAME,
	min_range: 0,
	max_range: 5,
	resolution: 1,
	min_interval: 1,
	max_batch_count: 0,
	wakeup_supported: false
};

std::vector<uint32_t> proxi_device::event_ids;

proxi_device::proxi_device()
: m_node_handle(-1)
, m_state(PROXIMITY_NODE_STATE_FAR)
, m_fired_time(0)
, m_sensorhub_controlled(false)
{
	const std::string sensorhub_interval_node_name = PROXI_SENSORHUB_POLL_NODE_NAME;
	config::sensor_config &config = config::sensor_config::get_instance();

	node_info_query query;
	node_info info;

	if (!util::find_model_id(SENSOR_TYPE_PROXI, m_model_id)) {
		_E("Failed to find model id");
		throw ENXIO;

	}

	query.sensorhub_controlled = m_sensorhub_controlled = util::is_sensorhub_controlled(sensorhub_interval_node_name);
	query.sensor_type = SENSOR_TYPE_PROXI;
	query.key = INPUT_NAME;
	query.iio_enable_node_name = "proximity_enable";
	query.sensorhub_interval_node_name = sensorhub_interval_node_name;

	if (!util::get_node_info(query, info)) {
		_E("Failed to get node info");
		throw ENXIO;
	}

	util::show_node_info(info);

	m_method = info.method;
	m_data_node = info.data_node_path;
	m_enable_node = info.enable_node_path;

	if (!config.get(SENSOR_TYPE_PROXI, m_model_id, ELEMENT_VENDOR, m_vendor)) {
		_E("[VENDOR] is empty");
		throw ENXIO;
	}

	_I("m_vendor = %s", m_vendor.c_str());

	if (!config.get(SENSOR_TYPE_PROXI, m_model_id, ELEMENT_NAME, m_chip_name)) {
		_E("[NAME] is empty");
		throw ENXIO;
	}

	_I("m_chip_name = %s",m_chip_name.c_str());

	m_node_handle = open(m_data_node.c_str(), O_RDWR);

	if (m_node_handle < 0) {
		_ERRNO(errno, _E, "proxi handle open fail for proxi device");
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

	_I("Proxi_sensor_hal is created!");
}

proxi_device::~proxi_device()
{
	close(m_node_handle);
	m_node_handle = -1;

	_I("Proxi_sensor_hal is destroyed!");
}

int proxi_device::get_poll_fd(void)
{
	return m_node_handle;
}

int proxi_device::get_sensors(const sensor_info_t **sensors)
{
	sensor_info.model_name = m_chip_name.c_str();
	sensor_info.vendor = m_vendor.c_str();
	*sensors = &sensor_info;

	return 1;
}

bool proxi_device::enable(uint32_t id)
{
	util::set_enable_node(m_enable_node, m_sensorhub_controlled, true, SENSORHUB_PROXIMITY_ENABLE_BIT);

	m_fired_time = 0;
	INFO("Enable proxi sensor");
	return true;
}

bool proxi_device::disable(uint32_t id)
{
	util::set_enable_node(m_enable_node, m_sensorhub_controlled, false, SENSORHUB_PROXIMITY_ENABLE_BIT);

	INFO("Disable proxi sensor");
	return true;
}

bool proxi_device::set_interval(uint32_t id, unsigned long val)
{
	return false;
}

bool proxi_device::update_value_input_event(void)
{
	struct input_event proxi_event;
	_I("proxi event detection!");

	int len = read(m_node_handle, &proxi_event, sizeof(proxi_event));

	if (len == -1) {
		_ERRNO(errno, _E, "Failed to read from m_node_handle");
		return false;
	}

	_D("read event,  len : %d , type : %x , code : %x , value : %x", len, proxi_event.type, proxi_event.code, proxi_event.value);
	if ((proxi_event.type != EV_ABS) || (proxi_event.code != ABS_DISTANCE))
		return false;

	if (proxi_event.value != PROXIMITY_NODE_STATE_FAR && proxi_event.value != PROXIMITY_NODE_STATE_NEAR) {
		_E("PROXIMITY_STATE Unknown: %d",proxi_event.value);
		return false;
	}

	m_state = proxi_event.value;
	m_fired_time = util::get_timestamp(&proxi_event.time);

	return true;
}

int proxi_device::read_fd(uint32_t **ids)
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

int proxi_device::get_data(uint32_t id, sensor_data_t **data, int *length)
{
	int remains = 1;
	sensor_data_t *sensor_data;
	sensor_data = (sensor_data_t *)malloc(sizeof(sensor_data_t));
	retvm_if(!sensor_data, -ENOMEM, "Memory allocation failed");

	sensor_data->accuracy = SENSOR_ACCURACY_GOOD;
	sensor_data->timestamp = m_fired_time;
	sensor_data->value_count = 1;
	sensor_data->values[0] = m_state * 5;

	*data = sensor_data;
	*length = sizeof(sensor_data_t);

	return --remains;
}
