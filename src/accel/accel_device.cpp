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
#include "accel_device.h"

#define GRAVITY 9.80665
#define G_TO_MG 1000
#define RAW_DATA_TO_G_UNIT(X) (((float)(X))/((float)G_TO_MG))
#define RAW_DATA_TO_METRE_PER_SECOND_SQUARED_UNIT(X) (GRAVITY * (RAW_DATA_TO_G_UNIT(X)))

#define MIN_RANGE(RES) (-((1 << (RES))/2))
#define MAX_RANGE(RES) (((1 << (RES))/2)-1)

#define REACTIVE_ALERT_OFF	0
#define REACTIVE_ALERT_ON	1

#define UNKNOWN_NAME "UNKNOWN"

#define SENSOR_TYPE_ACCEL		"ACCEL"

#define INPUT_NAME	"accelerometer_sensor"
#define ACCEL_SENSORHUB_POLL_NODE_NAME "accel_poll_delay"

static sensor_info_t sensor_info = {
	id: 0x1,
	name: "ACCELEROMETER",
	type: SENSOR_DEVICE_ACCELEROMETER,
	event_type: (SENSOR_DEVICE_ACCELEROMETER << SENSOR_EVENT_SHIFT) | RAW_DATA_EVENT,
	model_name: UNKNOWN_NAME,
	vendor: UNKNOWN_NAME,
	min_range: 0,
	max_range: 0,
	resolution: 0,
	min_interval: 0,
	max_batch_count: 0,
	wakeup_supported: false
};

std::vector<uint32_t> accel_device::event_ids;

accel_device::accel_device()
: m_node_handle(-1)
, m_x(-1)
, m_y(-1)
, m_z(-1)
, m_polling_interval(1000)
, m_fired_time(0)
, m_sensorhub_controlled(false)
{
	const std::string sensorhub_interval_node_name = ACCEL_SENSORHUB_POLL_NODE_NAME;
	config::sensor_config &config = config::sensor_config::get_instance();

	node_info_query query;
	node_info info;

	if (!util::find_model_id(SENSOR_TYPE_ACCEL, m_model_id)) {
		_E("Failed to find model id");
		throw ENXIO;
	}

	query.sensorhub_controlled = m_sensorhub_controlled = util::is_sensorhub_controlled(sensorhub_interval_node_name);
	query.sensor_type = SENSOR_TYPE_ACCEL;
	query.key = INPUT_NAME;
	query.iio_enable_node_name = "accel_enable";
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

	if (!config.get(SENSOR_TYPE_ACCEL, m_model_id, ELEMENT_VENDOR, m_vendor)) {
		_E("[VENDOR] is empty");
		throw ENXIO;
	}

	_I("m_vendor = %s", m_vendor.c_str());

	if (!config.get(SENSOR_TYPE_ACCEL, m_model_id, ELEMENT_NAME, m_chip_name)) {
		_E("[NAME] is empty");
		throw ENXIO;
	}

	_I("m_chip_name = %s",m_chip_name.c_str());

	long resolution;

	if (!config.get(SENSOR_TYPE_ACCEL, m_model_id, ELEMENT_RESOLUTION, resolution)) {
		_E("[RESOLUTION] is empty");
		throw ENXIO;
	}

	m_resolution = (int)resolution;

	_I("m_resolution = %d",m_resolution);

	double raw_data_unit;

	if (!config.get(SENSOR_TYPE_ACCEL, m_model_id, ELEMENT_RAW_DATA_UNIT, raw_data_unit)) {
		_E("[RAW_DATA_UNIT] is empty");
		throw ENXIO;
	}

	m_raw_data_unit = (float)(raw_data_unit);
	_I("m_raw_data_unit = %f", m_raw_data_unit);

	m_node_handle = open(m_data_node.c_str(), O_RDWR);

	if (m_node_handle < 0) {
		_ERRNO(errno, _E, "accel handle open fail for accel processor");
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
	} else {
		if (!info.buffer_length_node_path.empty())
			util::set_node_value(info.buffer_length_node_path, 480);

		if (!info.buffer_enable_node_path.empty())
			util::set_node_value(info.buffer_enable_node_path, 1);

		update_value = [=]() {
			return this->update_value_iio();
		};
	}

	_I("accel_sensor is created!");
}

accel_device::~accel_device()
{
	close(m_node_handle);
	m_node_handle = -1;

	_I("accel_sensor is destroyed!");
}

int accel_device::get_poll_fd()
{
	return m_node_handle;
}

int accel_device::get_sensors(const sensor_info_t **sensors)
{
	sensor_info.model_name = m_chip_name.c_str();
	sensor_info.vendor = m_vendor.c_str();
	sensor_info.min_range = MIN_RANGE(m_resolution) * RAW_DATA_TO_METRE_PER_SECOND_SQUARED_UNIT(m_raw_data_unit);
	sensor_info.max_range = MAX_RANGE(m_resolution) * RAW_DATA_TO_METRE_PER_SECOND_SQUARED_UNIT(m_raw_data_unit);
	sensor_info.resolution = RAW_DATA_TO_METRE_PER_SECOND_SQUARED_UNIT(m_raw_data_unit);
	sensor_info.min_interval = 1;
	sensor_info.max_batch_count = 0;
	*sensors = &sensor_info;

	return 1;
}

bool accel_device::enable(uint32_t id)
{
	util::set_enable_node(m_enable_node, m_sensorhub_controlled, true, SENSORHUB_ACCELEROMETER_ENABLE_BIT);
	set_interval(id, m_polling_interval);

	m_fired_time = 0;
	_I("Enable accelerometer sensor");
	return true;
}

bool accel_device::disable(uint32_t id)
{
	util::set_enable_node(m_enable_node, m_sensorhub_controlled, false, SENSORHUB_ACCELEROMETER_ENABLE_BIT);

	_I("Disable accelerometer sensor");
	return true;
}

bool accel_device::set_interval(uint32_t id, unsigned long val)
{
	unsigned long long polling_interval_ns;

	polling_interval_ns = ((unsigned long long)(val) * 1000llu * 1000llu);

	if (!util::set_node_value(m_interval_node, polling_interval_ns)) {
		_E("Failed to set polling resource: %s", m_interval_node.c_str());
		return false;
	}

	_I("Interval is changed from %dms to %dms", m_polling_interval, val);
	m_polling_interval = val;
	return true;
}

bool accel_device::update_value_input_event(void)
{
	int accel_raw[3] = {0,};
	bool x,y,z;
	int read_input_cnt = 0;
	const int INPUT_MAX_BEFORE_SYN = 10;
	unsigned long long fired_time = 0;
	bool syn = false;

	x = y = z = false;

	struct input_event accel_input;
	_D("accel event detection!");

	while ((syn == false) && (read_input_cnt < INPUT_MAX_BEFORE_SYN)) {
		int len = read(m_node_handle, &accel_input, sizeof(accel_input));
		if (len != sizeof(accel_input)) {
			_E("accel_file read fail, read_len = %d",len);
			return false;
		}

		++read_input_cnt;

		if (accel_input.type == EV_REL) {
			switch (accel_input.code) {
			case REL_X:
				accel_raw[0] = (int)accel_input.value;
				x = true;
				break;
			case REL_Y:
				accel_raw[1] = (int)accel_input.value;
				y = true;
				break;
			case REL_Z:
				accel_raw[2] = (int)accel_input.value;
				z = true;
				break;
			default:
				_E("accel_input event[type = %d, code = %d] is unknown.", accel_input.type, accel_input.code);
				return false;
				break;
			}
		} else if (accel_input.type == EV_SYN) {
			syn = true;
			fired_time = util::get_timestamp(&accel_input.time);
		} else {
			_E("accel_input event[type = %d, code = %d] is unknown.", accel_input.type, accel_input.code);
			return false;
		}
	}

	if (syn == false) {
		_E("EV_SYN didn't come until %d inputs had come", read_input_cnt);
		return false;
	}

	if (x)
		m_x =  accel_raw[0];
	if (y)
		m_y =  accel_raw[1];
	if (z)
		m_z =  accel_raw[2];

	m_fired_time = fired_time;

	_D("m_x = %d, m_y = %d, m_z = %d, time = %lluus", m_x, m_y, m_z, m_fired_time);

	return true;
}

bool accel_device::update_value_iio(void)
{
	const int READ_LEN = 14;
	char data[READ_LEN] = {0,};

	struct pollfd pfd;

	pfd.fd = m_node_handle;
	pfd.events = POLLIN | POLLERR;
	pfd.revents = 0;

	int ret = poll(&pfd, 1, -1);

	if (ret == -1) {
		_ERRNO(errno, _E, "Failed to poll from m_node_handle:%d", m_node_handle);
		return false;
	} else if (!ret) {
		_E("poll timeout m_node_handle:%d", m_node_handle);
		return false;
	}

	if (pfd.revents & POLLERR) {
		_E("poll exception occurred! m_node_handle:%d", m_node_handle);
		return false;
	}

	if (!(pfd.revents & POLLIN)) {
		_E("poll nothing to read! m_node_handle:%d, pfd.revents = %d", m_node_handle, pfd.revents);
		return false;
	}

	int len = read(m_node_handle, data, sizeof(data));

	if (len != sizeof(data)) {
		_E("Failed to read data, m_node_handle:%d read_len:%d", m_node_handle, len);
		return false;
	}

	memcpy(&m_x, data, sizeof(short));
	memcpy(&m_y, (data + 2), sizeof(short));
	memcpy(&m_z, (data + 4), sizeof(short));
	memcpy(&m_fired_time, (data + 6), sizeof(long long));

	_D("m_x = %d, m_y = %d, m_z = %d, time = %lluus", m_x, m_y, m_z, m_fired_time);

	return true;
}

int accel_device::read_fd(uint32_t **ids)
{
	if (!update_value()) {
		_D("Failed to update value");
		return false;
	}

	event_ids.clear();
	event_ids.push_back(sensor_info.id);

	*ids = &event_ids[0];

	return event_ids.size();
}

int accel_device::get_data(uint32_t id, sensor_data_t **data, int *length)
{
	int remains = 1;
	sensor_data_t *sensor_data;
	sensor_data = (sensor_data_t *)malloc(sizeof(sensor_data_t));
	retvm_if(!sensor_data, -ENOMEM, "Memory allocation failed");

	sensor_data->accuracy = SENSOR_ACCURACY_GOOD;
	sensor_data->timestamp = m_fired_time;
	sensor_data->value_count = 3;
	sensor_data->values[0] = m_x;
	sensor_data->values[1] = m_y;
	sensor_data->values[2] = m_z;

	raw_to_base(sensor_data);

	*data = sensor_data;
	*length = sizeof(sensor_data_t);

	return --remains;
}

void accel_device::raw_to_base(sensor_data_t *data)
{
	data->value_count = 3;
	data->values[0] = RAW_DATA_TO_METRE_PER_SECOND_SQUARED_UNIT(data->values[0] * m_raw_data_unit);
	data->values[1] = RAW_DATA_TO_METRE_PER_SECOND_SQUARED_UNIT(data->values[1] * m_raw_data_unit);
	data->values[2] = RAW_DATA_TO_METRE_PER_SECOND_SQUARED_UNIT(data->values[2] * m_raw_data_unit);
}
