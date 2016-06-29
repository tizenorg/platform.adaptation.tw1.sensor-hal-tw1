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

#ifndef _HRM_RAW_DEVICE_H_
#define _HRM_RAW_DEVICE_H_

#include <sensor/sensor_hal.h>
#include <string>
#include <vector>
#include <functional>

class hrm_raw_device : public sensor_device {
public:
	hrm_raw_device();
	virtual ~hrm_raw_device();

	int get_poll_fd(void);
	int get_sensors(const sensor_info_t **sensors);

	bool enable(uint32_t id);
	bool disable(uint32_t id);

	bool set_interval(uint32_t id, unsigned long val);

	int read_fd(uint32_t **ids);
	int get_data(uint32_t id, sensor_data_t **data, int *length);

private:
	int m_node_handle;
	sensor_data_t m_data;
	unsigned long m_polling_interval;
	unsigned long m_raw_interval;
	unsigned long m_led_green_interval;

	bool m_interval_supported;
	bool m_sensorhub_controlled;
	int m_enable;

	int m_method;
	std::string m_data_node;
	std::string m_enable_node;
	std::string m_interval_node;

	std::function<bool (void)> update_value;

	std::vector<uint32_t> event_ids;

	bool update_value_input_event(void);
};
#endif /* _HRM_RAW_DEVICE_H_ */
