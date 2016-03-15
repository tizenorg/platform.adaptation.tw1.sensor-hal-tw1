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

#include <unistd.h>
#include <linux/input.h>
#include <util.h>
#include <sensor_log.h>
#include <sensor_common.h>
#include "hrm_raw_data_reader_adi.h"

hrm_raw_data_reader_adi::hrm_raw_data_reader_adi()
{

}

hrm_raw_data_reader_adi::~hrm_raw_data_reader_adi()
{

}

bool hrm_raw_data_reader_adi::get_data(int handle, sensor_data_t &data)
{
	const int SLOT_A_RED = 0;
	const int SLOT_AB_BOTH = 1;
	const int SLOT_B_IR = 2;

	const int INPUT_MAX_BEFORE_SYN = 20;
	bool syn = false;
	int read_input_cnt = 0;
	struct input_event hrm_raw_input;

	static int ir_sum = 0, red_sum = 0;
	static int ppg_ch[8] = { 0, };
	static int ppg_ch_idx = 0;
	static int sub_mode = 0;

	while ((syn == false) && (read_input_cnt < INPUT_MAX_BEFORE_SYN)) {
		int len = read(handle, &hrm_raw_input, sizeof(hrm_raw_input));
		if (len != sizeof(hrm_raw_input)) {
			_E("hrm_raw_file read fail, read_len = %d", len);
			return false;
		}

		++read_input_cnt;

		if (hrm_raw_input.type == EV_REL) {
			switch (hrm_raw_input.code) {
			case REL_X:
				red_sum = hrm_raw_input.value - 1;
				break;
			case REL_Y:
				ir_sum = hrm_raw_input.value - 1;
				break;
			case REL_Z:
				sub_mode = hrm_raw_input.value - 1;
				break;
			default:
				_E("hrm_raw_input event[type = %d, code = %d] is unknown", hrm_raw_input.type, hrm_raw_input.code);
				return false;
				break;
			}
		} else if (hrm_raw_input.type == EV_MSC) {
			if (hrm_raw_input.code == MSC_RAW) {
				ppg_ch[ppg_ch_idx++] = (hrm_raw_input.value - 1);
			} else {
				_E("hrm_raw_input event[type = %d, code = %d] is unknown", hrm_raw_input.type, hrm_raw_input.code);
				return false;
			}
		} else if (hrm_raw_input.type == EV_SYN) {
			syn = true;
			ppg_ch_idx = 0;

			memset(data.values, 0, sizeof(data.values));

			if (sub_mode == SLOT_A_RED) {
				for (int i = 6; i < 10; i++)
					data.values[i] = (float)(ppg_ch[i - 6]);

			} else if(sub_mode == SLOT_AB_BOTH) {
				data.values[0] = (float)(ir_sum);
				data.values[1] = (float)(red_sum);

				for (int i = 2; i < 6; i++)
					data.values[i] = (float)(ppg_ch[i + 2]);

				for (int i = 6; i < 10; i++)
					data.values[i] = (float)(ppg_ch[i - 6]);

			} else if (sub_mode == SLOT_B_IR) {
				for (int i = 2; i < 6; i++)
					data.values[i] = (float)(ppg_ch[i - 2]);
			}

			data.values[10] = (float)(sub_mode);
			data.value_count = 11;
			data.timestamp = util::get_timestamp(&hrm_raw_input.time);

		} else {
			_E("hrm_raw_input event[type = %d, code = %d] is unknown.", hrm_raw_input.type, hrm_raw_input.code);
			return false;
		}
	}

	if (!syn) {
		_E("EV_SYN didn't come until %d inputs had come", read_input_cnt);
		return false;
	}

	return true;
}
