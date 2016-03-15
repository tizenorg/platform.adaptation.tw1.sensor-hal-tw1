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

#include "hrm_raw_data_reader.h"

#ifndef _HRM_RAW_DATA_READER_ADI_H_
#define _HRM_RAW_DATA_READER_ADI_H_

class hrm_raw_data_reader_adi : public hrm_raw_data_reader {
public:
	hrm_raw_data_reader_adi();
	~hrm_raw_data_reader_adi();

	virtual bool get_data(int handle, sensor_data_t &data);
};
#endif /*_HRM_RAW_DATA_READER_ADI_H_*/
