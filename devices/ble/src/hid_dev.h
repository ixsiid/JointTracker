// Copyright 2017-2018 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HID_DEV_H__
#define HID_DEV_H__

#include "hidd_le_prf_int.h"

#ifdef __cplusplus
extern "C" {
#endif

/* HID Report type */
#define HID_TYPE_INPUT 1
#define HID_TYPE_OUTPUT 2
#define HID_TYPE_FEATURE 3

typedef uint8_t gamepad_cmd_t;

// HID report mapping table
typedef struct
{
	uint16_t handle;	  // Handle of report characteristic
	uint16_t cccdHandle;  // Handle of CCCD for report characteristic
	uint8_t id;		  // Report ID
	uint8_t type;		  // Report type
	uint8_t mode;		  // Protocol mode (report or boot)
} hid_report_map_t;

// HID dev configuration structure
typedef struct
{
	uint32_t idleTimeout;  // Idle timeout in milliseconds
	uint8_t hidFlags;	   // HID feature flags

} hid_dev_cfg_t;

void hid_dev_register_reports(uint8_t num_reports, hid_report_map_t *p_report);

void hid_dev_send_report(esp_gatt_if_t gatts_if, uint16_t conn_id,
					uint8_t id, uint8_t type, uint8_t length, uint8_t *data);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif /* HID_DEV_H__ */
