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

#include <string.h>
#include <esp_log.h>

#include "hidd_le_prf_int.h"

/// characteristic presentation information
struct prf_char_pres_fmt {
	/// Unit (The Unit is a UUID)
	uint16_t unit;
	/// Description
	uint16_t description;
	/// Format
	uint8_t format;
	/// Exponent
	uint8_t exponent;
	/// Name space
	uint8_t name_space;
};

// HID report mapping table
static hid_report_map_t hid_rpt_map[HID_NUM_REPORTS];

#define GAMEPAD_REPORT_MAP(n) \
    0x05, 0x01,        /* USAGE_PAGE (Generic Desktop)         */                            \
    0x09, 0x05,        /* USAGE (Gamepad)                      */                            \
    0xa1, 0x01,        /* COLLECTION (Application)             */                            \
    0x09, 0x01,        /*   USAGE (Pointer)                    */                            \
    0xa1, 0x00,        /*   COLLECTION (Physical)              */                            \
    0x85,  n+1,        /*     REPORT_ID (first report id is 1) */                            \
    /* ---------- Buttons (1 to 8)  -------------------------------------- */                \
    0x05, 0x09,        /*     USAGE_PAGE (Button)              */                            \
    0x19, 0x01,        /*     USAGE_MINIMUM (Button 1)         */                            \
    0x29, 0x08,        /*     USAGE_MAXIMUM (Button 8)         */                            \
    0x15, 0x00,        /*     LOGICAL_MINIMUM (0)              */                            \
    0x25, 0x01,        /*     LOGICAL_MAXIMUM (1)              */                            \
    0x75, 0x01,        /*     REPORT_SIZE (1)                  */                            \
    0x95, 0x08,        /*     REPORT_COUNT (8)                 */                            \
    0x81, 0x02,        /*     INPUT (Data, Variable, Absolute) ;8 button bits */             \
    /* ---------- X/Y/Z position    -------------------------------------- */                \
    0x05, 0x01,	   /*     USAGE_PAGE (Generic Desktop)     */                            \
    0x09, 0x30,	   /*     USAGE (X)                        */                            \
    0x09, 0x31,	   /*     USAGE (Y)                        */                            \
    0x09, 0x32,	   /*     USAGE (Z)                        */                            \
    0x16, 0x01, 0x80,  /*     LOGICAL_MINIMUM (-32767)         */                            \
    0x26, 0xff, 0x7f,  /*     LOGICAL_MAXIMUM (32767)          */                            \
    0x75, 0x10,	   /*     REPORT_SIZE (16)                 */                            \
    0x95, 0x03,	   /*     REPORT_COUNT (3)                 */                            \
    0x81, 0x02,	   /*     INPUT (Data, Variable, Absolute) ;6 bytes (X,Y,Z) */           \
    /* ---------- rX/rY/rZ rotation ---------------------------------------*/                \
    0x05, 0x01,	   /*     USAGE_PAGE (Generic Desktop)     */                            \
    0x09, 0x33,	   /*     USAGE (rX)                       */                            \
    0x09, 0x34,	   /*     USAGE (rY)                       */                            \
    0x09, 0x35,	   /*     USAGE (rZ)                       */                            \
    0x09, 0x36,	   /*     Usage (Slider := Qw)             */                            \
    0x16, 0x01, 0x80,  /*     LOGICAL_MINIMUM (-32767)         */                            \
    0x26, 0xff, 0x7f,  /*     LOGICAL_MAXIMUM (32767)          */                            \
    0x75, 0x10,	   /*     REPORT_SIZE (16)                 */                            \
    0x95, 0x04,	   /*     REPORT_COUNT (3)                 */                            \
    0x81, 0x02,	   /*     INPUT (Data, Variable, Absolute) ;6 bytes rX, rY, rZ */        \
    /* ----------- END -----------------------------------------------------------*/         \
    0xc0,	             /*     END_COLLECTION */ \
    0xc0,	             /*     END_COLLECTION */


// HID Report Map characteristic value
static const uint8_t hidReportMap[] = {
	GAMEPAD_REPORT_MAP(0)
#if GAMEPAD_COUNT > 1
	GAMEPAD_REPORT_MAP(1)
#if GAMEPAD_COUNT > 2
	GAMEPAD_REPORT_MAP(2)
#if GAMEPAD_COUNT > 3
	GAMEPAD_REPORT_MAP(3)
#endif
#endif
#endif
};

/// Battery Service Attributes Indexes
enum {
	BAS_IDX_SVC,

	BAS_IDX_BATT_LVL_CHAR,
	BAS_IDX_BATT_LVL_VAL,
	BAS_IDX_BATT_LVL_NTF_CFG,
	BAS_IDX_BATT_LVL_PRES_FMT,

	BAS_IDX_NB,
};

#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a)&0xFF)
#define PROFILE_NUM 1
#define PROFILE_APP_IDX 0

struct gatts_profile_inst {
	esp_gatts_cb_t gatts_cb;
	uint16_t gatts_if;
	uint16_t app_id;
	uint16_t conn_id;
};

hidd_le_env_t hidd_le_env;

uint8_t hidProtocolMode = HID_PROTOCOL_MODE_REPORT;

// HID report mapping table
//static hidRptMap_t  hidRptMap[HID_NUM_REPORTS];

// HID Information characteristic value
static const uint8_t hidInfo[HID_INFORMATION_LEN] = {
    LO_UINT16(0x0111), HI_UINT16(0x0111),  // bcdHID (USB HID version)
    0x00,							   // bCountryCode
    HID_FLAGS_NORMALLY_CONNECTABLE         // Flags
};

//HID_FLAGS_REMOTE_WAKE 0x01			// RemoteWake

// HID External Report Reference Descriptor
static uint16_t hidExtReportRefDesc = ESP_GATT_UUID_BATTERY_LEVEL;

// HID Report Reference characteristic descriptor, mouse input
static uint8_t hidReportRefGamepadIn[][HID_REPORT_REF_LEN] = {
    {HID_RPT_ID_GAMEPAD0_IN, HID_REPORT_TYPE_INPUT},
#if GAMEPAD_COUNT > 1
    {HID_RPT_ID_GAMEPAD1_IN, HID_REPORT_TYPE_INPUT},
#if GAMEPAD_COUNT > 2
    {HID_RPT_ID_GAMEPAD2_IN, HID_REPORT_TYPE_INPUT},
#if GAMEPAD_COUNT > 3
    {HID_RPT_ID_GAMEPAD3_IN, HID_REPORT_TYPE_INPUT},
#endif
#endif
#endif
};

// HID Report Reference characteristic descriptor, Feature
static uint8_t hidReportRefFeature[HID_REPORT_REF_LEN] =
    {HID_RPT_ID_FEATURE, HID_REPORT_TYPE_FEATURE};

/// hid Service uuid
static uint16_t hid_le_svc		= ATT_SVC_HID;
uint16_t hid_count				= 0;
esp_gatts_incl_svc_desc_t incl_svc = {0};

#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))
///the uuid definition
static const uint16_t primary_service_uuid		 = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t include_service_uuid		 = ESP_GATT_UUID_INCLUDE_SERVICE;
static const uint16_t character_declaration_uuid	 = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint16_t hid_info_char_uuid		 = ESP_GATT_UUID_HID_INFORMATION;
static const uint16_t hid_report_map_uuid		 = ESP_GATT_UUID_HID_REPORT_MAP;
static const uint16_t hid_control_point_uuid		 = ESP_GATT_UUID_HID_CONTROL_POINT;
static const uint16_t hid_report_uuid			 = ESP_GATT_UUID_HID_REPORT;
static const uint16_t hid_proto_mode_uuid		 = ESP_GATT_UUID_HID_PROTO_MODE;
static const uint16_t hid_repot_map_ext_desc_uuid	 = ESP_GATT_UUID_EXT_RPT_REF_DESCR;
static const uint16_t hid_report_ref_descr_uuid	 = ESP_GATT_UUID_RPT_REF_DESCR;

static const uint16_t hid_pnp_uuid = ESP_GATT_UUID_PNP_ID;

///the propoty definition
// static const uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read	   = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write_nr	   = ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
static const uint8_t char_prop_read_write  = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
// static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_WRITE|ESP_GATT_CHAR_PROP_BIT_NOTIFY;

/// battary Service
static const uint16_t battary_svc = ESP_GATT_UUID_BATTERY_SERVICE_SVC;

static const uint16_t bat_lev_uuid	    = ESP_GATT_UUID_BATTERY_LEVEL;
static const uint8_t bat_lev_ccc[2]    = {0x00, 0x00};
static const uint16_t char_format_uuid = ESP_GATT_UUID_CHAR_PRESENT_FORMAT;

static uint8_t battary_lev = 50;
/// Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t bas_att_db[BAS_IDX_NB] = {
    // Battary Service Declaration
    [BAS_IDX_SVC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(battary_svc), (uint8_t *)&battary_svc}},

    // Battary level Characteristic Declaration
    [BAS_IDX_BATT_LVL_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    // Battary level Characteristic Value
    [BAS_IDX_BATT_LVL_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&bat_lev_uuid, ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t), &battary_lev}},

    // Battary level Characteristic - Client Characteristic Configuration Descriptor
    [BAS_IDX_BATT_LVL_NTF_CFG] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(bat_lev_ccc), (uint8_t *)bat_lev_ccc}},

    // Battary level report Characteristic Declaration
    [BAS_IDX_BATT_LVL_PRES_FMT] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_format_uuid, ESP_GATT_PERM_READ, sizeof(struct prf_char_pres_fmt), 0, NULL}},
};

uint8_t pnp[7] = {0x01,0x02, 0xe5,0xab, 0xcd,0x01, 0x10};

/// Full Hid device Database Description - Used to add attributes into the database
static esp_gatts_attr_db_t hidd_le_gatt_db[HIDD_LE_IDX_NB] = {
    // HID Service Declaration
    [HIDD_LE_IDX_SVC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ_ENCRYPTED, sizeof(uint16_t), sizeof(hid_le_svc), (uint8_t *)&hid_le_svc}},

    // HID Service Declaration
    [HIDD_LE_IDX_INCL_SVC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&include_service_uuid, ESP_GATT_PERM_READ, sizeof(esp_gatts_incl_svc_desc_t), sizeof(esp_gatts_incl_svc_desc_t), (uint8_t *)&incl_svc}},

    // HID Information Characteristic Declaration
    [HIDD_LE_IDX_HID_INFO_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},
    // HID Information Characteristic Value
    [HIDD_LE_IDX_HID_INFO_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_info_char_uuid, ESP_GATT_PERM_READ, sizeof(hids_hid_info_t), sizeof(hidInfo), (uint8_t *)&hidInfo}},

    // HID Control Point Characteristic Declaration
    [HIDD_LE_IDX_HID_CTNL_PT_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write_nr}},
    // HID Control Point Characteristic Value
    [HIDD_LE_IDX_HID_CTNL_PT_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_control_point_uuid, ESP_GATT_PERM_WRITE, sizeof(uint8_t), 0, NULL}},

    // Report Map Characteristic Declaration
    [HIDD_LE_IDX_REPORT_MAP_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},
    // Report Map Characteristic Value
    [HIDD_LE_IDX_REPORT_MAP_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_map_uuid, ESP_GATT_PERM_READ, HIDD_LE_REPORT_MAP_MAX_LEN, sizeof(hidReportMap), (uint8_t *)&hidReportMap}},

    // Report Map Characteristic - External Report Reference Descriptor
    [HIDD_LE_IDX_REPORT_MAP_EXT_REP_REF] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_repot_map_ext_desc_uuid, ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&hidExtReportRefDesc}},

    // Protocol Mode Characteristic Declaration
    [HIDD_LE_IDX_PROTO_MODE_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},
    // Protocol Mode Characteristic Value
    [HIDD_LE_IDX_PROTO_MODE_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_proto_mode_uuid, (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE), sizeof(uint8_t), sizeof(hidProtocolMode), (uint8_t *)&hidProtocolMode}},

    [HIDD_LE_IDX_REPORT_GAMEPAD0_IN_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},
    [HIDD_LE_IDX_REPORT_GAMEPAD0_IN_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_uuid, ESP_GATT_PERM_READ, HIDD_LE_REPORT_MAX_LEN, 0, NULL}},
    [HIDD_LE_IDX_REPORT_GAMEPAD0_IN_CCC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE), sizeof(uint16_t), 0, NULL}},
    [HIDD_LE_IDX_REPORT_GAMEPAD0_REP_REF] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_ref_descr_uuid, ESP_GATT_PERM_READ, sizeof(hidReportRefGamepadIn[0]), sizeof(hidReportRefGamepadIn[0]), hidReportRefGamepadIn[0]}},

#if GAMEPAD_COUNT > 1
    [HIDD_LE_IDX_REPORT_GAMEPAD1_IN_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},
    [HIDD_LE_IDX_REPORT_GAMEPAD1_IN_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_uuid, ESP_GATT_PERM_READ, HIDD_LE_REPORT_MAX_LEN, 0, NULL}},
    [HIDD_LE_IDX_REPORT_GAMEPAD1_IN_CCC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE), sizeof(uint16_t), 0, NULL}},
    [HIDD_LE_IDX_REPORT_GAMEPAD1_REP_REF] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_ref_descr_uuid, ESP_GATT_PERM_READ, sizeof(hidReportRefGamepadIn[1]), sizeof(hidReportRefGamepadIn[1]), hidReportRefGamepadIn[1]}},

#if GAMEPAD_COUNT > 2
    [HIDD_LE_IDX_REPORT_GAMEPAD2_IN_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},
    [HIDD_LE_IDX_REPORT_GAMEPAD2_IN_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_uuid, ESP_GATT_PERM_READ, HIDD_LE_REPORT_MAX_LEN, 0, NULL}},
    [HIDD_LE_IDX_REPORT_GAMEPAD2_IN_CCC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE), sizeof(uint16_t), 0, NULL}},
    [HIDD_LE_IDX_REPORT_GAMEPAD2_REP_REF] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_ref_descr_uuid, ESP_GATT_PERM_READ, sizeof(hidReportRefGamepadIn[2]), sizeof(hidReportRefGamepadIn[2]), hidReportRefGamepadIn[2]}},

#if GAMEPAD_COUNT > 3
    [HIDD_LE_IDX_REPORT_GAMEPAD3_IN_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},
    [HIDD_LE_IDX_REPORT_GAMEPAD3_IN_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_uuid, ESP_GATT_PERM_READ, HIDD_LE_REPORT_MAX_LEN, 0, NULL}},
    [HIDD_LE_IDX_REPORT_GAMEPAD3_IN_CCC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE), sizeof(uint16_t), 0, NULL}},
    [HIDD_LE_IDX_REPORT_GAMEPAD3_REP_REF] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_ref_descr_uuid, ESP_GATT_PERM_READ, sizeof(hidReportRefGamepadIn[3]), sizeof(hidReportRefGamepadIn[3]), hidReportRefGamepadIn[3]}},
#endif
#endif
#endif

    // Report Characteristic Declaration
    [HIDD_LE_IDX_REPORT_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},
    // Report Characteristic Value
    [HIDD_LE_IDX_REPORT_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_uuid, ESP_GATT_PERM_READ, HIDD_LE_REPORT_MAX_LEN, 0, NULL}},
    // Report Characteristic - Report Reference Descriptor
    [HIDD_LE_IDX_REPORT_REP_REF] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_ref_descr_uuid, ESP_GATT_PERM_READ, sizeof(hidReportRefFeature), sizeof(hidReportRefFeature), hidReportRefFeature}},

    [HIDD_LE_IDX_PNP_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},
    [HIDD_LE_IDX_PNP_VAL]  = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_pnp_uuid, ESP_GATT_PERM_READ, 7, 7, pnp}},

};

static void hid_add_id_tbl(void);

void esp_hidd_prf_cb_hdl(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
					esp_ble_gatts_cb_param_t *param) {
	switch (event) {
		case ESP_GATTS_REG_EVT: {
			esp_ble_gap_config_local_icon(ESP_BLE_APPEARANCE_GENERIC_HID);
			esp_hidd_cb_param_t hidd_param;
			hidd_param.init_finish.state = param->reg.status;
			if (param->reg.app_id == HIDD_APP_ID) {
				hidd_le_env.gatt_if = gatts_if;
				if (hidd_le_env.hidd_cb != NULL) {
					(hidd_le_env.hidd_cb)(ESP_HIDD_EVENT_REG_FINISH, &hidd_param);
					hidd_le_create_service(hidd_le_env.gatt_if);
				}
			}
			if (param->reg.app_id == BATTRAY_APP_ID) {
				hidd_param.init_finish.gatts_if = gatts_if;
				if (hidd_le_env.hidd_cb != NULL) {
					(hidd_le_env.hidd_cb)(ESP_BAT_EVENT_REG, &hidd_param);
				}
			}

			break;
		}
		case ESP_GATTS_CONF_EVT: {
			break;
		}
		case ESP_GATTS_CREATE_EVT:
			break;
		case ESP_GATTS_CONNECT_EVT: {
			esp_hidd_cb_param_t cb_param = {0};
			ESP_LOGI(HID_LE_PRF_TAG, "HID connection establish, conn_id = %x", param->connect.conn_id);
			memcpy(cb_param.connect.remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
			cb_param.connect.conn_id = param->connect.conn_id;
			hidd_clcb_alloc(param->connect.conn_id, param->connect.remote_bda);
			esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_NO_MITM);
			if (hidd_le_env.hidd_cb != NULL) {
				(hidd_le_env.hidd_cb)(ESP_HIDD_EVENT_BLE_CONNECT, &cb_param);
			}
			break;
		}
		case ESP_GATTS_DISCONNECT_EVT: {
			if (hidd_le_env.hidd_cb != NULL) {
				(hidd_le_env.hidd_cb)(ESP_HIDD_EVENT_BLE_DISCONNECT, NULL);
			}
			hidd_clcb_dealloc(param->disconnect.conn_id);
			break;
		}
		case ESP_GATTS_CLOSE_EVT:
			break;
		case ESP_GATTS_WRITE_EVT: {
			break;
		}
		case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
			if (param->add_attr_tab.num_handle == BAS_IDX_NB &&
			    param->add_attr_tab.svc_uuid.uuid.uuid16 == ESP_GATT_UUID_BATTERY_SERVICE_SVC &&
			    param->add_attr_tab.status == ESP_GATT_OK) {
				incl_svc.start_hdl = param->add_attr_tab.handles[BAS_IDX_SVC];
				incl_svc.end_hdl   = incl_svc.start_hdl + BAS_IDX_NB - 1;
				ESP_LOGI(HID_LE_PRF_TAG, "%s(), start added the hid service to the stack database. incl_handle = %d",
					    __func__, incl_svc.start_hdl);
				esp_ble_gatts_create_attr_tab(hidd_le_gatt_db, gatts_if, HIDD_LE_IDX_NB, 0);
			}
			if (param->add_attr_tab.num_handle == HIDD_LE_IDX_NB &&
			    param->add_attr_tab.status == ESP_GATT_OK) {
				memcpy(hidd_le_env.hidd_inst.att_tbl, param->add_attr_tab.handles,
					  HIDD_LE_IDX_NB * sizeof(uint16_t));
				ESP_LOGI(HID_LE_PRF_TAG, "hid svc handle = %x", hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_SVC]);
				hid_add_id_tbl();
				esp_ble_gatts_start_service(hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_SVC]);
			} else {
				esp_ble_gatts_start_service(param->add_attr_tab.handles[0]);
			}
			break;
		}

		default:
			break;
	}
}

void hidd_le_create_service(esp_gatt_if_t gatts_if) {
	/* Here should added the battery service first, because the hid service should include the battery service.
       After finish to added the battery service then can added the hid service. */
	esp_ble_gatts_create_attr_tab(bas_att_db, gatts_if, BAS_IDX_NB, 0);
}

void hidd_le_init(void) {
	// Reset the hid device target environment
	memset(&hidd_le_env, 0, sizeof(hidd_le_env_t));
}

void hidd_clcb_alloc(uint16_t conn_id, esp_bd_addr_t bda) {
	uint8_t i_clcb		= 0;
	hidd_clcb_t *p_clcb = NULL;

	for (i_clcb = 0, p_clcb = hidd_le_env.hidd_clcb; i_clcb < HID_MAX_APPS; i_clcb++, p_clcb++) {
		if (!p_clcb->in_use) {
			p_clcb->in_use	   = true;
			p_clcb->conn_id   = conn_id;
			p_clcb->connected = true;
			memcpy(p_clcb->remote_bda, bda, ESP_BD_ADDR_LEN);
			break;
		}
	}
	return;
}

bool hidd_clcb_dealloc(uint16_t conn_id) {
	uint8_t i_clcb		= 0;
	hidd_clcb_t *p_clcb = NULL;

	for (i_clcb = 0, p_clcb = hidd_le_env.hidd_clcb; i_clcb < HID_MAX_APPS; i_clcb++, p_clcb++) {
		memset(p_clcb, 0, sizeof(hidd_clcb_t));
		return true;
	}

	return false;
}

static struct gatts_profile_inst heart_rate_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
	   .gatts_cb = esp_hidd_prf_cb_hdl,
	   .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },

};

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
						  esp_ble_gatts_cb_param_t *param) {
	/* If event is register event, store the gatts_if for each profile */
	if (event == ESP_GATTS_REG_EVT) {
		if (param->reg.status == ESP_GATT_OK) {
			heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
		} else {
			ESP_LOGI(HID_LE_PRF_TAG, "Reg app failed, app_id %04x, status %d\n",
				    param->reg.app_id,
				    param->reg.status);
			return;
		}
	}

	do {
		int idx;
		for (idx = 0; idx < PROFILE_NUM; idx++) {
			if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
			    gatts_if == heart_rate_profile_tab[idx].gatts_if) {
				if (heart_rate_profile_tab[idx].gatts_cb) {
					heart_rate_profile_tab[idx].gatts_cb(event, gatts_if, param);
				}
			}
		}
	} while (0);
}

esp_err_t hidd_register_cb(void) {
	esp_err_t status;
	status = esp_ble_gatts_register_callback(gatts_event_handler);
	return status;
}

void hidd_set_attr_value(uint16_t handle, uint16_t val_len, const uint8_t *value) {
	hidd_inst_t *hidd_inst = &hidd_le_env.hidd_inst;
	if (hidd_inst->att_tbl[HIDD_LE_IDX_HID_INFO_VAL] <= handle &&
	    hidd_inst->att_tbl[HIDD_LE_IDX_REPORT_REP_REF] >= handle) {
		esp_ble_gatts_set_attr_value(handle, val_len, value);
	} else {
		ESP_LOGE(HID_LE_PRF_TAG, "%s error:Invalid handle value.", __func__);
	}
	return;
}

void hidd_get_attr_value(uint16_t handle, uint16_t *length, uint8_t **value) {
	hidd_inst_t *hidd_inst = &hidd_le_env.hidd_inst;
	if (hidd_inst->att_tbl[HIDD_LE_IDX_HID_INFO_VAL] <= handle &&
	    hidd_inst->att_tbl[HIDD_LE_IDX_REPORT_REP_REF] >= handle) {
		esp_ble_gatts_get_attr_value(handle, length, (const uint8_t **)value);
	} else {
		ESP_LOGE(HID_LE_PRF_TAG, "%s error:Invalid handle value.", __func__);
	}

	return;
}

static void hid_add_id_tbl(void) {
	int i = 0;

	// Gamepad input report
	hid_rpt_map[i].id		 = hidReportRefGamepadIn[0][0];
	hid_rpt_map[i].type		 = hidReportRefGamepadIn[0][1];
	hid_rpt_map[i].handle	 = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_GAMEPAD0_IN_VAL];
	hid_rpt_map[i].cccdHandle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_GAMEPAD0_IN_VAL];
	hid_rpt_map[i].mode		 = HID_PROTOCOL_MODE_REPORT;
		

#if GAMEPAD_COUNT > 1
	// Gamepad input report
	++i;
	hid_rpt_map[i].id		 = hidReportRefGamepadIn[1][0];
	hid_rpt_map[i].type		 = hidReportRefGamepadIn[1][1];
	hid_rpt_map[i].handle	 = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_GAMEPAD1_IN_VAL];
	hid_rpt_map[i].cccdHandle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_GAMEPAD1_IN_VAL];
	hid_rpt_map[i].mode		 = HID_PROTOCOL_MODE_REPORT;
#if GAMEPAD_COUNT > 2
	// Gamepad input report
	++i;
	hid_rpt_map[i].id		 = hidReportRefGamepadIn[2][0];
	hid_rpt_map[i].type		 = hidReportRefGamepadIn[2][1];
	hid_rpt_map[i].handle	 = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_GAMEPAD2_IN_VAL];
	hid_rpt_map[i].cccdHandle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_GAMEPAD2_IN_VAL];
	hid_rpt_map[i].mode		 = HID_PROTOCOL_MODE_REPORT;
#if GAMEPAD_COUNT > 3
	// Gamepad input report
	++i;
	hid_rpt_map[i].id		 = hidReportRefGamepadIn[3][0];
	hid_rpt_map[i].type		 = hidReportRefGamepadIn[3][1];
	hid_rpt_map[i].handle	 = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_GAMEPAD3_IN_VAL];
	hid_rpt_map[i].cccdHandle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_GAMEPAD3_IN_VAL];
	hid_rpt_map[i].mode		 = HID_PROTOCOL_MODE_REPORT;
#endif
#endif
#endif

	++i;
	// Feature report
	hid_rpt_map[i].id		 = hidReportRefFeature[0];
	hid_rpt_map[i].type		 = hidReportRefFeature[1];
	hid_rpt_map[i].handle	 = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_VAL];
	hid_rpt_map[i].cccdHandle = 0;
	hid_rpt_map[i].mode		 = HID_PROTOCOL_MODE_REPORT;

	// Setup report ID map
	hid_dev_register_reports(HID_NUM_REPORTS, hid_rpt_map);
}
