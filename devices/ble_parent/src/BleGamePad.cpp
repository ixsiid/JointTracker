#include "BleGamePad.h"

#include <string.h>
#include <esp_log.h>
#include <esp_bt.h>
#include <esp_bt_main.h>

#include <nvs_flash.h>

#define TAG "BLE_GAMEPAD"

uint16_t BleGamePadClass::hid_conn_id = 0;
bool BleGamePadClass::connected = false;

char BleGamePadClass::device[32] = {0};
bool BleGamePadClass::initialized = false;

const uint8_t BleGamePadClass::hidd_service_uuid128[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00};

const esp_ble_adv_data_t BleGamePadClass::hidd_adv_data = {
    .set_scan_rsp		= false,
    .include_name		= true,
    .include_txpower	= true,
    .min_interval		= 0x0006,	 //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval		= 0x0010,	 //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance		= 0x03c4,	 // (0x03C0) HID Generic, (0x03C3) Joystick, (0x03C4) GamePad
    .manufacturer_len	= 0,
    .p_manufacturer_data = NULL,
    .service_data_len	= 0,
    .p_service_data		= NULL,
    .service_uuid_len	= sizeof(hidd_service_uuid128),
    .p_service_uuid		= (uint8_t *)hidd_service_uuid128,
    .flag				= 0x6,
};

const esp_ble_adv_params_t BleGamePadClass::hidd_adv_params = {
    .adv_int_min   = 0x20,
    .adv_int_max   = 0x30,
    .adv_type	    = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map	   = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

void BleGamePadClass::hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param) {
	switch (event) {
		case ESP_HIDD_EVENT_REG_FINISH: {
			if (param->init_finish.state == ESP_HIDD_INIT_OK) {
				//esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
				esp_ble_gap_set_device_name(device);
				esp_ble_gap_config_adv_data((esp_ble_adv_data_t *)&hidd_adv_data);
			}
			break;
		}
		case ESP_BAT_EVENT_REG: {
			break;
		}
		case ESP_HIDD_EVENT_DEINIT_FINISH:
			break;
		case ESP_HIDD_EVENT_BLE_CONNECT: {
			ESP_LOGI(TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
			hid_conn_id = param->connect.conn_id;
			break;
		}
		case ESP_HIDD_EVENT_BLE_DISCONNECT: {
			connected = false;
			ESP_LOGI(TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
			esp_ble_gap_start_advertising((esp_ble_adv_params_t *)&hidd_adv_params);
			break;
		}
		case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT: {
			ESP_LOGI(TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
			ESP_LOG_BUFFER_HEX(TAG, param->vendor_write.data, param->vendor_write.length);
		}
		default:
			break;
	}
	return;
}

void BleGamePadClass::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
	switch (event) {
		case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
			esp_ble_gap_start_advertising((esp_ble_adv_params_t *)&hidd_adv_params);
			break;
		case ESP_GAP_BLE_SEC_REQ_EVT:
			for (int i = 0; i < ESP_BD_ADDR_LEN; i++) {
				ESP_LOGD(TAG, "%x:", param->ble_security.ble_req.bd_addr[i]);
			}
			esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
			break;
		case ESP_GAP_BLE_AUTH_CMPL_EVT:
			connected = true;
			esp_bd_addr_t bd_addr;
			memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
			ESP_LOGI(TAG, "remote BD_ADDR: %08x%04x",
				    (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
				    (bd_addr[4] << 8) + bd_addr[5]);
			ESP_LOGI(TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
			ESP_LOGI(TAG, "pair status = %s", param->ble_security.auth_cmpl.success ? "success" : "fail");
			if (!param->ble_security.auth_cmpl.success) {
				ESP_LOGE(TAG, "fail reason = 0x%x", param->ble_security.auth_cmpl.fail_reason);
			}
			break;
		default:
			break;
	}
}

esp_err_t BleGamePadClass::begin(const char *device_name) {
	if (initialized) return ESP_OK;
	initialized = true;

	for (int i=0; i<31; i++) {
		device[i] = device_name[i];
		if (device[i] == '\0') break;
	}
	device[31] = '\0';

	esp_err_t err;
	
	// Initialize NVS.
	err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK(err);

	ESP_ERROR_CHECK(err = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

	err = esp_bt_controller_init(&bt_cfg);
	if (err) {
		ESP_LOGE(TAG, "%s initialize controller failed\n", __func__);
		return err;
	}

	err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
	if (err) {
		ESP_LOGE(TAG, "%s enable controller failed\n", __func__);
		return err;
	}

	err = esp_bluedroid_init();
	if (err) {
		ESP_LOGE(TAG, "%s init bluedroid failed\n", __func__);
		return err;
	}

	err = esp_bluedroid_enable();
	if (err) {
		ESP_LOGE(TAG, "%s init bluedroid failed\n", __func__);
		return err;
	}

	err = esp_hidd_profile_init();
	if (err) {
		ESP_LOGE(TAG, "%s init bluedroid failed\n", __func__);
		return err;
	}

	///register the callback function to the gap module
	esp_ble_gap_register_callback(gap_event_handler);

	// デモの自前実装
	esp_hidd_register_callbacks(hidd_event_callback);

	/* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
	esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;  //bonding with peer device after authentication
	esp_ble_io_cap_t iocap	   = ESP_IO_CAP_NONE;   //set the IO capability to No output No input
	uint8_t key_size		   = 16;			    //the key size should be 7~16 bytes
	uint8_t init_key		   = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
	uint8_t rsp_key		   = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
	esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
	/* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribute to you,
    and the response key means which key you can distribute to the Master;
    If your BLE device act as a master, the response key means you hope which types of key of the slave should distribute to you,
    and the init key means which key you can distribute to the slave. */
	esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

	return ESP_OK;
}

void BleGamePadClass::send0(gamepad_u *pad) {
	esp_hidd_send_gamepad_value(hid_conn_id, 1, pad->raw, sizeof(gamepad_t));
}

#if GAMEPAD_COUNT > 1
void BleGamePadClass::send1(gamepad_u *pad) {
	esp_hidd_send_gamepad_value(hid_conn_id, 2, pad->raw, sizeof(gamepad_t));
}

#if GAMEPAD_COUNT > 2
void BleGamePadClass::send1(gamepad_u *pad) {
	esp_hidd_send_gamepad_value(hid_conn_id, 3, pad->raw, sizeof(gamepad_t));
}

#if GAMEPAD_COUNT > 3
void BleGamePadClass::send1(gamepad_u *pad) {
	esp_hidd_send_gamepad_value(hid_conn_id, 4, pad->raw, sizeof(gamepad_t));
}
#endif
#endif
#endif