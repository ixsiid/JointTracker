#pragma once

// HIDデバイスとして登録するGamepadの個数
#ifndef GAMEPAD_COUNT
	#error REQUIREMENT DEFINE GAMEPAD_COUNT (>= 1)
#endif

#include <stdint.h>
#include <esp_gap_ble_api.h>

#include "esp_hidd_prf_api.h"

#pragma pack(1)
struct gamepad_t {
	uint8_t buttons;
	int16_t x, y, z;
	int16_t rx, ry, rz;
	int16_t slider;
};
#pragma pack()

union gamepad_u {
	uint8_t raw[sizeof(gamepad_t)];
	gamepad_t pad;
};

class BleGamePadClass {
    public:
	static esp_err_t begin(const char *device_name);
	static void send(gamepad_t *pad, int index = 0);

	static void send0(gamepad_t *pad);
#if GAMEPAD_COUNT > 1
	static void send1(gamepad_t *pad);
#if GAMEPAD_COUNT > 2
	static void send2(gamepad_t *pad);
#if GAMEPAD_COUNT > 3
	static void send3(gamepad_t *pad);
#endif
#endif
#endif
	static bool connected;

    private:
     static bool initialized;
	static void send0(gamepad_u *pad);
	static void send1(gamepad_u *pad);
	static void send2(gamepad_u *pad);
	static void send3(gamepad_u *pad);
	static void send(gamepad_u *pad, int index);
	static char device[32];

	static uint16_t hid_conn_id;
	static const esp_ble_adv_data_t hidd_adv_data;
	static const esp_ble_adv_params_t hidd_adv_params;
	static const uint8_t hidd_service_uuid128[16];
	static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);
	static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
};

inline void BleGamePadClass::send(gamepad_t *pad, int index) { send((gamepad_u *)pad, index); }

inline void BleGamePadClass::send0(gamepad_t *pad) { send0((gamepad_u *)pad); }
#if GAMEPAD_COUNT > 1
inline void BleGamePadClass::send1(gamepad_t *pad) { send1((gamepad_u *)pad); }
#if GAMEPAD_COUNT > 2
inline void BleGamePadClass::send2(gamepad_t *pad) { send2((gamepad_u *)pad); }
#if GAMEPAD_COUNT > 3
inline void BleGamePadClass::send3(gamepad_t *pad) { send3((gamepad_u *)pad); }
#endif
#endif
#endif

extern BleGamePadClass BleGamePad;
