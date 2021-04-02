/* This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this software is
   distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include <fastmath.h>

#include "BleGamePad.h"

#include "Calibration.h"
#include "MadgwickAHRS.h"
#include "Vector3.h"
#include "espidf_LSM9DS1.h"
#include "i2c.h"
#include "data.h"

// #define LGFX_M5STICK_C // Defined by platformio.ini
#include <LovyanGFX.hpp>

// #define CALCULATE_PROCESS_PER_SECOND

#ifndef IMU_COUNT
#define IMU_COUNT 1
#endif

#if IMU_COUNT != GAMEPAD_COUNT
#error IMU_COUNT and GAMEPAD_COUNT must be the same value
#endif

#ifndef DEVICE_NAME
#error Please set DEVICE_NAME
#endif

static LGFX lcd;

extern "C" {
void app_main();
}

static MadgwickAHRS **ahrs_list;
static IIMU **imu_list;

uint8_t battery_voltage[IMU_COUNT];

void ble_send_task(void *arg) {
	gamepad_t pad[IMU_COUNT];
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	for (int i = 0; i < 2; i++) {
		pad[i].x = pad[i].y = pad[i].z = 0;
		pad[i].rx = pad[i].ry = pad[i].rz = 0;

		pad[i].slider	= 0x7fff;
		pad[i].buttons = 0b00000001;
	}

	while (true) {
		for (int i = 0; i < IMU_COUNT; i++) {
			IAHRS *ahrs = ahrs_list[i];
			vTaskDelay(10 / portTICK_PERIOD_MS);
			if (BleGamePad.connected && ahrs != nullptr) {
				pad[i].rx	    = ahrs->q.x * 32767.0f;
				pad[i].ry	    = ahrs->q.y * 32767.0f;
				pad[i].rz	    = ahrs->q.z * 32767.0f;
				pad[i].slider = ahrs->q.w * 32767.0f;
				BleGamePad.send(&pad[i], i);
			}
		}
	}
}

enum Page : int {
	GyroZeroBias,
	OriginCalibration,
	DirectionCalibration,
	Last,
};

int32_t page;

Vector3<int32_t> aa, gg, mm;

bool wait_gyro_calibration;
void draw_page() {
	lcd.startWrite();
	lcd.fillRect(0, 20, 135, 220, TFT_BLACK);
	lcd.setColor(TFT_WHITE);
	switch (page) {
		case GyroZeroBias:
			lcd.drawString("Gyro Zero Bias", 2, 24);
			lcd.drawString(wait_gyro_calibration ? "Calibrationg...." : "Done", 5, 40);
			break;
		case OriginCalibration:
			lcd.drawString("Origin Calibration", 2, 24);
			break;
		case DirectionCalibration:
			lcd.drawString("Direcation Calibration", 2, 24);
			break;
		default:
			lcd.drawString("Unknown Action", 2, 24);
			break;
	}

	char text[32];
	sprintf(text, "Battery: %f  ", (battery_voltage[0] << 4 | battery_voltage[1]) * 1.1f / 1000.0f);
	lcd.drawString(text, 10, 60);

	sprintf(text, "%+05d  %+05d  %+05d", gg.x, gg.y, gg.z);
	lcd.drawString(text, 3, 80);
	sprintf(text, "%+05d  %+05d  %+05d", aa.x, aa.y, aa.z);
	lcd.drawString(text, 3, 90);
	sprintf(text, "%+05d  %+05d  %+05d", mm.x, mm.y, mm.z);
	lcd.drawString(text, 3, 100);

	lcd.endWrite();
}

#ifdef CALCULATE_PROCESS_PER_SECOND
static int pps = 0;
#endif

bool begin_gyro_calibration;
bool origin_calibration, direction_calibration;
bool screen_invalidate;
bool button_home_press, button_side_press;
void lcd_task(void *arg) {
	vTaskDelay(1000 / portTICK_RATE_MS);
	draw_page();

	screen_invalidate = true;

	button_home_press = false;
	button_side_press = false;

	origin_calibration	  = false;
	direction_calibration = false;

#ifdef CALCULATE_PROCESS_PER_SECOND
	int _pps = 0;
	char buf[16];
#endif

	while (true) {
		vTaskDelay(100 / portTICK_RATE_MS);
		bool button_home_level = gpio_get_level(gpio_num_t::GPIO_NUM_37);
		bool button_side_level = gpio_get_level(gpio_num_t::GPIO_NUM_39);

		if (button_home_press && button_home_level) {
			// Page Action
			switch (page) {
				case Page::GyroZeroBias:
					begin_gyro_calibration = true;
					break;
				case Page::OriginCalibration:
					origin_calibration = true;
					break;
				case Page::DirectionCalibration:
					direction_calibration = true;
					break;
				default:
					break;
			}
		} else if (button_side_press && button_side_level) {
			// Page Change
			if (++page >= Page::Last) page = 0;
			screen_invalidate = true;
		}

		if (screen_invalidate) {
			screen_invalidate = false;
			draw_page();
		}

#ifdef CALCULATE_PROCESS_PER_SECOND
		if (_pps != pps) {
			sprintf(buf, "%d", _pps = pps);
			lcd.drawString(buf, 5, 120);
		}
#endif

		button_home_press = !button_home_level;
		button_side_press = !button_side_level;
	}
}

void ahrs_task(void *arg) {
	vTaskDelay(1000 / portTICK_RATE_MS);

	ESPIDF::I2CMaster *i2c = (ESPIDF::I2CMaster *)arg;

	Calibration **calib = new Calibration*[IMU_COUNT];
	for (int i = 0; i < IMU_COUNT; i++) {
		calib[i] = new Calibration(imu_list[i], 128);
		ahrs_list[i]	    = new MadgwickAHRS(1.0f);

		ahrs_list[i]->reset();
	}

	// Gyro scale (±2000 degree/seconds Range by int16_t data -> rad / seconds)
	// (1.0f / 32768.0f * 2000.0f / 180.0f * 3.14159265358979323846264338327950288f);
	const float s = 0.00106526443603169529841533860372f;

	// Accelemeter scale (± 8G Range by int16_t data -> m/s^2)
	const float t = 8.0 / 32768.0;

	// Magnetic scale (± 4Gauss by int16_t data -> gauss)
	const float u = 4.0 / 32768.0;

	Vector3<int32_t> a, g, m;

	for(int i=0; i<IMU_COUNT; i++) calib[i]->regist(Calibration::Mode::Gyro);
	bool calibrating = true;
	while (calibrating)  {
		vTaskDelay(20 / portTICK_RATE_MS);
		for(int i=0; i<IMU_COUNT; i++) calibrating &= !calib[i]->proccess();
	}

#ifdef CALCULATE_PROCESS_PER_SECOND
	TickType_t start = xTaskGetTickCount();
	int proc		  = 0;
#endif

	uint32_t count = 0x8000;

	int device_index = 0;
	while (true) {
		vTaskDelay(0);
		if (++count >= 0x8000) {	 // 0x8000 -> 32sec 0x80000 -> 8.7min 本運用時は0x80000でよさそ
			count = 0;

			i2c->read_bytes(0x68, 0x78, battery_voltage, 2);
			screen_invalidate = true;
		}

#ifdef CALCULATE_PROCESS_PER_SECOND
		TickType_t now = xTaskGetTickCount();
		proc++;
		if (now - start > 1000 / portTICK_RATE_MS) {
			pps	 = proc;
			start = now;
			proc	 = 0;
		}
#endif

		calib[device_index]->getAccelAdc(&a);
		calib[device_index]->getGyroAdc(&g);
		calib[device_index]->getMagAdc(&m);

		ahrs_list[device_index]->update(g * s, a * t, m * u);

		if (++device_index >= IMU_COUNT) device_index = 0;
	}
}

using namespace ESPIDF;

#include "data.h"

void app_main(void) {
	page = GyroZeroBias;

	lcd.init();
	lcd.setRotation(0);
	lcd.setBrightness(64);  // [0 - 255]
	lcd.setColorDepth(16);

	lcd.startWrite();
	lcd.drawString(DEVICE_NAME, 3, 3);
	lcd.endWrite();

	ESP_ERROR_CHECK(BleGamePad.begin(DEVICE_NAME));

	I2CMaster *i2c = new I2CMaster(&M5Stick_Grove);

	// BluetoothはCore0
	xTaskCreatePinnedToCore(ble_send_task, "hid_task", 2048, nullptr, 5, nullptr, 0);

	ahrs_list = new MadgwickAHRS *[IMU_COUNT];
	imu_list	= new IIMU *[IMU_COUNT];

	for (int i = 0; i < IMU_COUNT; i++) imu_list[i] = new LSM9DS1(i2c, i);
	xTaskCreatePinnedToCore(ahrs_task, "ahrs", 1024 * 4, i2c, 10, nullptr, 1);

	gpio_config_t button;
	button.mode		= gpio_mode_t::GPIO_MODE_INPUT;
	button.intr_type	= gpio_int_type_t::GPIO_INTR_DISABLE;
	button.pin_bit_mask = (1ULL << gpio_num_t::GPIO_NUM_39) | (1ULL << gpio_num_t::GPIO_NUM_37);
	button.pull_down_en = gpio_pulldown_t::GPIO_PULLDOWN_DISABLE;
	button.pull_up_en	= gpio_pullup_t::GPIO_PULLUP_ENABLE;
	ESP_ERROR_CHECK(gpio_config(&button));

	xTaskCreatePinnedToCore(lcd_task, "lcd", 1024 * 8, nullptr, 10, NULL, 0);
}
