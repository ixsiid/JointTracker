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
#include "espidf_MPU6886.h"
#include "i2c.h"
#include "data.h"

// #define LGFX_M5STICK_C // Defined by platformio.ini
#include <LovyanGFX.hpp>

// #define CALCULATE_PROCESS_PER_SECOND

static LGFX lcd;

extern "C" {
void app_main();
}

static MadgwickAHRS *ahrs = nullptr;
static data_u worker_data;

uint8_t battery_voltage[2];

void ble_send_task(void *arg) {
	gamepad_t pad[2];
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	for (int i = 0; i < 2; i++) {
		pad[i].x = pad[i].y = pad[i].z = 0;
		pad[i].rx = pad[i].ry = pad[i].rz = 0;

		pad[i].slider	= 0x7fff;
		pad[i].buttons = 0b00000001;
	}

	int i = 0;
	while (true) {
		vTaskDelay(10 / portTICK_PERIOD_MS);
		if (BleGamePad.connected && ahrs != nullptr) {
			pad[0].rx	    = ahrs->q.x * 32767.0f;
			pad[0].ry	    = ahrs->q.y * 32767.0f;
			pad[0].rz	    = ahrs->q.z * 32767.0f;
			pad[0].slider = ahrs->q.w * 32767.0f;
			BleGamePad.send0(&pad[0]);
		}

#if GAMEPAD_COUNT > 1
		vTaskDelay(10 / portTICK_PERIOD_MS);
		if (BleGamePad.connected) {
			pad[1].rx	    = worker_data.ahrs.x * 32767.0f;
			pad[1].ry	    = worker_data.ahrs.y * 32767.0f;
			pad[1].rz	    = worker_data.ahrs.z * 32767.0f;
			pad[1].slider = worker_data.ahrs.w * 32767.0f;
			BleGamePad.send1(&pad[1]);
		}
#endif
	}
}

enum Page : int {
	GyroZeroBias,
	OriginCalibration,
	DirectionCalibration,
	Last,
};

int32_t page;

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
			lcd.drawString(buf, 5, 80);
		}
#endif

		button_home_press = !button_home_level;
		button_side_press = !button_side_level;
	}
}

void ahrs_task(void *arg) {
	vTaskDelay(1000 / portTICK_RATE_MS);

	IIMU *imu			   = (IIMU *)arg;
	ESPIDF::I2CMaster *i2c = (ESPIDF::I2CMaster *)imu->getI2CMaster();

	Calibration *calib = new Calibration(imu, 128);
	ahrs			    = new MadgwickAHRS(0.15f);
	ahrs->reset();

	// Gyro scale (±2000 degree/seconds Range by int16_t data -> rad / seconds)
	// (1.0f / 32768.0f * 2000.0f / 180.0f * 3.14159265358979323846264338327950288f);
	const float s = 0.00106526443603169529841533860372f;

	// Accelemeter scale (± 8G Range by int16_t data -> m/s^2)
	const float t = 8.0 / 32768.0;

	Vector3<int32_t> a, g;

	calib->regist(Calibration::Mode::Gyro);
	while (true) {
		vTaskDelay(20 / portTICK_RATE_MS);
		if (calib->proccess()) break;
	}

#ifdef CALCULATE_PROCESS_PER_SECOND
	TickType_t start = xTaskGetTickCount();
	int proc		  = 0;
#endif

	uint32_t count = 0x8000;

	while (true) {
		vTaskDelay(0);
		if (++count >= 0x8000) { // 0x8000 -> 32sec 0x80000 -> 8.7min 本運用時は0x80000でよさそ
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

		calib->getAccelAdc(&a);
		calib->getGyroAdcWithCalibrate(&g);

		ahrs->update(g * s, a * t);

		/*
		|vTaskDelay|WithCalibrate|arhs->update|i2c_speed| pps|rate %|
		|----------|-------------|------------|---------|----|------|
		|         2|    TRUE     |    TRUE    |   400kHz|  50| 6.7 %|
		|         1|    TRUE     |    TRUE    |   400kHz| 100|  13 %|
		|         0|    TRUE     |    TRUE    |   100kHz| 380|  51 %|
		|         0|    TRUE     |    TRUE    |   400kHz| 750| 100 %|
		|         0|    TRUE     |    FALSE   |   400kHz| 760| 101 %|
		|         0|    FALSE    |    TRUE    |   400kHz| 830| 111 %|
		|         0|    TRUE     |    TRUE    |     1MHz| 965| 129 %| (動作不安定)
		|         0|    FALSE    |    FALSE   |     1MHz|1040| 139 %| (動作不安定)
		|         0|    FALSE    |    FALSE   |     2MHz|  動作せず |
		|         0|    FALSE    |    FALSE   |     5MHz|  動作せず |
		*/
	}
}

const char device[] = "Chest";

using namespace ESPIDF;

#include "data.h"

// Requirement "using namespace ESPIDF"
void parent_task(void *arg) {
	I2CMaster *i2c = new I2CMaster(&ESPIDF::M5Stick_Grove);

	data_u data;
	while (true) {
		vTaskDelay(10 / portTICK_RATE_MS);
		i2c->read_bytes(15, 0x23, data.raw, sizeof(data_u));

		/*
		printf("[0x%8x 0x%8x] (%.3f %.3f %.3f) %.3f\n",
			  data.header, data.footer,
			  data.ahrs.x, data.ahrs.y, data.ahrs.z,
			  data.ahrs.w);
		*/

		if (data.header == 0x01020304 && data.footer == 0xa7f32249) {
			worker_data.ahrs = data.ahrs;
		}
	}
}

void app_main(void) {
	page = GyroZeroBias;

	lcd.init();
	lcd.setRotation(0);
	lcd.setBrightness(64);  // [0 - 255]
	lcd.setColorDepth(16);

	lcd.startWrite();
	lcd.drawString(device, 3, 3);
	lcd.endWrite();

	ESP_ERROR_CHECK(BleGamePad.begin(device));

#if M5STICK
	I2CMaster *i2c = new I2CMaster(&M5Stick_Internal);
#elif M5STACK
	I2CMaster *i2c = new I2CMaster(&M5Stack_Internal);
#endif

	// BluetoothはCore0
	xTaskCreatePinnedToCore(ble_send_task, "hid_task", 2048, nullptr, 5, nullptr, 0);

	IIMU *imu = new MPU6886(i2c);
	xTaskCreatePinnedToCore(ahrs_task, "ahrs", 1024 * 4, imu, 10, nullptr, 1);

	gpio_config_t button;
	button.mode		= gpio_mode_t::GPIO_MODE_INPUT;
	button.intr_type	= gpio_int_type_t::GPIO_INTR_DISABLE;
	button.pin_bit_mask = (1ULL << gpio_num_t::GPIO_NUM_39) | (1ULL << gpio_num_t::GPIO_NUM_37);
	button.pull_down_en = gpio_pulldown_t::GPIO_PULLDOWN_DISABLE;
	button.pull_up_en	= gpio_pullup_t::GPIO_PULLUP_ENABLE;
	ESP_ERROR_CHECK(gpio_config(&button));

	xTaskCreatePinnedToCore(lcd_task, "lcd", 1024 * 8, nullptr, 10, NULL, 0);
#if GAMEPAD_COUNT > 1
	xTaskCreatePinnedToCore(parent_task, "parent", 1024 * 4, nullptr, 10, NULL, 0);
#endif
}
