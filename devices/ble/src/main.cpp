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

// #define LGFX_M5STICK_C // Defined by platformio.ini
#include <LovyanGFX.hpp>

static LGFX lcd;

extern "C" {
void app_main();
}

gamepad_t pad;
float theta = 0.0f;
void hid_demo_task(void *pvParameters) {
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	while (1) {
		vTaskDelay(20 / portTICK_PERIOD_MS);
		if (BleGamePad.connected) {
			BleGamePad.send(&pad);
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
	lcd.endWrite();
}

bool begin_gyro_calibration;
bool origin_calibration, direction_calibration;
bool screen_invalidate;
bool button_home_press, button_side_press;
void lcd_task(void *arg) {
	vTaskDelay(1000 / portTICK_RATE_MS);

	screen_invalidate = true;

	button_home_press = false;
	button_side_press = false;

	origin_calibration	  = false;
	direction_calibration = false;

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

		button_home_press = !button_home_level;
		button_side_press = !button_side_level;
	}
}

// Gyro scale (±2000 degree/seconds Range by int16_t data -> rad / seconds)
// (1.0f / 32768.0f * 2000.0f / 180.0f * 3.14159265358979323846264338327950288f);
const float s = 0.00106526443603169529841533860372f;

// Accelemeter scale (± 8G Range by int16_t data -> m/s^2)
const float t = 8.0 / 32768.0;

volatile Quaternion ahrs;

Vector3<float> bone = {0.0f, 0.0f, -0.25f};

void ahrs_task(void *arg) {
	vTaskDelay(1000 / portTICK_RATE_MS);

	Calibration *calib = new Calibration((IIMU *)arg, 128);
	calib->regist(Calibration::Mode::Gyro);

	MadgwickAHRS *ahrs = new MadgwickAHRS(1.0f);
	ahrs->reset();
	pad.buttons		   = 0b00000010;
	begin_gyro_calibration = false;
	wait_gyro_calibration  = true;
	screen_invalidate	   = true;

	Vector3<int32_t> a, g;

	Quaternion origin	 = Quaternion::identify();
	Quaternion direction = Quaternion::identify();

	while (true) {
		vTaskDelay(1);

		if (!calib->proccess()) {
			vTaskDelay(15 / portTICK_RATE_MS);
		} else if (begin_gyro_calibration) {
			begin_gyro_calibration = false;
			wait_gyro_calibration  = true;

			calib->regist(Calibration::Mode::Gyro);

			pad.buttons = 0b00000010;

			screen_invalidate = true;
		} else if (wait_gyro_calibration) {
			wait_gyro_calibration = false;
			pad.buttons		  = 0b00000001;

			screen_invalidate = true;
		} else if (origin_calibration) {
			origin_calibration = false;

			origin	= ahrs->q.inverse();
			direction = Quaternion::identify();

			// page++;
			// screen_invalidate = true;
		} else if (direction_calibration) {
			direction_calibration = false;

			Vector3<float> b = (ahrs->q * origin) * bone;
			float _05_theta  = (atan2(b.y, b.x) + 3.1415926535897932384626433832795f * 0.5f) * 0.5f;
			direction		  = Quaternion::xyzw(0.0f, 0.0f, cosf(_05_theta), sinf(_05_theta));

			// page--;
			// screen_invalidate = true;
		} else {
			calib->getAccelAdc(&a);
			calib->getGyroAdcWithCalibrate(&g);

			ahrs->update(g * s, a * t);

			Quaternion rot	  = ahrs->q * origin;
			Vector3<float> p = direction * (rot * bone);
			Quaternion q	  = direction * rot;

			pad.x = p.x * 32767.0f;
			pad.y = p.z * 32767.0f;
			pad.z = -p.y * 32767.0f;

			pad.rx	 = q.x * 32767.0f;
			pad.ry	 = q.z * 32767.0f;
			pad.rz	 = -q.y * 32767.0f;
			pad.slider = q.w * 32767.0f;
		}
	}
}

const char device[] = "Right";

using namespace ESPIDF;

void app_main(void) {
	page = GyroZeroBias;

	pad.x = pad.y = pad.z = 0;
	pad.rx = pad.ry = pad.rz = 0;

	pad.slider  = 0x7fff;
	pad.buttons = 0;

	lcd.init();
	lcd.setRotation(0);
	lcd.setBrightness(64);  // [0 - 255]
	lcd.setColorDepth(16);

	lcd.startWrite();
	lcd.drawString(device, 3, 3);
	lcd.endWrite();

	ESP_ERROR_CHECK(BleGamePad.begin(device));

	// BluetoothはCore0

	xTaskCreatePinnedToCore(hid_demo_task, "hid_task", 2048, NULL, 5, NULL, 0);

	I2CMaster *i2c = new I2CMaster(&I2CMaster::Default_M5Stick);
	IIMU *imu		= new MPU6886(i2c);

	xTaskCreatePinnedToCore(ahrs_task, "ahrs", 1024 * 4, imu, 10, nullptr, 1);

	gpio_config_t button;
	button.mode		= gpio_mode_t::GPIO_MODE_INPUT;
	button.intr_type	= gpio_int_type_t::GPIO_INTR_DISABLE;
	button.pin_bit_mask = (1ULL << gpio_num_t::GPIO_NUM_39) | (1ULL << gpio_num_t::GPIO_NUM_37);
	button.pull_down_en = gpio_pulldown_t::GPIO_PULLDOWN_DISABLE;
	button.pull_up_en	= gpio_pullup_t::GPIO_PULLUP_ENABLE;
	ESP_ERROR_CHECK(gpio_config(&button));

	draw_page();

	xTaskCreatePinnedToCore(lcd_task, "lcd", 1024 * 8, nullptr, 10, NULL, 0);
}
