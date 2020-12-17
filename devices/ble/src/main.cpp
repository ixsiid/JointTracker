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
#include "GpioInterrupt.h"

// #define LGFX_M5STICK_C // Defined by platformio.ini
#include <LovyanGFX.hpp>

static LGFX lcd;

extern "C" {
void app_main();
}

bool enable;

gamepad_t pad;
float theta = 0.0f;
void hid_demo_task(void *pvParameters) {
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	while (1) {
		vTaskDelay(100 / portTICK_PERIOD_MS);
		pad.buttons = (pad.buttons + 1) & 0x7f;
		if (enable) pad.buttons |= 0x80;
		theta += 0.03f;
		pad.x = cosf(theta) * 32767.0f;
		pad.y = sinf(theta) * 32767.0f;
		pad.z = (cosf(theta) * sinf(theta)) * 32767.0f;

		pad.rx = cosf(theta) * 32767.0f;
		pad.ry = sinf(theta) * 32767.0f;
		pad.rz = (cosf(theta) * sinf(theta)) * 32767.0f;

		pad.slider = -(cosf(theta) * sinf(theta)) * 32767.0f;

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

void draw_page() {
	lcd.startWrite();
	lcd.fillRect(0, 20, 135, 220, TFT_BLACK);
	lcd.setColor(TFT_WHITE);
	switch (page) {
		case GyroZeroBias:
			lcd.drawString("Gyro Zero Bias", 2, 24);
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

volatile bool page_change;
void IRAM_ATTR on_side_click(void *arg) { page_change = true; }

void IRAM_ATTR on_front_click(void *arg) {}

void lcd_task(void *arg) {
	vTaskDelay(1000 / portTICK_RATE_MS);
	while (true) {
		vTaskDelay(100 / portTICK_RATE_MS);
		if (page_change) {
			page_change = false;
			if (++page >= Page::Last) page = 0;

			printf("page change -> %d", page);
			draw_page();
		}
	}
}

const char device[] = "VMT_25";

void app_main(void) {
	page	  = GyroZeroBias;
	enable = false;

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

	xTaskCreatePinnedToCore(&hid_demo_task, "hid_task", 2048, NULL, 5, NULL, 0);

	ESP_ERROR_CHECK(GpioInterrupt.begin(static_cast<uint64_t>(ButtonMask::M5StickCBoth)));
	ESP_ERROR_CHECK(GpioInterrupt.add_event_handler((gpio_num_t)Button::M5StickCFront, on_front_click, nullptr));
// 	ESP_ERROR_CHECK(GpioInterrupt.add_event_handler((gpio_num_t)Button::M5StickCSide, on_side_click, nullptr));

	draw_page();

	xTaskCreatePinnedToCore(lcd_task, "lcd", 1024 * 8, nullptr, 10, NULL, 0);
}
