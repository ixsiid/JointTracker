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

#include <driver/gpio.h>

#include <fastmath.h>

#include "BleGamePad.h"

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
		vTaskDelay(100 / portTICK_PERIOD_MS);
		pad.buttons++;
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

void IRAM_ATTR on_button_click(void *arg) {}

const char device[] = "VMT_25";

void app_main(void) {
	lcd.init();
	lcd.setRotation(0);
	lcd.setBrightness(128); // [0 - 255]
	lcd.setColorDepth(16);

	lcd.startWrite();
	lcd.drawString(device, 3, 3);
	lcd.endWrite();

	BleGamePad.begin(device);

	pad.x = pad.y = pad.z = 0;
	pad.rx = pad.ry = pad.rz = 0;

	pad.slider  = 0x7fff;
	pad.buttons = 0;

	xTaskCreate(&hid_demo_task, "hid_task", 2048, NULL, 5, NULL);

	gpio_config_t button;
	button.mode		= gpio_mode_t::GPIO_MODE_INPUT;
	button.intr_type	= gpio_int_type_t::GPIO_INTR_POSEDGE;
	button.pin_bit_mask = 1ULL << GPIO_NUM_39;
	button.pull_down_en = gpio_pulldown_t::GPIO_PULLDOWN_DISABLE;
	button.pull_up_en	= gpio_pullup_t::GPIO_PULLUP_DISABLE;
	ESP_ERROR_CHECK(gpio_config(&button));

	ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));
	ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_NUM_39, on_button_click, nullptr));

	// xTaskCreatePinnedToCore(gpio_intr_task, "gpio", 1024 * 8, slave, 10, NULL, 1);
}
