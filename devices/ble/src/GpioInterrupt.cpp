#include "GpioInterrupt.h"

esp_err_t GpioInterruptClass::begin(gpio_num_t port, gpio_isr_t callback, void *args, PullMode pull) {
	gpio_config_t button;
	button.mode		= gpio_mode_t::GPIO_MODE_INPUT;
	button.intr_type	= gpio_int_type_t::GPIO_INTR_POSEDGE;
	button.pin_bit_mask = 1ULL << port;

	if (pull == PullMode::PullDown) {
		button.pull_down_en = gpio_pulldown_t::GPIO_PULLDOWN_ENABLE;
		button.pull_up_en	= gpio_pullup_t::GPIO_PULLUP_DISABLE;
	} else if (pull == PullMode::PullUp) {
		button.pull_down_en = gpio_pulldown_t::GPIO_PULLDOWN_DISABLE;
		button.pull_up_en	= gpio_pullup_t::GPIO_PULLUP_ENABLE;
	} else {
		button.pull_down_en = gpio_pulldown_t::GPIO_PULLDOWN_DISABLE;
		button.pull_up_en	= gpio_pullup_t::GPIO_PULLUP_DISABLE;
	}

	esp_err_t err;
	if (err = gpio_config(&button)) return err;

	if (err = gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1)) return err;
	if (err = gpio_isr_handler_add(port, callback, nullptr)) return err;

	return ESP_OK;
}
