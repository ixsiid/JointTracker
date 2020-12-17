#include "GpioInterrupt.h"


esp_err_t GpioInterruptClass::begin(uint64_t button_mask, PullMode pull) {
	esp_err_t err;

	gpio_config_t button;
	button.mode		= gpio_mode_t::GPIO_MODE_INPUT;
	button.intr_type	= gpio_int_type_t ::GPIO_INTR_POSEDGE;
	button.pin_bit_mask = button_mask;

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

	err = gpio_config(&button);
	if (err) return err;
	err = gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
	if (err) return err;

	return ESP_OK;
}

esp_err_t GpioInterruptClass::add_event_handler(gpio_num_t port, gpio_isr_t callback, void *args) {
	esp_err_t err;
	
	err = gpio_isr_handler_add(port, callback, args);
	if (err) return err;

	return ESP_OK;
}
