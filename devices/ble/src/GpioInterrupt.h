#pragma once

#include <driver/gpio.h>
#include <esp_err.h>

enum PullMode {
	None,
	PullDown,
	PullUp,
};

enum Button {
	M5AtomMatrix = gpio_num_t::GPIO_NUM_39,
	M5StickCFront = gpio_num_t::GPIO_NUM_39,
	M5StickCSide = gpio_num_t::GPIO_NUM_37,
};

class GpioInterruptClass {
    public:
	static esp_err_t begin(gpio_num_t port, gpio_isr_t callback, void *args, PullMode pull = PullMode::None);
	static esp_err_t begin(Button port, gpio_isr_t callback, void *args, PullMode pull = PullMode::None);
};

inline esp_err_t begin(Button port, gpio_isr_t callback, void *args, PullMode pull) {
	return begin(port, callback, args, pull);
}

extern GpioInterruptClass GpioInterrupt;
