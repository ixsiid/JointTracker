#pragma once

#include <driver/gpio.h>
#include <esp_err.h>

enum class PullMode {
	None,
	PullDown,
	PullUp,
};

enum class ButtonMask : uint64_t {
	M5StickCFront = (1ULL << gpio_num_t::GPIO_NUM_37),
	M5StickCSide  = (1ULL << gpio_num_t::GPIO_NUM_39),
	M5StickCBoth  = (1ULL << gpio_num_t::GPIO_NUM_37) | (1ULL << gpio_num_t::GPIO_NUM_39),
};

enum class Button {
	M5StickCFront = GPIO_NUM_37,
	M5StickCSide  = GPIO_NUM_39,
};

class GpioInterruptClass {
    public:
	static esp_err_t begin(uint64_t button_mask, PullMode pull = PullMode::None);
	static esp_err_t add_event_handler(gpio_num_t port, gpio_isr_t callback, void *args);
};

extern GpioInterruptClass GpioInterrupt;
