#pragma once

#include <driver/i2c.h>
#include <esp_log.h>
#include <stdio.h>

namespace ESPIDF {

struct wire_s {
	i2c_port_t i2cnum;
	gpio_num_t io_scl;
	gpio_num_t io_sda;
	uint32_t i2c_speed;
};

#define BUFFER_LENGTH ((size_t)512)
#define DEFAULT_WAIT_TICK (1000 / portTICK_PERIOD_MS)

class I2CMaster {
    public:
	static wire_s Default_M5Stack, Default_M5Stick, Default_M5Atom;
	I2CMaster(wire_s* conf);
	I2CMaster(i2c_port_t port, gpio_num_t scl, gpio_num_t sda, uint32_t freq_hz);

	esp_err_t get_last_error();

	uint8_t read(uint8_t address, uint8_t registry);
	esp_err_t read_bytes(uint8_t address, uint8_t registry, uint8_t* buffer, size_t buffer_length);
	esp_err_t write(uint8_t address, uint8_t registry, uint8_t data);
	esp_err_t write_bytes(uint8_t address, uint8_t registry, uint8_t* data, size_t data_length);

    private:
	esp_err_t begin_transmission(uint8_t address, uint8_t registry);
	i2c_port_t port;
	esp_err_t last_error;
};

class I2CSlave {
    public:
	I2CSlave(wire_s* conf, uint8_t slave_address);
	I2CSlave(i2c_port_t port, gpio_num_t scl, gpio_num_t sda, uint8_t slave_address);

	esp_err_t get_last_error();

	uint8_t read(TickType_t wait = DEFAULT_WAIT_TICK);
	size_t read_bytes(uint8_t* buffer, size_t buffer_length, TickType_t wait = DEFAULT_WAIT_TICK);
	esp_err_t write(uint8_t data, bool clear_buffer = false, TickType_t wait = DEFAULT_WAIT_TICK);
	esp_err_t write_bytes(uint8_t* data, size_t data_length, bool clear_buffer = false, TickType_t wait = DEFAULT_WAIT_TICK);

    private:
	i2c_port_t port;
	esp_err_t last_error;
};
}  // namespace ESPIDF
