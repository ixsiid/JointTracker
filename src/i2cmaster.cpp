#include "i2c.h"

#define ACK_CHECK_EN 0x1	 /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */

namespace ESPIDF {

I2CMaster::I2CMaster(const wire_s* wire) : I2CMaster(wire->i2cnum, wire->io_scl, wire->io_sda, wire->i2c_speed) {}
I2CMaster::I2CMaster(i2c_port_t port, gpio_num_t scl, gpio_num_t sda, uint32_t freq_hz) {
	this->port = port;
	
	i2c_config_t conf;
	conf.mode		    = I2C_MODE_MASTER;
	conf.sda_io_num    = sda;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_io_num    = scl;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;

	conf.master.clk_speed = freq_hz;

	last_error = i2c_param_config(port, &conf);
	if (last_error == ESP_OK) {
		last_error = i2c_driver_install(port, conf.mode,
								  0,	 //I2C_MASTER_RX_BUF_DISABLE
								  0,	 //I2C_MASTER_TX_BUF_DISABLE
								  0);
	}
}

esp_err_t I2CMaster::get_last_error() { return last_error; }

esp_err_t I2CMaster::begin_transmission(uint8_t address, uint8_t registry) {
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 1));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, registry, 1));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(i2c_master_cmd_begin(port, cmd, DEFAULT_WAIT_TICK));
	i2c_cmd_link_delete(cmd);

	return 0;
}

esp_err_t I2CMaster::read_bytes(uint8_t address, uint8_t registry, uint8_t* buffer, size_t buffer_length) {
	if (buffer_length == 0) return ESP_OK;

	begin_transmission(address, registry);

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
	i2c_master_read(cmd, buffer, buffer_length, I2C_MASTER_LAST_NACK);
	i2c_master_stop(cmd);

	last_error = i2c_master_cmd_begin(port, cmd, DEFAULT_WAIT_TICK);
	i2c_cmd_link_delete(cmd);

	return last_error;
}

uint8_t I2CMaster::read(uint8_t address, uint8_t registry) {
	uint8_t data = 0;
	read_bytes(address, registry, &data, 1);
	return data;
}

esp_err_t I2CMaster::write(uint8_t address, uint8_t registry, uint8_t data) {
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, registry, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	last_error = i2c_master_cmd_begin(port, cmd, DEFAULT_WAIT_TICK);
	i2c_cmd_link_delete(cmd);
	return last_error;
}

esp_err_t I2CMaster::write_bytes(uint8_t address, uint8_t registry, uint8_t* data, size_t data_length) {
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, registry, ACK_CHECK_EN);
	i2c_master_write(cmd, data, data_length, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	last_error = i2c_master_cmd_begin(port, cmd, DEFAULT_WAIT_TICK);
	i2c_cmd_link_delete(cmd);
	return last_error;
}

}  // namespace ESPIDF
