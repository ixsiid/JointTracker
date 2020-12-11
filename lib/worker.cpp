#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <nvs.h>

#include <driver/gpio.h>
#include <driver/uart.h>

#include <esp_intr_alloc.h>
#include <esp_log.h>
#include <esp_timer.h>

#include <AtoMatrix.h>

#include "Calibration.h"
#include "MadgwickAHRS.h"
#include "Vector3.h"
#include "data.h"
#include "espidf_MPU6886.h"
#include "i2c.h"

#define TAG "SLAVE"

#define CONFIG_I2C_SLAVE_PORT_NUM ((i2c_port_t)1)
#define CONFIG_I2C_SLAVE_SDA ((gpio_num_t)26)
#define CONFIG_I2C_SLAVE_SCL ((gpio_num_t)32)

/*
right_nee   13
right_ankle 14
left_nee    17
left_ankle  18
*/

// #define CHARA_INDEX (1)

#define RX_BUFFER_LENGTH ((size_t)512)

extern "C" {
void app_main();
}

uint8_t slave_address = 1;

SemaphoreHandle_t print_mux = NULL;

data_u data;
IAHRS *ahrs;

// Gyro scale (±2000 degree/seconds Range by int16_t data -> rad / seconds)
// (1.0f / 32768.0f * 2000.0f / 180.0f * 3.14159265358979323846264338327950288f);
const float s = 0.00106526443603169529841533860372f;

// Accelemeter scale (± 8G Range by int16_t data -> m/s^2)
const float t = 8.0 / 32768.0;

// Matrix LED表示
static uint32_t numbers[20] = {
    //.......1....2....3....4....5....
    0b00000000001000101001010010100010,	 //  0
    0b00000000001000110000100001000010,
    0b00000000001000101000010001000111,
    0b00000000011100001000100000100111,
    0b00000000010100101001110000100001,

    0b00000000011100100001100000100110,	 //  5
    0b00000000001100100001100010100010,
    0b00000000011100001000010001000010,
    0b00000000001000101000100010100111,
    0b00000000011100101001110000100001,

    0b00000001001010101101011010110010,	 // 10
    0b00000001001010110100101001010010,
    0b00000001001010101100011001010111,
    0b00000001011110001100101000110111,
    0b00000001010110101101111000110001,

    0b00000001011110100101101000110110,	 // 15
    0b00000001001110100101101010110010,
    0b00000001011110001100011001010010,
    0b00000001001010101100101010110111,
    0b00000001011110101101111000110001,
};

static AtoMatrix *matrix;
static uint8_t pixels[25];
void setNumber(uint8_t number, uint8_t color) {
	for (int i = 0; i < 25; i++) pixels[i] = (numbers[number] & (1 << i)) ? color : AtoMatrix::BLACK;
}

#define RED ((uint8_t)0b00001000)
#define GREEN ((uint8_t)0b00100000)

using namespace ESPIDF;

static bool start_gyro_calibration	 = true;
static bool finish_gyro_calibration = false;

static void i2c_slave_task(void *arg) {
	I2CSlave *slave = (I2CSlave *)arg;
	uint8_t *rx	 = (uint8_t *)malloc(RX_BUFFER_LENGTH);
	while (1) {
		xSemaphoreTake(print_mux, portMAX_DELAY);
		slave->read_bytes(rx, 64, 0);

		switch (rx[0]) {
			case COMMAND_GET_QUATERNION:
				slave->write_bytes(data.raw, sizeof(data_u), true, 10 / portTICK_RATE_MS);
				break;
		}
		xSemaphoreGive(print_mux);
		vTaskDelay(1);
	}
	vSemaphoreDelete(print_mux);
	vTaskDelete(NULL);
}

static void data_update(void *arg) {
#ifndef CHARA_INDEX
	Calibration *calib = new Calibration((IIMU *)arg, 128);
	calib->regist(Calibration::Mode::Gyro);
#else
	Calibration *calib = new Calibration((IIMU *)arg, Config::characterisitcs[CHARA_INDEX]);
#endif

	ahrs = new MadgwickAHRS(1.0f);
	ahrs->reset();

	Vector3<int32_t> g, a;
	// int32_t max_a = 0, max_g = 0;

	while (true) {
		vTaskDelay(1);
		if (!calib->proccess()) {
			vTaskDelay(15 / portTICK_RATE_MS);
		} else if (start_gyro_calibration) {
			start_gyro_calibration  = false;
			finish_gyro_calibration = true;

			setNumber(slave_address, RED);
			matrix->update();

			calib->regist(Calibration::Mode::Gyro);
		} else if (finish_gyro_calibration) {
			finish_gyro_calibration = false;
			setNumber(slave_address, GREEN);
			matrix->update();
		} else {
			calib->getAccelAdc(&a);
			calib->getGyroAdc(&g);

			/* ジャイロレンジオーバー検出用
			if (max_g < +g.x) max_g = +g.x;
			if (max_g < +g.y) max_g = +g.y;
			if (max_g < +g.z) max_g = +g.z;
			if (max_g < -g.x) max_g = -g.x;
			if (max_g < -g.y) max_g = -g.y;
			if (max_g < -g.z) max_g = -g.z;

			if (max_a < +a.x) max_a = +a.x;
			if (max_a < +a.y) max_a = +a.y;
			if (max_a < +a.z) max_a = +a.z;
			if (max_a < -a.x) max_a = -a.x;
			if (max_a < -a.y) max_a = -a.y;
			if (max_a < -a.z) max_a = -a.z;
			*/

			ahrs->update(g * s, a * t);
			data.ahrs = ahrs->q;

			/*
			デバッグ用
			printf("d: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
				  ahrs->q.x, ahrs->q.y, ahrs->q.z, ahrs->q.w,
				  q.x, q.y, q.z, q.w,
				  p.x, p.y, p.z, p.w);
			fflush(stdout);
			*/
		}
	}
}

static nvs_handle nvs;

static void uart_task(void *arg) {
	uint8_t buffer[4];
	int i = 0;

	while (true) {
		vTaskDelay(20);
		buffer[i] = getc(stdin);
		if (buffer[i] == 0xff) continue;

		if (buffer[i] == 0x0a) {
			int p0 = buffer[(i - 2) & 0b11] - '0';
			if (p0 != 1) p0 = 0;
			int p1 = buffer[(i - 1) & 0b11] - '0';
			if (p1 >= 0 && p1 < 10) {
				int a = p0 * 10 + p1;

				ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs));
				esp_err_t e = nvs_set_u8(nvs, "address", (uint8_t)a);
				if (e != ESP_OK) continue;
				
				nvs_commit(nvs);
				uint8_t b;
				nvs_get_u8(nvs, "address", &b);
				nvs_close(nvs);

				if (a == b) {
					printf("set address %d\n", a);
					esp_restart();
				} else {
					printf("failed save address %d\n", a);
				}
			}
		}

		i = (i + 1) & 0b11;
	}
}

/*
static void IRAM_ATTR on_button_click(void *arg) { start_gyro_calibration = true; }
*/

void app_main() {
	esp_err_t e = nvs_flash_init();
	if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		// NVS partition was truncated and needs to be erased
		// Retry nvs_flash_init
		ESP_ERROR_CHECK(nvs_flash_erase());
		e = nvs_flash_init();
	}
	ESP_ERROR_CHECK(e);
	ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs));
	nvs_get_u8(nvs, "address", &slave_address);
	nvs_close(nvs);
		
	setNumber(slave_address, RED);
	matrix = new AtoMatrix(I2S_NUM_0, pixels);
	matrix->update();

	data.header = SYNC_HEADER;
	data.ahrs	  = Quaternion::identify();
	data.footer = SYNC_FOOTER;

	I2CMaster *master = new I2CMaster(&I2CMaster::Default_M5Atom);
	I2CSlave *slave   = new I2CSlave(CONFIG_I2C_SLAVE_PORT_NUM, CONFIG_I2C_SLAVE_SCL, CONFIG_I2C_SLAVE_SDA, slave_address);

	IIMU *imu = new MPU6886(master);

	/*
	// GPIO割り込みを実装するとマトリックスの表示が乱れるので、コメントアウト

	gpio_config_t button;
	button.mode		= gpio_mode_t::GPIO_MODE_INPUT;
	button.intr_type	= gpio_int_type_t::GPIO_INTR_POSEDGE;
	button.pin_bit_mask = 1ULL << GPIO_NUM_39;
	button.pull_down_en = gpio_pulldown_t::GPIO_PULLDOWN_DISABLE;
	button.pull_up_en	= gpio_pullup_t::GPIO_PULLUP_DISABLE;
	ESP_ERROR_CHECK(gpio_config(&button));

	ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));
	ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_NUM_39, on_button_click, nullptr));
	*/

	print_mux = xSemaphoreCreateMutex();
	xTaskCreate(i2c_slave_task, "i2c_test_task_0", 1024 * 8, slave, 10, NULL);
	xTaskCreate(data_update, "update_ahrs", 1024 * 8, imu, 10, NULL);
	xTaskCreate(uart_task, "uart", 1024 * 2, nullptr, 10, nullptr);
}
