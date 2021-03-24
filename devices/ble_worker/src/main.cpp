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

#define TAG "WORKER"

#define CONFIG_I2C_SLAVE_PORT_NUM ((i2c_port_t)1)
#define CONFIG_I2C_SLAVE_SDA ((gpio_num_t)26)
#define CONFIG_I2C_SLAVE_SCL ((gpio_num_t)32)

#define RX_BUFFER_LENGTH ((size_t)512)

extern "C" {
void app_main();
}

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

static IAHRS *ahrs;

static void i2c_slave_task(void *arg) {
	SemaphoreHandle_t mutex = xSemaphoreCreateMutex();

	I2CSlave *slave = (I2CSlave *)arg;
	uint8_t *rx	 = (uint8_t *)malloc(RX_BUFFER_LENGTH);

	data_u data;

	data.header = SYNC_HEADER;
	data.ahrs	  = Quaternion::identify();
	data.footer = SYNC_FOOTER;

	while (1) {
		vTaskDelay(1);
		xSemaphoreTake(mutex, portMAX_DELAY);
		slave->read_bytes(rx, 64, 0);

		switch (rx[0]) {
			case COMMAND_GET_QUATERNION:
				data.ahrs = ahrs->q;
				slave->write_bytes(data.raw, sizeof(data_u), true, 10 / portTICK_RATE_MS);
				printf("%x %x %f \n", data.header, data.footer, data.ahrs.w);
				break;
		}
		xSemaphoreGive(mutex);
	}
	vSemaphoreDelete(mutex);
	vTaskDelete(NULL);
}

static void ahrs_task(void *arg) {
	Calibration *calib = new Calibration((IIMU *)arg, 128);
	calib->regist(Calibration::Mode::Gyro);

	ahrs = new MadgwickAHRS(0.15f);
	ahrs->reset();

	Vector3<int32_t> g, a;

	// Gyro scale (±2000 degree/seconds Range by int16_t data -> rad / seconds)
	// (1.0f / 32768.0f * 2000.0f / 180.0f * 3.14159265358979323846264338327950288f);
	const float s = 0.00106526443603169529841533860372f;

	// Accelemeter scale (± 8G Range by int16_t data -> m/s^2)
	const float t = 8.0 / 32768.0;

	while (!calib->proccess()) vTaskDelay(15 / portTICK_RATE_MS);

	setNumber(SLAVE_ADDRESS, GREEN);
	matrix->update();

	while (true) {
		vTaskDelay(0);

		calib->getAccelAdc(&a);
		calib->getGyroAdcWithCalibrate(&g);

		ahrs->update(g * s, a * t);
	}
}

void app_main() {
	setNumber(SLAVE_ADDRESS, RED);
	matrix = new AtoMatrix(I2S_NUM_0, pixels);
	matrix->update();

	I2CMaster *master = new I2CMaster(&M5Atom_Internal);
	I2CSlave *slave   = new I2CSlave(&M5Atom_Grove, SLAVE_ADDRESS);

	IIMU *imu = new MPU6886(master);

	xTaskCreatePinnedToCore(i2c_slave_task, "i2c_slave", 1024 * 4, slave, 10, NULL, 0);
	xTaskCreatePinnedToCore(ahrs_task, "ahrs", 1024 * 4, imu, 10, nullptr, 1);
}
