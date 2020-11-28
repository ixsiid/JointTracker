#include <AtoMatrix.h>
#include <esp_intr_alloc.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "Calibration.h"
#include "Characteristics.h"
#include "MadgwickAHRS.h"
#include "Vector3.h"
#include "command.h"
#include "data.h"
#include "espidf_MPU6886.h"
#include "i2c.h"

#define TAG "SLAVE"

#define CONFIG_I2C_SLAVE_PORT_NUM ((i2c_port_t)1)
#define CONFIG_I2C_SLAVE_SDA ((gpio_num_t)26)
#define CONFIG_I2C_SLAVE_SCL ((gpio_num_t)32)

#ifndef SLAVE_ADDRESS
#define SLAVE_ADDRESS ((uint8_t)18)
#define CHARA_INDEX (1)
#endif

#define RX_BUFFER_LENGTH ((size_t)512)

extern "C" {
void app_main();
}

SemaphoreHandle_t print_mux = NULL;

data_u data;
IAHRS *ahrs;

// Gyro scale (±2000 degree/seconds Range by int16_t data -> rad / seconds)
// (1.0f / 32768.0f * 2000.0f / 180.0f * 3.14159265358979323846264338327950288f);
const float s = 0.00106526443603169529841533860372f;

// Accelemeter scale (± 8G Range by int16_t data -> m/s^2)
const float t = 8.0 / 32768.0;

using namespace ESPIDF;

static bool start_gyro_calibration = true;
static bool set_neutral_quaternion = true;
static bool set_z_direction		= false;

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
			case COMMAND_SET_NEUTRAL_QUATERNION:
				set_neutral_quaternion = true;
				break;
			case COMMAND_START_GYRO_CALIBRATION:
				// start_gyro_calibration = true;
				break;
			case COMMAND_SET_Z_DIRECTION:
				set_z_direction = true;
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

	float beta = 0.5f;
#else
	Calibration *calib = new Calibration((IIMU *)arg, Config::characterisitcs[CHARA_INDEX]);
	float beta	    = 0.02f;
#endif

	ahrs = new MadgwickAHRS(beta);
	ahrs->reset();

	// Neutral調整用
	Quaternion p = Quaternion::identify();

	Vector3<int32_t> g, a;

	int32_t max_a = 0, max_g = 0;

	while (true) {
		vTaskDelay(1);
		if (!calib->proccess()) {
			vTaskDelay(15 / portTICK_RATE_MS);
		} else if (start_gyro_calibration) {
			start_gyro_calibration = false;
			set_neutral_quaternion = true;
			calib->regist(Calibration::Mode::Gyro);
		} else if (set_neutral_quaternion) {
			// キャリブレーション終わった直後にもココが呼ばれる
			set_neutral_quaternion = false;

			p	    = ahrs->q.inverse();
			data.rot = Quaternion::identify();
		} else if (set_z_direction) {
			set_z_direction = false;

			// {0, -1, 0} を回転させた結果が {0, 0, -1}となるようにY軸回転を加える
			// {0, 0, -1} はビートセイバーのスタート画面が表示される方向
			// 親がY軸方向に存在するという前提になっているため一般化したい ⇒ 親情報がSlave側には含まれていないのがネック
			Quaternion q	   = p * ahrs->q;
			Vector3<float> av = {2.0f * (q.x * q.y - q.w * q.z),
							 (q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z),
							 2.0f * (q.y * q.z + q.w * q.x)};

			float theta = 0.5f * atan2f(av.x, -av.z);

			data.rot = {0.0f, sinf(theta), 0.0f, cosf(theta)};
		} else {
			calib->getData(&a, &g);

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

			ahrs->update(g * s, a * t);

			Quaternion q = data.rot * (p * ahrs->q);

			// SteamVR上に座標系を変換
			data.q = {-q.x, -q.y, q.z, q.w};
		}
	}
}

void app_main() {
	data.header = SYNC_HEADER;
	data.q	  = Quaternion::identify();
	data.rot	  = Quaternion::identify();
	data.footer = SYNC_FOOTER;

	I2CMaster *master = new I2CMaster(&I2CMaster::Default_M5Atom);
	I2CSlave *slave   = new I2CSlave(CONFIG_I2C_SLAVE_PORT_NUM, CONFIG_I2C_SLAVE_SCL, CONFIG_I2C_SLAVE_SDA, SLAVE_ADDRESS);

	IIMU *imu = new MPU6886(master);

	print_mux = xSemaphoreCreateMutex();
	xTaskCreate(i2c_slave_task, "i2c_test_task_0", 1024 * 2, slave, 10, NULL);
	xTaskCreate(data_update, "update_ahrs", 1024 * 16, imu, 10, NULL);
}
