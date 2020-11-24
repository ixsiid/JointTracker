#include <esp_intr_alloc.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "Calibration.h"
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
#define CONFIG_I2C_SLAVE_ADDRESS ((uint8_t)0x08)

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
static bool set_neutral_quaternion = false;
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
	Calibration *calib = new Calibration((IIMU *)arg, 128);
	calib->regist(Calibration::Mode::Gyro);
	ahrs = new MadgwickAHRS(0.5f);
	ahrs->reset();

	// Neutral調整用
	Quaternion p = Quaternion::identify();
	// Neutral調整後の結果を保存
	Quaternion q = Quaternion::identify();
	// Y軸周り調整用
	Quaternion ay = Quaternion::identify();

	Vector3<int32_t> g, a;

	int32_t max_a = 0, max_g = 0;

	while (true) {
		vTaskDelay(1);
		if (!calib->proccess()) {
			vTaskDelay(15 / portTICK_RATE_MS);
		} else if (start_gyro_calibration) {
			start_gyro_calibration = false;
			calib->regist(Calibration::Mode::Gyro);
		} else if (set_neutral_quaternion) {
			set_neutral_quaternion = false;

			p = ahrs->q.inverse();
			ay = Quaternion::identify();
		} else if (set_z_direction) {
			set_z_direction = false;

			/*
			// {0, -1, 0} を回転させた結果が {0, 0, -1}となるようにY軸回転を加える
			// {0, 0, -1} はビートセイバーのスタート画面が表示される方向
			Vector3<float> av = {-2.0f * (q.x * q.y - q.w * q.z),
							 -(q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z),
							 -2.0f * (q.y * q.z + q.w * q.x)};

			float theta = atan2f(av.x, -av.z);

			ay = {0.0f, sinf(theta), 0.0f, cosf(theta)};
			*/

			Quaternion dq = ahrs->q * p;
			// 正規化する
			Vector3<float> dv = {dq.x, dq.y, dq.z};
			dq.w = 1.0f / sqrtf(2.0f);
			dv *= dq.w / sqrtf(dv.Dot2());
			dq.x = dv.x;
			dq.y = dv.y;
			dq.z = dv.z;
			
			Quaternion correct = {dq.w, 0.0f, 0.0f, dq.w}; // X軸に90°のQuaternionがキャリブレーション後の結果
			ay = (p.inverse() * correct) * p * dq.inverse();
		} else {
			calib->getAccelAdc(&a);
			calib->getGyroAdc(&g);

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
			Quaternion dq = ahrs->q * p;
			q = p * ((ay * dq) * p.inverse());
			// q = p * (ay * ahrs->q);


			// SteamVR上に座標系を変換
			q = {-q.x, -q.y, q.z, q.w};

			data.q = q;
		}
	}
}

void app_main() {
	data.header = SYNC_HEADER;
	data.q	  = Quaternion::identify();
	data.footer = SYNC_FOOTER;

	I2CMaster *master = new I2CMaster(&I2CMaster::Default_M5Atom);
	I2CSlave *slave   = new I2CSlave(CONFIG_I2C_SLAVE_PORT_NUM, CONFIG_I2C_SLAVE_SCL, CONFIG_I2C_SLAVE_SDA, CONFIG_I2C_SLAVE_ADDRESS);

	IIMU *imu = new MPU6886(master);

	print_mux = xSemaphoreCreateMutex();
	xTaskCreate(i2c_slave_task, "i2c_test_task_0", 1024 * 2, slave, 10, NULL);
	xTaskCreate(data_update, "update_ahrs", 1024 * 2, imu, 10, NULL);
}
