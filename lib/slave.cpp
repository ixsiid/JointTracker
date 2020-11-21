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
#define CONFIG_I2C_SLAVE_ADDRESS ((uint8_t)0x09)

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
		} else if (set_z_direction) {
			set_z_direction = false;
			// {0, -1, 0} を回転させた結果が {0, 0, -1}となるようにY軸回転を加える
			// {0, 0, -1} はビートセイバーのスタート画面が表示される方向
			Vector3<float> av = {-2.0f * (q.x * q.y - q.w * q.z),
							 -(q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z),
							 -2.0f * (q.y * q.z + q.w * q.x)};

			float theta = atan2f(av.x, -av.z);

			ay = {0.0f, sinf(theta), 0.0f, cosf(theta)};
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
			q = p * ahrs->q;
			// SteamVR上に座標系を変換
			q = {-q.x, -q.y, q.z, q.w};

			// Y軸周りを調整して送信バッファに保存
			data.q = ay * q;
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

/*

#include <BLEDevice.h>
#include <M5Atom.h>
#include <Wire.h>
#include <WireSlave.h>

#include "data.h"

#define DEVICE_NAME "Lower Tracker Branch"

#define SDA_PIN 26
#define SCL_PIN 32
#define I2C_SLAVE_ADDR 0x08

void onI2CRequest();
void onI2CReceive(int);

data_u data;
uint8_t z = 0;

BLEServer* server;
BLEAdvertising* adv;

uint8_t seq = 0;

void setup() {
	M5.begin(true, false, true);
	data.x = 0.1f;
	data.y = -0.5f;
	data.z = 1.2f;

	M5.dis.clear();
	M5.dis.drawpix(0, 0, HUE_GREEN);

	BLEDevice::init(DEVICE_NAME);
	server = BLEDevice::createServer();
	adv	  = server->getAdvertising();

	BLEAdvertisementData d = BLEAdvertisementData();
	d.setFlags(0x06);
	std::string payload = "";
	payload += (char)(4 + sizeof(data_u));
	payload += (char)0xff;  // Advertise Data Type: 0xff, Manufacturer specific data
	payload += (char)0xff;  // Manufacture ID (Low)
	payload += (char)0xff;  // Manufacture ID (High)
	payload += (char)seq;   // シーケンス番号
	for (int i = 0; i < sizeof(data_u); i++) payload += (char)data.raw[i];
	d.addData(payload);
	seq++;
	adv->setAdvertisementData(d);
	adv->start();

	Serial.printf("Start Lower Tracker Branch, %p\n", server);
}

int t = 0;

void loop() {
	delay(40);
	adv->stop();

	if (t++ > 50) {
		t = 0;
		data.x += 0.2f;
	}
	if (t == 20) {
		data.y -= 0.4f;
	}
	if (t == 30) {
		data.z += 0.1f;
	}

	BLEAdvertisementData d = BLEAdvertisementData();
	d.setFlags(0x06);
	std::string payload = "";
	payload += (char)(4 + sizeof(data_u));
	payload += (char)0xff;  // Advertise Data Type: 0xff, Manufacturer specific data
	payload += (char)0xff;  // Manufacture ID (Low)
	payload += (char)0xff;  // Manufacture ID (High)
	payload += (char)seq;   // シーケンス番号
	for (int i = 0; i < sizeof(data_u); i++) payload += (char)data.raw[i];
	d.addData(payload);
	seq++;
	adv->setAdvertisementData(d);
	adv->start();
}
*/