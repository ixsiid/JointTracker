#include <M5Stack.h>
#include <WiFi.h>
#include <Wire.h>
#include <Preferences.h>

#include "OscClient.h"
#include "data.h"

// uint8_t who[128];

OscClient* osc;
VMTJointArgument_t osc_args;

data_u data;

Preferences pref;

struct JointConfigure {
	char root_serial[20];
	uint16_t reserved;
	uint8_t address;
	uint8_t tracker_index;
	Vector3<float> bone;
	Quaternion rotation;
};

struct Joint_s {
	char root_serial[20];
	uint16_t reserved;
	uint8_t address;
	uint8_t tracker_index;
	Vector3<float> bone;
	Quaternion rotation;
	Quaternion calibrate;
	Quaternion xy_correction;
};

// ボーンはIMUの座標系で
size_t fix_bone_count = 0;
Joint_s fix_bone[8];

size_t movable_count = 0;
Joint_s movable[8];

uint8_t vmt_host[4];
uint16_t vmt_port;

bool fix_send = true;

void printBone() {
	M5.Lcd.fillRect(0, 90, 320, 120, BLACK);
	M5.Lcd.setCursor(0, 90 + 2);

	Joint_s* allJointList[] = {fix_bone, movable};
	size_t count[]		    = {fix_bone_count, movable_count};
	for (int i = 0; i < 2; i++) {
		M5.Lcd.println(i == 0 ? "  Fix Joint" : "\n  Movable Joint");
		for (int n = 0; n < count[i]; n++) {
			Joint_s* j = allJointList[i] + n;
			M5.Lcd.printf("%2d %12s [%+.2f %+.2f %+.2f]\n", j->tracker_index, j->root_serial, j->bone.x, j->bone.y, j->bone.z);
		}
	}
}

#define CONFIGURE_CMD_WIFI 0x128422fd
#define CONFIGURE_CMD_REBOOT 0xff8d91a8
#define CONFIGURE_CMD_BONECOUNT 0x38771d81
#define CONFIGURE_CMD_FIXBONE 0xf729adc8
#define CONFIGURE_CMD_MOVABLE 0xab8cf912
#define CONFIGURE_CMD_HOST 0x431fac89

union configure_u {
	char raw[128];
	struct {
		size_t length;
		uint32_t command;
		union {
			byte data[120];
			struct {	// wifi settings
				char ssid[32];
				char passphrase[32];
			};
			struct {
				char root_serial[20];
				uint8_t reserved_0;
				uint8_t bone_index;
				uint8_t address;
				uint8_t tracker_index;
				Vector3<float> bone;
				Quaternion rotation;
			};
			struct {
				uint8_t fix_bone_count;
				uint8_t movable_count;
				uint16_t reserved_1;
			};
			struct {
				uint8_t ip0;
				uint8_t ip1;
				uint8_t ip2;
				uint8_t ip3;
				uint16_t port;
				uint16_t reserved_2;
			};
		};
	};
};

#define CONFIGURE_DONE 0x7921a8ca

static void
uart_configure_task(void* arg) {
	configure_u cmd;
	bool wifi_configured = false;
	bool bone_configured = false;
	bool host_configured = false;

	bool interpriter = true;

	char key_fix[5] = "fix0";
	char key_mov[5] = "mov0";

	uint32_t ip;

	while (true) {
		// この辺の M5.Lcd.print はデバッグ用なので消す
		vTaskDelay(100);
		if (interpriter) printf(">\n");
		size_t len = fread(cmd.raw, 1, 128, stdin);

		interpriter = len > 0;
		if (len != cmd.length) continue;
		switch (cmd.command) {
			case CONFIGURE_CMD_WIFI:
				M5.Lcd.printf(" configure wifi: %s, %s\n", cmd.ssid, cmd.passphrase);
				size_t a;
				a = pref.putString("ssid", cmd.ssid);
				M5.Lcd.printf("\n write ssid %d", a);
				a = pref.putString("pass", cmd.passphrase);
				M5.Lcd.printf("\n write pass %d", a);

				wifi_configured = true;
				break;
			case CONFIGURE_CMD_REBOOT:
				pref.end();
				ESP.restart();
				break;
			case CONFIGURE_CMD_BONECOUNT:
				M5.Lcd.printf(" configure bone count: %d, %d\n", cmd.fix_bone_count, cmd.movable_count);
				if (cmd.fix_bone_count > 8) cmd.fix_bone_count = 8;
				if (cmd.movable_count > 8) cmd.movable_count = 8;
				pref.putChar("fix", cmd.fix_bone_count);
				pref.putChar("mov", cmd.movable_count);

				bone_configured = true;
				break;
			case CONFIGURE_CMD_FIXBONE:
				M5.Lcd.printf(" configure Fix bone: %d\n", cmd.bone_index);
				if (cmd.bone_index > 8) continue;
				key_fix[3] = '0' + cmd.bone_index;
				pref.putBytes(key_fix, cmd.data, 52);
				break;
			case CONFIGURE_CMD_MOVABLE:
				M5.Lcd.printf(" configure Mov bone: %d\n", cmd.bone_index);
				if (cmd.bone_index > 8) continue;
				key_mov[3] = '0' + cmd.bone_index;
				pref.putBytes(key_mov, cmd.data, 52);
				break;
			case CONFIGURE_CMD_HOST:
				M5.Lcd.printf(" configure host: %d.%d.%d.%d:%d", cmd.ip0, cmd.ip1, cmd.ip2, cmd.ip3, cmd.port);
				ip = cmd.ip0;
				ip <<= 8;
				ip |= cmd.ip1;
				ip <<= 8;
				ip |= cmd.ip2;
				ip <<= 8;
				ip |= cmd.ip3;
				pref.putInt("host_ip", ip);
				pref.putShort("host_port", cmd.port);

				host_configured = true;
				break;
		}

		if (wifi_configured && bone_configured && host_configured) {
			M5.Lcd.println("initialized");
			pref.putInt("version", CONFIGURE_DONE);
			wifi_configured = false;
			bone_configured = false;
			host_configured = false;
		}
	}
}

bool main_loop_start;

void setup() {
	M5.begin(true, false, false, true);
	pref.begin("JTracker", false);

	xTaskCreate(uart_configure_task, "configure_task", 1024 * 4, nullptr, 10, nullptr);

	if (pref.getInt("version", 0) != CONFIGURE_DONE) {
		main_loop_start = false;
		M5.Lcd.print("Please configure wifi and bone settings first.");
		return;
	}
	main_loop_start = true;

	char ssid[32], passphrase[32];
	pref.getString("ssid", ssid, 31);
	pref.getString("pass", passphrase, 31);
	ssid[31] = passphrase[31] = '\0';

	/*
	// I2C 接続アドレス一覧
	for (int i = 0; i < 127; i++) {
		Wire.beginTransmission(i);
		who[i] = Wire.endTransmission();
	}

	for (int y = 0; y < 8; y++) {
		M5.Lcd.setCursor(0, 140 + y * 10);
		for (int i = 0; i < 16; i++) {
			M5.Lcd.printf("%2x ", who[i + y * 16]);
		}
	}
	*/

	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, passphrase);
	M5.Lcd.setCursor(5, 2);
	M5.Lcd.print("WiFi connecting");
	bool dot = true;
	while (WiFi.status() != WL_CONNECTED) {
		delay(300);
		M5.Lcd.setCursor(5 + 6 * 16, 2);
		M5.Lcd.print((dot = !dot) ? "." : " ");
	}

	uint32_t ip = pref.getInt("host_ip", 0x00000000);
	vmt_host[0] = ip >> 24;
	vmt_host[1] = (ip >> 16) & 0xff;
	vmt_host[2] = (ip >> 8) & 0xff;
	vmt_host[3] = ip & 0xff;
	vmt_port	  = pref.getShort("host_port", 39570);

	M5.Lcd.setCursor(5, 2);
	M5.Lcd.print("WiFi connected, ");
	M5.Lcd.print(WiFi.localIP().toString());
	M5.Lcd.printf("->%d.%d.%d.%d", vmt_host[0], vmt_host[1], vmt_host[2], vmt_host[3]);

	osc	    = new OscClient(vmt_host, vmt_port);
	osc_args = {nullptr,			  // Serial
			  1.0f, 0.0f, 0.0f, 0.0f,  // qw, qz, qy, qx
			  0.0f, 1.0f, 0.0f,		  // z, y, x
			  0.0f, 0, 0};			  // time, enable, index

	fix_bone_count = pref.getChar("fix", 0);
	char key[5]	= "fix0";
	for (int i = 0; i < fix_bone_count; i++) {
		Joint_s* j = fix_bone + i;
		pref.getBytes(key, j, sizeof(JointConfigure));
		j->calibrate	  = {0.0f, 0.0f, 0.0f, 1.0f};
		j->xy_correction = {0.0f, 0.0f, 0.0f, 1.0f};
		key[3]++;
	}

	movable_count = pref.getChar("mov", 0);
	key[0] = 'm', key[1] = 'o', key[2] = 'v', key[3] = '0';
	for (int i = 0; i < movable_count; i++) {
		Joint_s* j = movable + i;
		pref.getBytes(key, j, sizeof(JointConfigure));
		j->calibrate	  = {0.0f, 0.0f, 0.0f, 1.0f};
		j->xy_correction = {0.0f, 0.0f, 0.0f, 1.0f};
		key[3]++;
	}

	printBone();
}

uint8_t cmd[1];

void loop() {
	if (!main_loop_start) {
		delay(1000);
		return;
	}
	int py = 25;

	if (osc_args.enable && fix_send) {
		osc_args.enable = false;
		for (int i = 0; i < fix_bone_count; i++) {
			Joint_s* j	 = fix_bone + i;
			osc_args.serial = j->root_serial;
			osc_args.index	 = j->tracker_index;
			osc_args.set(j->rotation, j->rotation * j->bone);
			osc->send_joint(&osc_args);
		}
		osc_args.enable = true;
	}

	cmd[0] = COMMAND_GET_QUATERNION;
	for (int i = 0; i < movable_count; i++) {
		py += 11;

		Joint_s* j = movable + i;

		osc_args.serial = j->root_serial;
		osc_args.index	 = j->tracker_index;

		i2c_err_t err = Wire.writeTransmission(j->address, cmd, 1, true);
		delayMicroseconds(15);
		i2c_err_t err2 = Wire.readTransmission(j->address, data.raw, sizeof(data_u));

		if (err != ESP_OK || err2 != ESP_OK) continue;
		if (data.header != SYNC_HEADER || data.footer != SYNC_FOOTER) continue;

		M5.Lcd.setCursor(0, py);
		M5.Lcd.printf("%2d [%d, %d] %3.3f, %3.3f, %3.3f, %3.3f      \n", j->address, err, err2, data.ahrs.x, data.ahrs.y, data.ahrs.z, data.ahrs.w);

		j->rotation = data.ahrs;

		if (osc_args.enable) {
			Quaternion rot	    = j->rotation * j->calibrate;
			Vector3<float> pos = rot * j->bone;
			osc_args.set(j->xy_correction * rot, j->xy_correction * pos);
			osc->send_follow(&osc_args);
		}
	}

	M5.update();
	if (M5.BtnA.wasPressed()) {
		// 原点設定（キャリブレーション）
		for (int i = 0; i < movable_count; i++) {
			Joint_s* j	  = movable + i;
			j->calibrate	  = j->rotation.inverse();
			j->xy_correction = Quaternion::identify();
		}
	}

	if (M5.BtnB.wasPressed()) {
		// XY平面回転補正
		for (int i = 0; i < movable_count; i++) {
			Joint_s* j	  = movable + i;
			Vector3<float> b = (j->rotation * j->calibrate) * j->bone;
			float _05_theta  = (atan2(b.y, b.x) + 3.1415926535897932384626433832795f * 0.5f) * 0.5f;
			j->xy_correction = Quaternion::xyzw(0.0f, 0.0f, cosf(_05_theta), sinf(_05_theta));
		}
	}

	if (M5.BtnC.wasPressed()) {
		osc_args.enable = 1 - osc_args.enable;
		M5.Lcd.setCursor(190, 18);
		M5.Lcd.print(osc_args.enable ? "RUNNING" : "STOP   ");

		if (!osc_args.enable) {
			osc_args.serial = nullptr;
			osc_args.set({0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f});
			for (int i = 0; i < fix_bone_count; i++) {
				osc_args.index = fix_bone[i].tracker_index;
				osc->send_joint(&osc_args);
			}
			for (int i = 0; i < movable_count; i++) {
				osc_args.index = movable[i].tracker_index;
				osc->send_follow(&osc_args);
			}
		}
	}

	delay(1);
}
