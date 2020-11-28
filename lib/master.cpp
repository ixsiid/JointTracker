#include <M5Stack.h>
#include <WiFi.h>
#include <Wire.h>

#include "Config.h"
#include "OscClient.h"
#include "command.h"
#include "data.h"

uint8_t who[128];

OscClient* osc;
VMTJointArgument_t osc_args;

data_u data;

Vector3<float> left_hip;
Vector3<float> right_hip;
Vector3<float> left_nee;
Vector3<float> right_nee;
Vector3<float> left_ankle;
Vector3<float> right_ankle;

struct Joint_s {
	const char* root_serial;
	int32_t mode;
	uint8_t address;
	uint8_t tracker_index;
	Vector3<float> boneLength;
	Quaternion last_position;
	Quaternion last_rotation;
};

struct JointChain_s {
	size_t count;
	Joint_s* joints;
};

struct JointList_s {
	size_t count;
	JointChain_s* chains;
};

const float sq2 = sqrtf(0.5f);

Joint_s right_leg[3] = {
    {Config::root_tracker_serial, 0, 0, 12, {0.11f, -0.14f, -0.08f}, {0.0f, 1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},  // hip
    {"VMT_12", 1, 13, 13, {0.00f, -0.38f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},				  // nee
    {"VMT_13", 1, 14, 14, {0.00f, -0.48f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},				  // ankle
};

Joint_s left_leg[3] = {
    {Config::root_tracker_serial, 0, 0, 16, {-0.11f, -0.14f, -0.08f}, {0.0f, 1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},  // hip
    {"VMT_16", 1, 17, 17, {0.00f, -0.38f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},				   // nee
    {"VMT_17", 1, 18, 18, {0.00f, -0.48f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f, 1.0f}},				   // ankle
};

JointChain_s right_leg_chain = {3, right_leg};
JointChain_s left_leg_chain  = {3, left_leg};
JointChain_s chains[2]	    = {right_leg_chain, left_leg_chain};

JointList_s joints = {2, chains};

bool send = false;

void setup() {
	M5.begin(true, false, false, true);

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
	WiFi.begin(Config::ssid, Config::passphrase);
	M5.Lcd.setCursor(2, 12);
	M5.Lcd.print("WiFi connecting");
	while (WiFi.status() != WL_CONNECTED) {
		delay(300);
		M5.Lcd.print(".");
	}
	M5.Lcd.print("Done");

	osc	    = new OscClient(Config::vmt_host, Config::vmt_port);
	osc_args = {0, 0,				  // Mode, Serial
			  1.0f, 0.0f, 0.0f, 0.0f,  // qw, qz, qy, qx
			  0.0f, 1.0f, 0.0f,		  // z, y, x
			  0.0f, 1, 0};			  // time, enable, index
}

uint8_t cmd[1] = {0};

void loop() {
	int py = 25;

	for (int k = 0; k < joints.count; k++) {
		JointChain_s* j = joints.chains + k;

		for (int i = 0; i < j->count; i++) {
			Joint_s* joint = &(j->joints[i]);

			int addr		= joint->address;
			osc_args.index = joint->tracker_index;

			if (addr) {
				i2c_err_t err = Wire.writeTransmission(addr, &(*cmd = COMMAND_GET_QUATERNION), 1, true);
				delayMicroseconds(15);
				i2c_err_t err2 = Wire.readTransmission(addr, data.raw, sizeof(data_u));

				if (data.header == SYNC_HEADER && data.footer == SYNC_FOOTER) {
					M5.Lcd.setCursor(0, py);
					M5.Lcd.printf("%2d [%d, %d] %3.3f, %3.3f, %3.3f, %3.3f      \n", i, err, err2, data.q.x, data.q.y, data.q.z, data.q.w);

					joint->last_position = data.q;
					joint->last_rotation = data.rot;
				}
			}

			Vector3<float> pos = joint->last_position * joint->boneLength;
			Quaternion rot	    = joint->last_position * joint->last_rotation;

			if (send) {
				osc_args.mode	 = joint->mode;
				osc_args.serial = joint->root_serial;
				osc_args.set(rot, pos);
				size_t len = osc->send(&osc_args);

				if (false) {
					M5.Lcd.setCursor(0, 40);
					M5.Lcd.fillRect(0, 40, 320, 100, TFT_BLACK);
					M5.Lcd.printf("len: %d\n", len);
					for (int i = 0; i < len; i++) {
						M5.Lcd.printf("%2x ", osc->buffer[i]);
						if ((i & 0b1111) == 0b1111)
							M5.Lcd.print('\n');
						else if ((i & 0b111) == 0b111)
							M5.Lcd.print("  ");
					}
				}
			}

			py += 11;
		}
	}

	uint8_t address[] = {13, 14, 17, 18};

	M5.update();
	if (M5.BtnB.wasPressed()) {
		for (int i = 0; i <4; i++) {
			Wire.flush();
			delay(10);
			Wire.writeTransmission(address[i], &(*cmd = COMMAND_SET_NEUTRAL_QUATERNION), 1, true);
			Wire.flush();
			delay(5);
		}
	}
	if (M5.BtnA.wasPressed()) {
		for (int i = 0; i < 4; i++) {
			Wire.flush();
			delay(10);
			Wire.writeTransmission(address[i], &(*cmd = COMMAND_SET_Z_DIRECTION), 1, true);
			Wire.flush();
			delay(5);
		}
	}

	if (M5.BtnC.wasPressed()) {
		send = 1 - send;
		M5.Lcd.setCursor(190, 0);
		M5.Lcd.printf(send ? "RUNNING" : "STOP   ");

		osc_args.enable = send;
		if (!send) {
			for (int i = 7; i <= 9; i++) {
				osc_args.index	 = i;
				osc_args.serial = nullptr;
				osc_args.enable = 0;
				osc_args.set({0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f});
				osc->send(&osc_args);
			}
		}
	}

	delay(2);
}
