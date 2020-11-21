#pragma once

#include <WiFiUdp.h>
#include <stdint.h>
#include <stdlib.h>

#include "Vector3.h"

struct OscArgument_t {
	float qw;
	float qz;
	float qy;
	float qx;
	float z;
	float y;
	float x;
	float time;
	int32_t enable;
	int32_t index;

	void set(Quaternion rotation, Vector3<float> position) {
		this->qw = rotation.w;
		this->qz = rotation.z;
		this->qy = rotation.y;
		this->qx = rotation.x;

		this->z = position.z;
		this->y = position.y;
		this->x = position.x;
	}
};

union OscArgment_u {
	uint8_t raw[40];
	OscArgument_t data;
};

class OscClient {
    public:
	OscClient(const uint8_t* address, uint16_t port);
	void send(OscArgment_u* arguments);
	void send(OscArgument_t* arguments);
	void reconnect();

    private:
	WiFiUDP udp;
	IPAddress * address;
	int port;
//	uint8_t buffer[68] = "/VMT/Raw/Driver\0,iiffffffff\0";

public:
	uint8_t buffer[68] = {
		'/', 'V', 'M', 'T', '/', 'R', 'a', 'w', '/', 'D', 'r', 'i', 'v', 'e', 'r', '\0', // packet address
		',', 'i', 'i', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f', '\0'                 // packet tag
	};

	private:

	static const int local_port		= 62333;
	static const size_t data_start_index = 28;
};

OscClient::OscClient(const uint8_t* address, uint16_t port) {
	this->address = new IPAddress(address);
	this->port = port;

	udp.begin(local_port);
}

inline void OscClient::send(OscArgument_t* arguments) { send((OscArgment_u*)arguments); }

void OscClient::send(OscArgment_u* arguments) {
	// OSCプロトコルのパケット
	// https://github.com/gpsnmeajp/VirtualMotionTracker/blob/master/docs/note.md
	// http://veritas-vos-liberabit.com/trans/OSC/OSC-spec-1_0.html#:~:text=OSC%E3%83%91%E3%82%B1%E3%83%83%E3%83%88%E3%81%AF%E3%80%81%E3%83%90%E3%82%A4%E3%83%8A%E3%83%AA%E3%83%87%E3%83%BC%E3%82%BF,%E9%85%8D%E4%BF%A1%E3%81%99%E3%82%8B%E8%B2%AC%E5%8B%99%E3%82%92%E8%B2%A0%E3%81%86%E3%80%82
	// アドレスパターン: /VMT/Raw/Driver
	// タグ文字列: ,iiffffffff
	// OSC引数: （バイナリ列）トラッカー番号, 有効(1), 時間補正(0f), x, y, z, qx, qy, qz, qw
	// 各データ間にnull (\0) を1byte挿入

	for (int i = 0; i < sizeof(OscArgment_u); i++) {
		buffer[data_start_index + i] = arguments->raw[sizeof(OscArgment_u) - 1 - i];
	}

	udp.beginPacket(*address, port);
	udp.write(buffer, 68);
	udp.endPacket();
}
