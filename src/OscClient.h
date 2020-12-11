#pragma once

#include <WiFiUdp.h>
#include <stdint.h>
#include <stdlib.h>

#include "Vector3.h"

struct VMTJointArgument_t {
	const char * serial;
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
		this->qz = -rotation.y;
		this->qy = rotation.z;
		this->qx = rotation.x;

		this->z = -position.y;
		this->y = position.z;
		this->x = position.x;
	}
};

union VMTJointArgument_u {
	uint8_t raw[sizeof(VMTJointArgument_t)];
	VMTJointArgument_t data;
};

class OscClient {
    public:
	OscClient(const uint8_t* address, uint16_t port);
	size_t send_joint(VMTJointArgument_t* arguments);
	size_t send_follow(VMTJointArgument_t* arguments);
	void reconnect();

    private:
	WiFiUDP udp;
	IPAddress * address;
	int port;

	size_t send(VMTJointArgument_u* arguments, uint8_t * base_buffer);
//	uint8_t buffer[84] = "/VMT/Joint/Driver\0,iiffffffffi\0";
	uint8_t joint_buffer[36 + sizeof(VMTJointArgument_t) + 32] = { // テキストバッファとして32バイト
	//	00   01   02   03     04   05   06   07     08   09   0a   0b     0c   0d   0e   0f
		'/', 'V', 'M', 'T',   '/', 'J', 'o', 'i',   'n', 't', '/', 'D',   'r', 'i', 'v', 'e', // 16 len
		'r', '\0','\0','\0',  // packet address, padding for 4byte
		                      ',', 'i', 'i', 'f',   'f', 'f', 'f', 'f',   'f', 'f', 'f', 's', // 32 len
	     '\0','\0','\0','\0',   // packet tag, padding for 4byte                                 // 36 len
	};
	uint8_t follow_buffer[36 + sizeof(VMTJointArgument_t) + 32] = { // テキストバッファとして32バイト
	//	00   01   02   03     04   05   06   07     08   09   0a   0b     0c   0d   0e   0f
		'/', 'V', 'M', 'T',   '/', 'F', 'o', 'l',   'l', 'o', 'w', '/', 'D',   'r', 'i', 'v', // 16 len
		'e', 'r', '\0','\0',  // packet address, padding for 4byte
		                      ',', 'i', 'i', 'f',   'f', 'f', 'f', 'f',   'f', 'f', 'f', 's', // 32 len
	     '\0','\0','\0','\0',   // packet tag, padding for 4byte                                 // 36 len
	};

	static const int local_port		= 62333;
	static const size_t data_start_index = 36;
};

inline size_t OscClient::send_joint(VMTJointArgument_t* arguments) { return send((VMTJointArgument_u*)arguments, joint_buffer); }
inline size_t OscClient::send_follow(VMTJointArgument_t* arguments) { return send((VMTJointArgument_u*)arguments, follow_buffer); }


OscClient::OscClient(const uint8_t* address, uint16_t port) {
	this->address = new IPAddress(address);
	this->port = port;

	udp.begin(local_port);
}

size_t OscClient::send(VMTJointArgument_u* arguments, uint8_t * buffer) {
	// OSCプロトコルのパケット
	// https://github.com/gpsnmeajp/VirtualMotionTracker/blob/master/docs/note.md
	// http://veritas-vos-liberabit.com/trans/OSC/OSC-spec-1_0.html#:~:text=OSC%E3%83%91%E3%82%B1%E3%83%83%E3%83%88%E3%81%AF%E3%80%81%E3%83%90%E3%82%A4%E3%83%8A%E3%83%AA%E3%83%87%E3%83%BC%E3%82%BF,%E9%85%8D%E4%BF%A1%E3%81%99%E3%82%8B%E8%B2%AC%E5%8B%99%E3%82%92%E8%B2%A0%E3%81%86%E3%80%82
	// アドレスパターン: /VMT/Joint/Driver
	// タグ文字列: ,iiffffffffi
	// OSC引数: （バイナリ列）トラッカー番号, 有効(1), 時間補正(0f), x, y, z, qx, qy, qz, qw
	// 各データ間にnull (\0) を1byte挿入

	size_t count = data_start_index + sizeof(VMTJointArgument_u) - 4;
	for (int i = 0; i < sizeof(VMTJointArgument_u) - 4; i++) { // 文字列は特殊処理する
		buffer[data_start_index + i] = arguments->raw[sizeof(VMTJointArgument_u) - 1 - i];
	}

	size_t serial_length = arguments->data.serial ? strlen(arguments->data.serial) : 0;
	size_t padding_length = 4 - (serial_length & 0b11);

	count += serial_length;
	for (int i=0; i<serial_length; i++) {
		buffer[data_start_index + i + sizeof(VMTJointArgument_u) - 4] = arguments->data.serial[i];
	}

	for(int i=0; i<padding_length; i++) {
		buffer[count++] = '\0';
	}
	
	udp.beginPacket(*address, port);
	udp.write(buffer, count);
	udp.endPacket();

	return count;
}
