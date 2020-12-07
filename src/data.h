#pragma once

#include <stdint.h>

#include "Vector3.h"

union data_u {
	uint8_t raw[24];
	struct {
		uint32_t header;
		Quaternion ahrs;
		uint32_t footer;
	};
};

#define COMMAND_GET_QUATERNION ((uint8_t)0x23)
// #define COMMAND_SET_NEUTRAL_QUATERNION ((uint8_t)0x63)
// #define COMMAND_SET_Z_DIRECTION ((uint8_t)0x67)
// #define COMMAND_START_GYRO_CALIBRATION ((uint8_t)0xad)

#define SYNC_HEADER ((uint32_t)0x01020304)
#define SYNC_FOOTER ((uint32_t)0xa7f32249)
