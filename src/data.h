#include <stdint.h>

#include "Vector3.h"

union data_u {
	uint8_t raw[24];
	struct {
		uint32_t header;
		Quaternion q;
		Quaternion rot;
		uint32_t footer;
	};
};
