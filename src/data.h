#include <stdint.h>

#include "Vector3.h"

union data_u {
	uint8_t raw[24];
	struct {
		uint32_t header;
		Quaternion q;
		uint32_t footer;
	};
};
