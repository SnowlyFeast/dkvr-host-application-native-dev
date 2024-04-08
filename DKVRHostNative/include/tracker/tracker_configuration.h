#pragma once

#include <cstdint>

namespace dkvr {

	struct Behavior
	{
		uint8_t Encode() const;
		void Decode(uint8_t behavior);

		bool active = false;
		bool raw = false;
		bool led = true;
	};

	struct Calibration
	{
		float gyro_offset[3];
		float accel_mat[12];
		float mag_mat[12];
	};

}	// namespace dkvr