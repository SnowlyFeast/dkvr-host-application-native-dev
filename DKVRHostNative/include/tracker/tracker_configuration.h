#pragma once

#include <cstdint>

#include "math/vector.h"
#include "math/matrix.h"

namespace dkvr {

	struct Behavior
	{
		static constexpr uint8_t kInvalid = (0b10000000u);

		constexpr uint8_t Encode() const;
		void Decode(uint8_t behavior);
		void Reset();

		bool active;
		bool raw;
		bool led;
	};

	struct Calibration
	{
		void Reset();

		float gyro_offset[3];
		float accel_mat[12];
		float mag_mat[12];
	};

}	// namespace dkvr