#pragma once

#include <cstdint>

#include "math/vector.h"
#include "math/matrix.h"

namespace dkvr {

	struct TrackerBehavior
	{
		static constexpr uint8_t kInvalid = (0b10000000u);

		static constexpr uint8_t Encode(TrackerBehavior behavior);
		uint8_t Encode() const;
		void Decode(uint8_t behavior);
		void Reset();

		bool active;
		bool raw;
		bool led;
	};

	struct CalibrationMatrix
	{
		void Reset();

		float gyr[12];
		float acc[12];
		float mag[12];
	};

	struct NoiseVariance
	{
		float gyr[3];
		float acc[3];
		float mag[3];
	};

}	// namespace dkvr