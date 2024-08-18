#pragma once

#include <type_traits>

namespace dkvr {
	
	struct Vector3f
	{
		float data[3];
		float& operator[] (int i) { return data[i]; }
		float operator[] (int i) const { return data[i]; }
	};

	/// <summary>
	/// Sequence of w, x, y and z
	/// </summary>
	struct Quaternionf
	{
		float data[4];
		float& operator[] (int i) { return data[i]; }
		float operator[] (int i) const { return data[i]; }
	};

	struct RawDataSet
	{
		Vector3f gyr, acc, mag;
	};

	struct TrackerData
	{
		Quaternionf orientation;
		RawDataSet raw;
	};

	static_assert(std::is_trivial_v<RawDataSet>);
	static_assert(std::is_standard_layout_v<RawDataSet>);
	static_assert(std::is_trivial_v<TrackerData>);
	static_assert(std::is_standard_layout_v<TrackerData>);

}	// namespace dkvr