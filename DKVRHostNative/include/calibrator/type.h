#pragma once

#include <algorithm>
#include <vector>

#include "Eigen/Core"

namespace dkvr
{
	enum class SampleType
	{
		ZNegative,
		ZPositive,
		YNegative,
		YPositive,
		XNegative,
		XPositive,
		Rotational
	};

	/// <summary>
	/// <para>Represent an affine transformation matrix.</para>
	/// <para>Member named 'transform' is actually a linear-map and
	///       'transform' + 'offset' are true transformation matrix.</para>
	/// </summary>
	struct CalibrationMatrix
	{
		Eigen::Matrix3f transform;
		Eigen::Vector3f offset;

		void CopyTo(float dst[12]) const
		{
			std::copy_n(transform.data(), 9, dst);
			std::copy_n(offset.data(), 3, dst + 9);
		}
	};

}	// namespace dkvr