#pragma once

#include <type_traits>

namespace dkvr {
	
	struct Vector3f
	{
		float data[3];
		float& operator[] (int i) { return data[i]; }
		float operator[] (int i) const { return data[i]; }

		float x() const { return data[0]; }
		float y() const { return data[1]; }
		float z() const { return data[2]; }
		float& x() { return data[0]; }
		float& y() { return data[1]; }
		float& z() { return data[2]; }
	};

	struct Quaternionf
	{
		float data[4];	// sequence of w, x, y, z
		float& operator[] (int i) { return data[i]; }
		float operator[] (int i) const { return data[i]; }

		float w() const { return data[0]; }
		float x() const { return data[1]; }
		float y() const { return data[2]; }
		float z() const { return data[3]; }
		float& w() { return data[0]; }
		float& x() { return data[1]; }
		float& y() { return data[2]; }
		float& z() { return data[3]; }
	};

	struct RawDataSet
	{
		Vector3f gyr, acc, mag;
	};

	struct NominalDataSet
	{
		Quaternionf orientation;
		Vector3f linear_acceleration;
		Vector3f magnetic_disturbance;
	};

	class TrackerData
	{
	public:
		bool IsRawUpdated() { bool temp = raw_updated_; raw_updated_ = false; return temp; }
		bool IsNominalUpdated() { bool temp = nominal_updated_; nominal_updated_ = false; return temp; }

		const RawDataSet& raw() const { return raw_; }
		const NominalDataSet& nominal() const { return nominal_; }

		void set_raw(RawDataSet raw) { raw_ = raw; raw_updated_ = true; }
		void set_nominal(NominalDataSet nominal) { nominal_ = nominal; nominal_updated_; }

	private:
		RawDataSet raw_;
		NominalDataSet nominal_;

		bool raw_updated_;
		bool nominal_updated_;
	};

	static_assert(sizeof Vector3f == sizeof(float) * 3);
	static_assert(sizeof Quaternionf == sizeof(float) * 4);

	static_assert(std::is_trivial_v<RawDataSet>);
	static_assert(std::is_standard_layout_v<RawDataSet>);
	static_assert(offsetof(RawDataSet, RawDataSet::gyr) == 0);
	static_assert(offsetof(RawDataSet, RawDataSet::acc) == sizeof Vector3f);
	static_assert(offsetof(RawDataSet, RawDataSet::mag) == sizeof Vector3f * 2);

	static_assert(std::is_trivial_v<NominalDataSet>);
	static_assert(std::is_standard_layout_v<NominalDataSet>);
	static_assert(offsetof(NominalDataSet, NominalDataSet::orientation) == 0);
	static_assert(offsetof(NominalDataSet, NominalDataSet::linear_acceleration) == sizeof Quaternionf);
	static_assert(offsetof(NominalDataSet, NominalDataSet::magnetic_disturbance) == sizeof Quaternionf + sizeof Vector3f);

}	// namespace dkvr