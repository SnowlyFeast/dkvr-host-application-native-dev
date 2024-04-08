#include "tracker/tracker.h"

namespace dkvr {

	Tracker::Tracker(unsigned long address) :
		info_{ .address = address }, connection_(ConnectionStatus::Disconnected), behavior_{}, calib_{}, netstat_{}, validator_(), readings_{}
	{
	}

	void Tracker::Reset()
	{
		connection_ = ConnectionStatus::Disconnected;
		netstat_ = NetworkStatistics{};
		validator_.InvalidateAll();
		readings_ = IMUReadings{};
	}

	void Tracker::UpdateRtt()
	{
		using namespace std::chrono;
		nanoseconds rtt = steady_clock::now() - netstat_.last_ping_sent;
		netstat_.rtt = duration_cast<milliseconds>(rtt);
	}

	void Tracker::set_gyro_offset(float* offset)
	{
		memcpy_s(calib_.gyro_offset, sizeof(calib_.gyro_offset), offset, sizeof(calib_.gyro_offset));
		InvalidateCalibGr();
	}

	void Tracker::set_accel_mat(float* mat)
	{
		memcpy_s(calib_.accel_mat, sizeof(calib_.accel_mat), mat, sizeof(calib_.accel_mat));
		InvalidateCalibAc();
	}

	void Tracker::set_mag_mat(float* mat)
	{
		memcpy_s(calib_.mag_mat, sizeof(calib_.mag_mat), mat, sizeof(calib_.mag_mat));
		InvalidateCalibMg();
	}

}	// namespace dkvr