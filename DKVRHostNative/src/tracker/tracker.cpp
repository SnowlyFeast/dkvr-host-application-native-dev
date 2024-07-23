#include "tracker/tracker.h"

namespace dkvr {

	Tracker::Tracker(unsigned long address) :
		info_{ .address = address, .name = "unnamed tracker", .connection = ConnectionStatus::Disconnected },
		netstat_{},
		behavior_{},
		calib_{},
		validator_(),
		readings_{},
		statistic_{},
		status_{},
		imu_readings_updated_(false),
		statistic_update_required_(false),
		status_update_required_(false),
		locate_required_(false),
		mag_ref_recalc_required_(false)
	{
		behavior_.Reset();
		calib_.Reset();
	}

	void Tracker::Reset()
	{
		info_.connection = ConnectionStatus::Disconnected;
		netstat_ = TrackerNetworkStatistics{ 0, 0, };
		validator_.InvalidateAll();
		readings_ = IMUReadings{};
		statistic_ = TrackerStatistic{};
		status_ = TrackerStatus{};
	}

	void Tracker::UpdateRtt()
	{
		using namespace std::chrono;
		nanoseconds rtt = steady_clock::now() - netstat_.last_ping_sent;
		netstat_.rtt = duration_cast<milliseconds>(rtt);
	}

	void Tracker::set_imu_readings(Vector3 gyro, Vector3 accel, Vector3 mag)
	{
		readings_.gyr = gyro;
		readings_.acc = accel;
		readings_.mag = mag;
		imu_readings_updated_ = true;
	}

}	// namespace dkvr