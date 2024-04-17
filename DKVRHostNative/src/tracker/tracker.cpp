#include "tracker/tracker.h"

namespace dkvr {

	Tracker::Tracker(unsigned long address) :
		info_{ .address = address }, connection_(ConnectionStatus::Disconnected), netstat_{}, behavior_{}, calib_{}, validator_(), readings_{}, status_{}
	{
		behavior_.Reset();
	}

	void Tracker::Reset()
	{
		info_.connection = ConnectionStatus::Disconnected;
		netstat_ = TrackerNetworkStatistics{ 0, 0, };
		validator_.InvalidateAll();
		readings_ = IMUReadings{};
		status_ = TrackerStatus{};
	}

	void Tracker::UpdateRtt()
	{
		using namespace std::chrono;
		nanoseconds rtt = steady_clock::now() - netstat_.last_ping_sent;
		netstat_.rtt = duration_cast<milliseconds>(rtt);
	}

	void Tracker::InvalidateCalibration()
	{
		validator_.Invalidate(ConfigurationKey::CalibrationGr);
		validator_.Invalidate(ConfigurationKey::CalibrationAc);
		validator_.Invalidate(ConfigurationKey::CalibrationMg);
	}

}	// namespace dkvr