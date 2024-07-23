#pragma once

#include "tracker/tracker_configuration.h"
#include "tracker/tracker_imu.h"
#include "tracker/tracker_info.h"
#include "tracker/tracker_netstat.h"
#include "tracker/tracker_statistic.h"
#include "tracker/tracker_status.h"

namespace dkvr {

	class Tracker
	{
	public:
		Tracker(unsigned long address);

		void Reset();

		// tracker information
		unsigned long address() const { return info_.address; }
		std::string name() const { return info_.name; }

		void set_name(std::string name) { info_.name = name; }

		// connection status
		ConnectionStatus connection_status() const { return info_.connection; }
		bool IsDisconnected() const { return info_.connection == ConnectionStatus::Disconnected; }
		bool IsHandshaked() const { return info_.connection == ConnectionStatus::Handshaked; }
		bool IsConnected() const { return info_.connection == ConnectionStatus::Connected; }

		void SetDisconnected() { info_.connection = ConnectionStatus::Disconnected; }
		void SetHandshaked() { info_.connection = ConnectionStatus::Handshaked; }
		void SetConnected() { info_.connection = ConnectionStatus::Connected; }

		// network statistics
		uint32_t send_sequence_num() { return netstat_.send_sequence_num++; }
		uint32_t recv_sequence_num() const { return netstat_.recv_sequence_num; }
		std::chrono::steady_clock::time_point last_heartbeat_sent() const { return netstat_.last_heartbeat_sent; }
		std::chrono::steady_clock::time_point last_heartbeat_recv() const { return netstat_.last_heartbeat_recv; }
		std::chrono::steady_clock::time_point last_ping_sent() const { return netstat_.last_ping_sent; }
		long long rtt() const { return netstat_.rtt.count(); }

		void set_recv_sequence_num(uint32_t seq) { netstat_.recv_sequence_num = seq; }
		void UpdateHeartbeatSent() { netstat_.last_heartbeat_sent = std::chrono::steady_clock::now(); }
		void UpdateHeartbeatRecv() { netstat_.last_heartbeat_recv = std::chrono::steady_clock::now(); }
		void UpdatePingSent() { netstat_.last_ping_sent = std::chrono::steady_clock::now(); }
		void UpdateRtt();

		// configuration
		uint8_t behavior() const { return behavior_.Encode(); }
		bool active() const { return behavior_.active; }
		bool raw() const { return behavior_.raw; }
		bool led() const { return behavior_.led; }
		CalibrationMatrix calibration() const { return calib_; }
		const CalibrationMatrix& calibration_ref() const { return calib_; }

		void set_behavior(uint8_t behavior) { behavior_.Decode(behavior); validator_.InvalidateBehavior(); }
		void set_active(bool active) { behavior_.active = active; validator_.InvalidateBehavior(); }
		void set_raw(bool raw) { behavior_.raw = raw; validator_.InvalidateBehavior(); }
		void set_led(bool led) { behavior_.led = led; validator_.InvalidateBehavior(); }
		void set_calibration(CalibrationMatrix calib) { calib_ = calib; validator_.InvalidateCalibration(); }

		// validator
		std::vector<ConfigurationKey> GetEveryInvalid() const { return validator_.GetEveryInvalid(); }
		bool IsAllValid() const { return validator_.IsAllValid(); }
		bool IsValid(ConfigurationKey key) const { return validator_.IsValid(key); }

		void InvalidateAll() { validator_.InvalidateAll(); }
		void Validate(ConfigurationKey key) { validator_.Validate(key); }

		// IMU readings
		Quaternion quaternion() const { return readings_.quat; }
		Vector3 gyro() const { return readings_.gyr; }
		Vector3 accel() const { return readings_.acc; }
		Vector3 mag() const { return readings_.mag; }
		IMUReadings imu_readings() const { return readings_; }
		bool IsImuReadingsUpdated() { bool temp = imu_readings_updated_; imu_readings_updated_ = false; return temp; }

		void set_quaternion(Quaternion quat) { readings_.quat = quat; imu_readings_updated_ = true; }
		void set_imu_readings(Vector3 gyro, Vector3 accel, Vector3 mag);

		// tracker statistic
		uint8_t execution_time() const { return statistic_.execution_time; }

		void set_tracker_statistic(TrackerStatistic statistic) { statistic_ = statistic; }

		// tracker status
		uint8_t init_result() const { return status_.init_result; }
		uint8_t last_err() const { return status_.last_err; }
		uint8_t battery_perc() const { return status_.battery_perc; }

		void set_tracker_status(TrackerStatus status) { status_ = status; }

		// update request indicator
		void RequestStatisticUpdate() { statistic_update_required_ = true; }
		void RequestStatusUpdate() { status_update_required_ = true; }
		void RequestLocate() { locate_required_ = true; }
		void RequestMagRefRecalc() { mag_ref_recalc_required_ = true; }

		bool IsStatisticUpdateRequired() { bool temp = statistic_update_required_; statistic_update_required_ = false; return temp; }
		bool IsStatusUpdateRequired() { bool temp = status_update_required_; status_update_required_ = false; return temp; }
		bool IsLocateRequired() { bool temp = locate_required_; locate_required_ = false; return temp; }
		bool IsMagneticRefRecalRequired() { bool temp = mag_ref_recalc_required_; mag_ref_recalc_required_ = false; return temp; }
		
	private:
		TrackerInformation info_;
		TrackerNetworkStatistics netstat_;
		TrackerBehavior behavior_;
		CalibrationMatrix calib_;
		ConfigurationValidator validator_;
		IMUReadings readings_;
		TrackerStatistic statistic_;
		TrackerStatus status_;
		bool imu_readings_updated_;
		bool statistic_update_required_;
		bool status_update_required_;
		bool locate_required_;
		bool mag_ref_recalc_required_;
	};

}	// namespace dkvr