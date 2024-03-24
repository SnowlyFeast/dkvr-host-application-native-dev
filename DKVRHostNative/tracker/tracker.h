#pragma once

#include "tracker_configuration.h"
#include "tracker_config_validator.h"
#include "tracker_imu.h"
#include "tracker_info.h"
#include "tracker_netstat.h"
#include "tracker_status.h"

namespace dkvr {

	class Tracker
	{
	public:
		enum class ConnectionStatus
		{
			Disconnected,
			Handshaked,
			Connected
		};

		Tracker(unsigned long address);

		void Reset();

		// tracker information
		unsigned long address() const { return info_.address; }
		std::string name() const { return info_.name; }

		void set_name(std::string name) { info_.name = name; }

		// connection status
		ConnectionStatus connection_status() const { return connection_; }
		bool IsDisconnected() const { return connection_ == ConnectionStatus::Disconnected; }
		bool IsHandshaked() const { return connection_ == ConnectionStatus::Handshaked; }
		bool IsConnected() const { return connection_ == ConnectionStatus::Connected; }

		void SetDisconnected() { connection_ = ConnectionStatus::Disconnected; }
		void SetHandshaked() { connection_ = ConnectionStatus::Handshaked; }
		void SetConnected() { connection_ = ConnectionStatus::Connected; }

		// network statistics
		uint32_t send_sequence_num() { return netstat_.send_sequence_num++; }
		uint32_t recv_sequence_num() const { return netstat_.recv_sequence_num; }
		std::chrono::steady_clock::time_point last_heartbeat_sent() const { return netstat_.last_heartbeat_sent; }
		std::chrono::steady_clock::time_point last_heartbeat_recv() const { return netstat_.last_heartbeat_recv; }
		std::chrono::steady_clock::time_point last_ping_sent() const { return netstat_.last_ping_sent; }
		std::chrono::milliseconds rtt() const { return netstat_.rtt; }

		//void IncreaseSendSequenceNumber() { netstat_.send_sequence_num++; }
		void set_recv_sequence_num(uint32_t seq) { netstat_.recv_sequence_num = seq; }
		void UpdateHeartbeatSent() { netstat_.last_heartbeat_sent = std::chrono::steady_clock::now(); }
		void UpdateHeartbeatRecv() { netstat_.last_heartbeat_recv = std::chrono::steady_clock::now(); }
		void UpdatePingSent() { netstat_.last_ping_sent = std::chrono::steady_clock::now(); }
		void UpdateRtt();

		// configuration
		uint8_t behavior() const { return behavior_.Encode(); }
		const float* gyro_offset() const { return calib_.gyro_offset; }
		const float* accel_mat() const { return calib_.accel_mat; }
		const float* mag_mat() const { return calib_.mag_mat; }

		void set_behavior(uint8_t behavior) { behavior_.Decode(behavior); InvalidateBehavior(); }
		void set_active(bool active) { behavior_.active = active; InvalidateBehavior(); }
		void set_raw(bool raw) { behavior_.raw = raw; InvalidateBehavior(); }
		void set_led(bool led) { behavior_.led = led; InvalidateBehavior(); }
		void set_gyro_offset(float* offset);
		void set_accel_mat(float* mat);
		void set_mag_mat(float* mat);

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

		void set_quaternion(Quaternion quat) { readings_.quat = quat; }
		void set_gyro(Vector3 gyro) { readings_.gyr = gyro; }
		void set_accel(Vector3 accel) { readings_.acc = accel; }
		void set_mag(Vector3 mag) { readings_.mag = mag; }

		// tracker status
		uint8_t init_result() const { return status_.init_result; }
		uint8_t last_err() const { return status_.last_err; }
		uint8_t battery_perc() const { return status_.battery_perc; }

		void set_tracker_status(TrackerStatus status) { status_ = status; }

		
	private:
		void InvalidateBehavior() { validator_.Invalidate(ConfigurationKey::Behavior); }
		void InvalidateCalibGr() { validator_.Invalidate(ConfigurationKey::CalibrationGr); }
		void InvalidateCalibAc() { validator_.Invalidate(ConfigurationKey::CalibrationAc); }
		void InvalidateCalibMg() { validator_.Invalidate(ConfigurationKey::CalibrationMg); }

		Information info_;
		ConnectionStatus connection_;
		NetworkStatistics netstat_;
		Behavior behavior_;
		Calibration calib_;
		ConfigurationValidator validator_;
		IMUReadings readings_;
		TrackerStatus status_;
		
	};

}	// namespace dkvr