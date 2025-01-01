#pragma once

#include "tracker/tracker_configuration.h"
#include "tracker/tracker_data.h"
#include "tracker/tracker_info.h"
#include "tracker/tracker_netstat.h"
#include "tracker/tracker_statistic.h"
#include "tracker/tracker_status.h"

namespace dkvr {

    class Tracker
    {
    public:
        __pragma(dkvr_export) enum class ConnectionStatus
        {
            Disconnected,
            Handshaked,
            Connected
        };

    private:
        using ConfigurationKey = TrackerConfiguration::ConfigurationKey;

    public:
        Tracker(unsigned long address) :
            address_(address), name_("unnamed tracker"), connection_(ConnectionStatus::Disconnected),
            netstat_{},
            status_{},
            statistic_{},
            config_{},
            data_{}
        {
            config_.Reset();
        }

        void Reset()
        {
            connection_ = ConnectionStatus::Disconnected;
            netstat_ = TrackerNetworkStatistics{ 0, 0, };
            status_ = TrackerStatus{};
            statistic_ = TrackerStatistic{};
            data_ = TrackerData{};
            config_.InvalidateAll();
            ResetRequestIndicator();
        }

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
		void UpdateRtt()
		{
			using namespace std::chrono;
			nanoseconds rtt = steady_clock::now() - netstat_.last_ping_sent;
			netstat_.rtt = duration_cast<milliseconds>(rtt);
		}

		// tracker status
		uint8_t init_result() const { return status_.init_result; }
		uint8_t battery_perc() const { return status_.battery_level; }

		void set_tracker_status(TrackerStatus status) { status_ = status; }

		// tracker statistic
		uint8_t execution_time() const { return statistic_.execution_time; }
		uint8_t interrupt_miss_rate() const { return statistic_.interrupt_miss_rate; }
		uint8_t imu_miss_rate() const { return statistic_.imu_miss_rate; }

		void set_tracker_statistic(TrackerStatistic statistic) { statistic_ = statistic; }

		// behavior
		uint8_t behavior() const { return behavior_.Encode(); }
		bool active() const { return behavior_.active; }
		bool raw() const { return behavior_.raw; }
		bool led() const { return behavior_.led; }

        void set_behavior_led(bool on)      { config_.set_led(on); }
        void set_behavior_active(bool on)   { config_.set_active(on); }
        void set_behavior_raw(bool on)      { config_.set_raw(on); }
        void set_behavior_nominal(bool on)  { config_.set_nominal(on); }

		// calibration
		TrackerCalibration calibration() const { return calib_; }
		const float* gyro_transform() const { return calib_.gyr_transform; }
		const float* accel_transform() const { return calib_.acc_transform; }
		const float* mag_transform() const { return calib_.mag_transform; }
		const float* noise_variance() const { return calib_.gyr_noise_var; }
		const TrackerCalibration& calibration_ref() const { return calib_; }

		void set_calibration(TrackerCalibration calib) { calib_ = calib; validator_.InvalidateCalibration(); }

		// validator
		std::vector<ConfigurationKey> GetEveryInvalid() const { return validator_.GetEveryInvalid(); }
		bool IsAllValid() const { return validator_.IsAllValid(); }
		bool IsValid(ConfigurationKey key) const { return validator_.IsValid(key); }

		void InvalidateAll() { validator_.InvalidateAll(); }
		void Validate(ConfigurationKey key) { validator_.Validate(key); }

		// tracker data
		Quaternionf orientation() const { return data_.orientation; }
		RawDataSet raw_data() const { return data_.raw; }
		Vector3f gyro() const { return data_.raw.gyr; }
		Vector3f accel() const { return data_.raw.acc; }
		Vector3f mag() const { return data_.raw.mag; }

		TrackerData tracker_data() const { return data_; }
		bool IsRawDataUpdated() { bool temp = raw_data_updated_; raw_data_updated_ = false; return temp; }
		bool IsOrientationUpdated() { bool temp = orientation_updated_; orientation_updated_ = false; return temp; }

		void set_orientation(Quaternionf quat) { data_.orientation = quat; orientation_updated_ = true; }
		void set_raw_data(RawDataSet raw) { data_.raw = raw; raw_data_updated_ = true; }

		// update request indicator
		void RequestStatisticUpdate() { statistic_update_required_ = true; }
		void RequestStatusUpdate() { status_update_required_ = true; }
		void RequestLocate() { locate_required_ = true; }

		bool IsStatisticUpdateRequired() { bool temp = statistic_update_required_; statistic_update_required_ = false; return temp; }
		bool IsStatusUpdateRequired() { bool temp = status_update_required_; status_update_required_ = false; return temp; }
		bool IsLocateRequired() { bool temp = locate_required_; locate_required_ = false; return temp; }
		
	private:
		TrackerInformation info_;
		TrackerNetworkStatistics netstat_;
		TrackerStatus status_;
		TrackerStatistic statistic_;
		TrackerBehavior behavior_;
		TrackerCalibration calib_;
		ConfigurationValidator validator_;
		TrackerData data_;

		bool raw_data_updated_ = false;
		bool orientation_updated_ = false;

		bool statistic_update_required_ = false;
		bool status_update_required_ = false;
		bool locate_required_ = false;
	};

}	// namespace dkvr