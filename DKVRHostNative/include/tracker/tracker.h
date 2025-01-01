#pragma once

#include "tracker/tracker_configuration.h"
#include "tracker/tracker_data.h"
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
        unsigned long address() const { return address_; }
        std::string name() const { return name_; }

        void set_name(std::string name) { name_ = std::move(name); }

        // connection status
        ConnectionStatus connection_status() const { return connection_; }
        bool IsDisconnected() const { return connection_ == ConnectionStatus::Disconnected; }
        bool IsHandshaked() const   { return connection_ == ConnectionStatus::Handshaked; }
        bool IsConnected() const    { return connection_ == ConnectionStatus::Connected; }

        void SetDisconnected()  { connection_ = ConnectionStatus::Disconnected; }
        void SetHandshaked()    { connection_ = ConnectionStatus::Handshaked; }
        void SetConnected()     { connection_ = ConnectionStatus::Connected; }

        // network statistics
        uint32_t send_sequence_num()        { return netstat_.send_sequence_num++; }
        uint32_t recv_sequence_num() const  { return netstat_.recv_sequence_num; }
        std::chrono::steady_clock::time_point last_heartbeat_sent() const   { return netstat_.last_heartbeat_sent; }
        std::chrono::steady_clock::time_point last_heartbeat_recv() const   { return netstat_.last_heartbeat_recv; }
        std::chrono::steady_clock::time_point last_ping_sent() const        { return netstat_.last_ping_sent; }
        long long rtt() const               { return netstat_.rtt.count(); }

        void set_recv_sequence_num(uint32_t seq){ netstat_.recv_sequence_num = seq; }
        void UpdateHeartbeatSent()  { netstat_.last_heartbeat_sent = std::chrono::steady_clock::now(); }
        void UpdateHeartbeatRecv()  { netstat_.last_heartbeat_recv = std::chrono::steady_clock::now(); }
        void UpdatePingSent()       { netstat_.last_ping_sent = std::chrono::steady_clock::now(); }
        void UpdateRtt()
        {
            using namespace std::chrono;
            nanoseconds rtt = steady_clock::now() - netstat_.last_ping_sent;
            netstat_.rtt = duration_cast<milliseconds>(rtt);
        }

        // tracker status
        uint8_t init_result() const  { return status_.init_result; }
        uint8_t battery_perc() const { return status_.battery_level; }

        TrackerStatus tracker_status() const          { return status_; }
        void set_tracker_status(TrackerStatus status) { status_ = status; }

        // tracker statistic
        uint8_t execution_time() const      { return statistic_.execution_time; }
        uint8_t interrupt_miss_rate() const { return statistic_.interrupt_miss_rate; }
        uint8_t imu_miss_rate() const       { return statistic_.imu_miss_rate; }

        TrackerStatistic tracker_statistic() const             { return statistic_; }
        void set_tracker_statistic(TrackerStatistic statistic) { statistic_ = statistic; }

        // configuration
        bool behavior_led() const       { return config_.behavior().led; }
        bool behavior_active() const    { return config_.behavior().active; }
        bool behavior_raw() const       { return config_.behavior().raw; }
        bool behavior_nominal() const   { return config_.behavior().nominal; }

        void set_behavior_led(bool on)      { config_.set_led(on); }
        void set_behavior_active(bool on)   { config_.set_active(on); }
        void set_behavior_raw(bool on)      { config_.set_raw(on); }
        void set_behavior_nominal(bool on)  { config_.set_nominal(on); }

        uint8_t                   behavior_encoded() const  { return config_.behavior().Encode(); }
        TrackerBehavior           behavior() const          { return config_.behavior(); }
        TrackerCalibration        calibration() const       { return config_.calibration(); }
        const TrackerCalibration& calibration_cref() const  { return config_.calibration(); }

        void set_behavior(uint8_t encoded_behavior)          { config_.set_behavior(TrackerBehavior::Decode(encoded_behavior)); }
        void set_behavior(TrackerBehavior behavior)          { config_.set_behavior(behavior); }
        void set_calibration(TrackerCalibration calibration) { config_.set_calibration(calibration); }

        bool IsAllSynced() const           { return config_.IsAllValid(); }
        bool IsBehaviorSynced() const      { return config_.IsValid(ConfigurationKey::Behavior); }
        bool IsGyrTransformSynced() const  { return config_.IsValid(ConfigurationKey::GyrTransform); }
        bool IsAccTransformSynced() const  { return config_.IsValid(ConfigurationKey::AccTransform); }
        bool IsMagTransformSynced() const  { return config_.IsValid(ConfigurationKey::MagTransform); }
        bool IsNoiseVarianceSynced() const { return config_.IsValid(ConfigurationKey::NoiseVariance); }
        std::vector<ConfigurationKey> GetEveryUnsynced() const { return config_.GetEveryInvalid(); }

        void SetBehaviorSynced()        { config_.Validate(ConfigurationKey::Behavior); }
        void SetGyrTransformSynced()    { config_.Validate(ConfigurationKey::GyrTransform); }
        void SetAccTransformSynced()    { config_.Validate(ConfigurationKey::AccTransform); }
        void SetMagTransformSynced()    { config_.Validate(ConfigurationKey::MagTransform); }
        void SetNoiseVarianceSynced()   { config_.Validate(ConfigurationKey::NoiseVariance); }

        // tracker data
        const RawDataSet& raw_data() const          { return data_.raw(); }
        const NominalDataSet& nominal_data() const  { return data_.nominal(); }
        Vector3f raw_gyro() const                   { return data_.raw().gyr; }
        Vector3f raw_accel() const                  { return data_.raw().acc; }
        Vector3f raw_mag() const                    { return data_.raw().mag; }
        Quaternionf orientation() const             { return data_.nominal().orientation; }
        Vector3f linear_acceleration() const        { return data_.nominal().linear_acceleration; }
        Vector3f magnetic_disturbance() const       { return data_.nominal().magnetic_disturbance; }

        bool IsRawDataUpdated()     { return data_.IsRawUpdated(); }
        bool IsNominalDataUpdated() { return data_.IsNominalUpdated(); }
        
        void set_raw_data(RawDataSet raw)             { data_.set_raw(raw); }
        void set_nominal_data(NominalDataSet nominal) { data_.set_nominal(nominal); }


    // misc request indicator
    public:
        bool IsStatisticUpdateRequired() { bool temp = statistic_update_required_; statistic_update_required_ = false; return temp; }
        bool IsStatusUpdateRequired()    { bool temp = status_update_required_; status_update_required_ = false; return temp; }
        bool IsLocateRequired()          { bool temp = locate_required_; locate_required_ = false; return temp; }

        void RequestStatisticUpdate() { statistic_update_required_ = true; }
        void RequestStatusUpdate()    { status_update_required_ = true; }
        void RequestLocate()          { locate_required_ = true; }

    private:
        bool statistic_update_required_ = false;
        bool status_update_required_ = false;
        bool locate_required_ = false;

        void ResetRequestIndicator()
        {
            statistic_update_required_ = false;
            status_update_required_ = false;
            locate_required_ = false;
        }


    // private member
    private:
        unsigned long address_;
        std::string name_;
        ConnectionStatus connection_;

        TrackerNetworkStatistics netstat_;
        TrackerStatus status_;
        TrackerStatistic statistic_;
        TrackerConfiguration config_;
        TrackerData data_;
    };

}	// namespace dkvr