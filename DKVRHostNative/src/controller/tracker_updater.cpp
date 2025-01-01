#include "controller/tracker_updater.h"

#include <chrono>
#include <cstdint>

#include "instruction/instruction_set.h"

#include "tracker/tracker.h"
#include "tracker/tracker_info.h"

namespace dkvr 
{

    namespace
    {
        constexpr std::chrono::milliseconds kThreadDelay(1000);
        constexpr std::chrono::milliseconds kHeartbeatInterval(1000);
        constexpr std::chrono::milliseconds kTimeoutInterval(5000);
#ifdef _DEBUG
        constexpr std::chrono::milliseconds kRttUpdateInterval(500);
        constexpr std::chrono::milliseconds kStatusUpdateInterval(500);
#else
        constexpr std::chrono::milliseconds kRttUpdateInterval(5000);
        constexpr std::chrono::milliseconds kStatusUpdateInterval(5000);
#endif

        Instruction BuildInstruction(InstructionHint hint, uint32_t seq, const void* payload)
        {
            Instruction inst = hint.ToInstruction();
            inst.sequence = seq;
            if (payload != nullptr)
                memcpy_s(inst.payload, sizeof Instruction::payload, payload, hint.length());

            return inst;
        }
    }

    TrackerUpdater::TrackerUpdater(NetworkService& net_service, TrackerProvider& tk_provider) :
        now_(),
        updater_thread_(*this),
        net_service_(net_service),
        tk_provider_(tk_provider)
    { 
        updater_thread_ += &TrackerUpdater::UpdateTracker;
    }

    void TrackerUpdater::Run()
    {
        updater_thread_.Run();
        logger_.Debug("Tracker updater thread launched.");
    }

    void TrackerUpdater::Stop()
    {
        updater_thread_.Stop();
        logger_.Debug("Tracker updater thread closed.");
    }

    void TrackerUpdater::UpdateTracker()
    {
        {
            std::vector<AtomicTracker> trackers = tk_provider_.GetAllTrackers();
            now_ = std::chrono::steady_clock::now();

            // individual tracker update
            for (AtomicTracker& target : trackers)
            {
                UpdateConnection(target);
                if (!target->IsConnected()) continue;	// don't update if not connected

                UpdateHeartbeatAndRtt(target);
                HandleUpdateRequired(target);
                SyncConfigurationWithClient(target);
            }

            // bunch tracker update
            static std::chrono::time_point last_status_update = std::chrono::steady_clock::now();
            if ((now_ - last_status_update) >= kStatusUpdateInterval)
            {
                for (AtomicTracker& target : trackers)
                {
                    UpdateStatusAndStatistic(target);
                }

                last_status_update = now_;
            }
        }	// Atomic Tracker release

        // delay
        std::this_thread::sleep_for(kThreadDelay);
    }

    void TrackerUpdater::UpdateConnection(Tracker* target)
    {
        switch (target->connection_status())
        {
        default:
        case Tracker::ConnectionStatus::Disconnected:
            break;

        case Tracker::ConnectionStatus::Handshaked:
        {
            Instruction inst = BuildInstruction(InstructionSet::Handshake2, target->send_sequence_num(), nullptr);
            net_service_.Send(target->address(), inst);
            break;
        }

        case Tracker::ConnectionStatus::Connected:
        {
            if ((now_ - target->last_heartbeat_recv()) >= kTimeoutInterval) {
                target->Reset();
#ifdef DKVR_DEBUG_TRACKER_CONNECTION_DETAIL
                unsigned long ip = target->address();
                unsigned char* ptr = reinterpret_cast<unsigned char*>(&ip);
                logger_.Debug("Tracker timed-out (ip {:d}.{:d}.{:d}.{:d})", ptr[0], ptr[1], ptr[2], ptr[3]);
#endif
                break;
            }
        }
        }
    }

    void TrackerUpdater::UpdateHeartbeatAndRtt(Tracker* target)
    {
        if ((now_ - target->last_heartbeat_sent()) >= kHeartbeatInterval) {
            Instruction inst = BuildInstruction(InstructionSet::Heartbeat, target->send_sequence_num(), nullptr);
            net_service_.Send(target->address(), inst);
            target->UpdateHeartbeatSent();
        }

        if ((now_ - target->last_ping_sent()) >= kRttUpdateInterval) {
            Instruction inst = BuildInstruction(InstructionSet::Ping, target->send_sequence_num(), nullptr);
            net_service_.Send(target->address(), inst);
            target->UpdatePingSent();
        }
    }

    void TrackerUpdater::HandleUpdateRequired(Tracker* target)
    {
        if (target->IsStatisticUpdateRequired())
        {
            Instruction inst = BuildInstruction(InstructionSet::Statistic, target->send_sequence_num(), nullptr);
            net_service_.Send(target->address(), inst);
        }

        if (target->IsStatusUpdateRequired())
        {
            Instruction inst = BuildInstruction(InstructionSet::Status, target->send_sequence_num(), nullptr);
            net_service_.Send(target->address(), inst);
        }

        if (target->IsLocateRequired())
        {
            Instruction inst = BuildInstruction(InstructionSet::Locate, target->send_sequence_num(), nullptr);
            net_service_.Send(target->address(), inst);
        }
    }

    void TrackerUpdater::SyncConfigurationWithClient(Tracker* target)
    {
        using ConfigurationKey = TrackerConfiguration::ConfigurationKey;

        if (!target->IsAllSynced()) {
            for (ConfigurationKey key : target->GetEveryUnsynced()) {
                Instruction inst{};
                switch (key)
                {
                case ConfigurationKey::Behavior:
                {
                    uint8_t behavior = target->behavior_encoded();
                    inst = BuildInstruction(InstructionSet::Behavior, target->send_sequence_num(), &behavior);
                    break;
                }

                case ConfigurationKey::GyrTransform:
                    inst = BuildInstruction(InstructionSet::CalibrationGr, target->send_sequence_num(), target->calibration_cref().gyr_transform);
                    break;

                case ConfigurationKey::AccTransform:
                    inst = BuildInstruction(InstructionSet::CalibrationAc, target->send_sequence_num(), target->calibration_cref().acc_transform);
                    break;

                case ConfigurationKey::MagTransform:
                    inst = BuildInstruction(InstructionSet::CalibrationMg, target->send_sequence_num(), target->calibration_cref().mag_transform);
                    break;

                case ConfigurationKey::NoiseVariance:
                    inst = BuildInstruction(InstructionSet::NoiseVariance, target->send_sequence_num(), target->calibration_cref().noise_variance);
                    break;

                default:
                case ConfigurationKey::Size:
                    continue;
                }
                net_service_.Send(target->address(), inst);
            }
        }
    }

    void TrackerUpdater::UpdateStatusAndStatistic(Tracker* target)
    {
        Instruction inst1 = BuildInstruction(InstructionSet::Status, target->send_sequence_num(), nullptr);
        Instruction inst2 = BuildInstruction(InstructionSet::Statistic, target->send_sequence_num(), nullptr);
        net_service_.Send(target->address(), inst1);
        net_service_.Send(target->address(), inst2);
    }

}	// namespace dkvr