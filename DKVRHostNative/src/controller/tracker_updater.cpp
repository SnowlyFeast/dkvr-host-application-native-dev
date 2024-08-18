#include "controller/tracker_updater.h"

#include <cstdint>

#include "controller/instruction_set.h"

#include "tracker/tracker.h"
#include "tracker/tracker_info.h"

namespace dkvr {

	static Instruction BuildInstruction(InstructionHint, uint32_t, const void*);

	constexpr std::chrono::milliseconds kThreadDelay = std::chrono::milliseconds(1000);
	constexpr long long kHeartbeatInterval	= 1000;
	constexpr long long kTimeoutInterval	= 5000;
	constexpr long long kRttUpdateInterval	= 5000;

	TrackerUpdater::TrackerUpdater(NetworkService& net_service, TrackerProvider& tk_provider) :
		thread_ptr_(nullptr), exit_flag_(false), now_(), net_service_(net_service), tk_provider_(tk_provider) { }

	void TrackerUpdater::Run()
	{
		if (thread_ptr_)
			return;

		logger_.Debug("Launching tracker updater thread.");
		exit_flag_ = false;
		thread_ptr_ = std::make_unique<std::thread>(&TrackerUpdater::UpdaterThreadLoop, this);
	}

	void TrackerUpdater::Stop()
	{
		if (thread_ptr_) 
		{
			exit_flag_ = true;
			thread_ptr_->join();
			thread_ptr_.reset();
			logger_.Debug("Tracker updater thread successfully closed.");
		}
	}

	void TrackerUpdater::UpdaterThreadLoop()
	{
		while (!exit_flag_) 
		{
			UpdateTracker();
			std::this_thread::sleep_for(kThreadDelay);
		}
	}

	void TrackerUpdater::UpdateTracker()
	{
		std::vector<AtomicTracker> trackers = tk_provider_.GetAllTrackers();
		now_ = std::chrono::steady_clock::now();

		for (AtomicTracker& target : trackers) 
		{
			UpdateConnection(target);
			if (!target->IsConnected())
				continue;

			UpdateHeartbeatAndRtt(target);
			HandleUpdateRequired(target);
			MatchConfigurationWithClient(target);
		}
	}

	void TrackerUpdater::UpdateConnection(Tracker* target)
	{
		switch (target->connection_status())
		{
		default:
		case ConnectionStatus::Disconnected:
			break;

		case ConnectionStatus::Handshaked:
		{
			Instruction inst = BuildInstruction(InstructionSet::Handshake2, target->send_sequence_num(), nullptr);
			net_service_.Send(target->address(), inst);
			break;
		}

		case ConnectionStatus::Connected:
		{
			using namespace std::chrono;

			milliseconds timeout = duration_cast<milliseconds>(now_ - target->last_heartbeat_recv());

			if (timeout.count() > kTimeoutInterval) {
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
		using namespace std::chrono;

		milliseconds duration = duration_cast<milliseconds>(now_ - target->last_heartbeat_sent());

		if (duration.count() >= kHeartbeatInterval) {
			Instruction inst = BuildInstruction(InstructionSet::Heartbeat, target->send_sequence_num(), nullptr);
			net_service_.Send(target->address(), inst);
			target->UpdateHeartbeatSent();
		}

		if (duration.count() >= kRttUpdateInterval) {
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

	void TrackerUpdater::MatchConfigurationWithClient(Tracker* target)
	{
		if (!target->IsAllValid()) {
			for (ConfigurationKey key : target->GetEveryInvalid()) {
				Instruction inst{};
				switch (key)
				{
				case ConfigurationKey::Behavior:
				{
					uint8_t behavior = target->behavior();
					inst = BuildInstruction(InstructionSet::Behavior, target->send_sequence_num(), &behavior);
					break;
				}

				case ConfigurationKey::GyrTransform:
					inst = BuildInstruction(InstructionSet::CalibrationGr, target->send_sequence_num(), target->calibration_ref().gyr_transform);
					break;

				case ConfigurationKey::AccTransform:
					inst = BuildInstruction(InstructionSet::CalibrationAc, target->send_sequence_num(), target->calibration_ref().acc_transform);
					break;

				case ConfigurationKey::MagTransform:
					inst = BuildInstruction(InstructionSet::CalibrationMg, target->send_sequence_num(), target->calibration_ref().mag_transform);
					break;

				case ConfigurationKey::NoiseVariance:
					inst = BuildInstruction(InstructionSet::NoiseVariance, target->send_sequence_num(), target->calibration_ref().gyr_noise_var);
					break;

				default:
				case ConfigurationKey::Size:
					continue;
				}
				net_service_.Send(target->address(), inst);
			}
		}
	}

	Instruction BuildInstruction(InstructionHint hint, uint32_t seq, const void* payload)
	{
		Instruction inst
		{
			.header = kOpenerValue,
			.length = hint.length(),
			.align = hint.align,
			.opcode = static_cast<uint8_t>(hint.opcode),
			.sequence = seq
		};
		if (payload != nullptr)
			memcpy_s(inst.payload, sizeof(Instruction::payload), payload, hint.length());

		return inst;
	}

}	// namespace dkvr