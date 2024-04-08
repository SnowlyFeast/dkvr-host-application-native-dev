#include "controller/tracker_updater.h"

#include <cstdint>

#include "controller/instruction_set.h"

namespace dkvr {

	static Instruction BuildInstruction(InstructionHint, uint32_t, const void*);

	constexpr std::chrono::milliseconds kThreadDelay = std::chrono::milliseconds(1000);
	constexpr long long kHeartbeatInterval	= 1000;
	constexpr long long kTimeoutInterval	= 5000;
	constexpr long long kRttUpdateInterval	= 5000;

	void TrackerUpdater::Run()
	{
		if (thread_)
			return;

		logger_.Debug("Launching tracker updater thread.");
		exit_flag_ = false;
		thread_ = new std::thread(&TrackerUpdater::UpdaterThreadLoop, this);
	}

	void TrackerUpdater::Stop()
	{
		if (thread_) 
		{
			exit_flag_ = true;
			thread_->join();
			delete thread_;
			thread_ = nullptr;
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

			UpdateHeartbeat(target);
			MatchConfigurationWithClient(target);
		}
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
			using namespace std::chrono;

			milliseconds timeout = duration_cast<milliseconds>(now_ - target->last_heartbeat_recv());

			if (timeout.count() > kTimeoutInterval) {
				target->Reset();
#ifdef DKVR_DEBUG_TRACKER_CONNECTION_DETAIL	// TODO : create some header for DEFINE
				unsigned long ip = target->address();
				unsigned char* ptr = reinterpret_cast<unsigned char*>(&ip);
				logger_.Debug("Tracker timed-out (ip {:d}.{:d}.{:d}.{:d})", ptr[0], ptr[1], ptr[2], ptr[3]);
#endif
				break;
			}
		}
		}
	}

	void TrackerUpdater::UpdateHeartbeat(Tracker* target)
	{
		using namespace std::chrono;

		milliseconds duration = duration_cast<milliseconds>(now_ - target->last_heartbeat_sent());

		if (duration.count() >= kHeartbeatInterval) {
			Instruction inst = BuildInstruction(InstructionSet::Heartbeat, target->send_sequence_num(), nullptr);
			net_service_.Send(target->address(), inst);
			target->UpdateHeartbeatSent();
		}
	}

	void TrackerUpdater::UpdateRtt(Tracker* target)
	{
		using namespace std::chrono;

		milliseconds duration = duration_cast<milliseconds>(now_ - target->last_ping_sent());

		if (duration.count() >= kRttUpdateInterval) {
			Instruction inst = BuildInstruction(InstructionSet::Ping, target->send_sequence_num(), nullptr);
			net_service_.Send(target->address(), inst);
			target->UpdatePingSent();
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

				case ConfigurationKey::CalibrationGr:
					inst = BuildInstruction(InstructionSet::CalibrationGr, target->send_sequence_num(), target->gyro_offset());
					break;

				case ConfigurationKey::CalibrationAc:
					inst = BuildInstruction(InstructionSet::CalibrationAc, target->send_sequence_num(), target->accel_mat());
					break;

				case ConfigurationKey::CalibrationMg:
					inst = BuildInstruction(InstructionSet::CalibrationMg, target->send_sequence_num(), target->mag_mat());
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
			.header = kHeaderValue,
			.length = hint.length,
			.align = hint.align,
			.opcode = static_cast<uint8_t>(hint.opcode),
			.sequence = seq
		};
		if (payload != nullptr)
			memcpy_s(inst.payload, sizeof(Instruction::payload), payload, hint.length);

		return inst;
	}

}	// namespace dkvr