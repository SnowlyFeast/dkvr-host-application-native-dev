#include "controller/instruction_handler.h"

#include "controller/instruction_set.h"

namespace dkvr
{
	void InstructionHandler::Handle(Tracker* target, Instruction& inst)
	{
		switch (Opcode(inst.opcode))
		{
			// network
		case Opcode::Handshake1:
			Handshake1(target, inst); break;

		case Opcode::Handshake2:
			Handshake2(target, inst); break;

		case Opcode::Heartbeat:
			Heartbeat(target, inst); break;

		case Opcode::Ping:
			Ping(target, inst); break;

		case Opcode::Pong:
			Pong(target, inst); break;

			// miscellanous
		case Opcode::Locate:
			Locate(target, inst); break;

			// configuration
		case Opcode::Active:
			Behavior(target, inst); break;

		case Opcode::Inactive:
			Behavior(target, inst); break;

		case Opcode::Behavior:
			Behavior(target, inst); break;

		case Opcode::CalibrationGr:
			CalibrationGr(target, inst); break;

		case Opcode::CalibrationAc:
			CalibrationAc(target, inst); break;

		case Opcode::CalibrationMg:
			CalibrationMg(target, inst); break;

			// data transfer
		case Opcode::Status:
			Status(target, inst); break;

		case Opcode::ImuRaw:
			ImuRaw(target, inst); break;

		case Opcode::ImuQuat:
			ImuQuat(target, inst); break;

		case Opcode::Statistic:
			Statistic(target, inst); break;

			// unknown opcode
		default:
		{
			unsigned long ip = target->address();
			unsigned char* ptr = reinterpret_cast<unsigned char*>(&ip);
			logger_.Error(
				"Unknown instruction(0x{:x}) received from {:d}.{:d}.{:d}.{:d}",
				inst.opcode,
				ptr[0], ptr[1], ptr[2], ptr[3]
			);
			break;
		}
		}	// switch case
	}

	void InstructionHandler::Handshake1(Tracker* target, Instruction& inst)
	{
		if (target->IsDisconnected()) {
			target->SetHandshaked();
#ifdef DKVR_DEBUG_TRACKER_CONNECTION_DETAIL
			unsigned long ip = target->address();
			unsigned char* ptr = reinterpret_cast<unsigned char*>(&ip);
			logger_.Debug("Tracker handshaked (ip {:d}.{:d}.{:d}.{:d})", ptr[0], ptr[1], ptr[2], ptr[3]);
#endif
		}
	}

	void InstructionHandler::Handshake2(Tracker* target, Instruction& inst)
	{
		// host side opcode
		logger_.Debug("Host-side opcode(Handshake2) received.");
	}

	void InstructionHandler::Heartbeat(Tracker* target, Instruction& inst)
	{
		if (target->IsHandshaked()) {
			target->SetConnected();
#ifdef DKVR_DEBUG_TRACKER_CONNECTION_DETAIL
			unsigned long ip = target->address();
			unsigned char* ptr = reinterpret_cast<unsigned char*>(&ip);
			logger_.Debug("Tracker connected (ip {:d}.{:d}.{:d}.{:d})", ptr[0], ptr[1], ptr[2], ptr[3]);
#endif
		}
		target->UpdateHeartbeatRecv();
	}

	void InstructionHandler::Ping(Tracker* target, Instruction& inst)
	{
		// host side opcdoe
		logger_.Debug("Host-side opcode(Ping) received.");
	}

	void InstructionHandler::Pong(Tracker* target, Instruction& inst)
	{
		target->UpdateRtt();
	}

	void InstructionHandler::Locate(Tracker* target, Instruction& inst)
	{
		// host side opcode
		logger_.Debug("Host-side opcode(Locate) received.");
	}

	void InstructionHandler::Behavior(Tracker* target, Instruction& inst)
	{
		if (target->IsConnected()) {
			target->Validate(ConfigurationKey::Behavior);
		}
	}

	void InstructionHandler::CalibrationGr(Tracker* target, Instruction& inst)
	{
		if (target->IsConnected()) {
			target->Validate(ConfigurationKey::CalibrationGr);
		}
	}

	void InstructionHandler::CalibrationAc(Tracker* target, Instruction& inst)
	{
		if (target->IsConnected()) {
			target->Validate(ConfigurationKey::CalibrationAc);
		}
	}

	void InstructionHandler::CalibrationMg(Tracker* target, Instruction& inst)
	{
		if (target->IsConnected()) {
			target->Validate(ConfigurationKey::CalibrationMg);
		}
	}

	void InstructionHandler::Status(Tracker* target, Instruction& inst)
	{
		// TODO: Make Tracker Status Struct. Implement.
		TrackerStatus status{};
		memcpy(&status, inst.payload, sizeof(status));
		target->set_tracker_status(status);
	}

	void InstructionHandler::ImuRaw(Tracker* target, Instruction& inst)
	{
		if (target->IsConnected()) {
			Vector3 gyro{
				inst.payload[0].single,
				inst.payload[1].single,
				inst.payload[2].single
			};
			Vector3 accel{
				inst.payload[3].single,
				inst.payload[4].single,
				inst.payload[5].single
			};
			Vector3 mag
			{
				inst.payload[6].single,
				inst.payload[7].single,
				inst.payload[8].single
			};

			target->set_imu_readings(gyro, accel, mag);
		}
	}

	void InstructionHandler::ImuQuat(Tracker* target, Instruction& inst)
	{
		if (target->IsConnected()) {
			target->set_quaternion(Quaternion(
				inst.payload[0].single,
				inst.payload[1].single,
				inst.payload[2].single,
				inst.payload[3].single
			));
		}
	}

	void InstructionHandler::Statistic(Tracker* target, Instruction& inst)
	{
		if (target->IsConnected())
		{
			TrackerStatistic statistic{};
			memcpy(&statistic, inst.payload, sizeof TrackerStatistic);
			target->set_tracker_statistic(statistic);
		}
	}

}