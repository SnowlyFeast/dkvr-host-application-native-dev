#include "con_data.h"

#include "instruction_set.h"

namespace dkvr {

	void DataTransferController::Handle(Tracker* target, Instruction& inst)
	{
		switch (Opcode(inst.opcode))
		{
		case Opcode::Status:
			Status(target, inst);
			break;
		case Opcode::ImuRaw:
			ImuRaw(target, inst);
			break;
		case Opcode::ImuQuat:
			ImuQuat(target, inst);
			break;
		default:
			break;
		}
	}

	void DataTransferController::Status(Tracker* target, Instruction& inst)
	{
		// TODO: Make Tracker Status Struct. Implement.
		TrackerStatus status{};
		memcpy(&status, inst.payload, sizeof(status));
		target->set_tracker_status(status);
	}

	void DataTransferController::ImuRaw(Tracker* target, Instruction& inst)
	{
		if (target->IsConnected()) {
			target->set_gyro(Vector3(
				inst.payload[0].single,
				inst.payload[1].single,
				inst.payload[2].single
			));
			target->set_accel(Vector3(
				inst.payload[3].single,
				inst.payload[4].single,
				inst.payload[5].single
			));
			target->set_mag(Vector3(
				inst.payload[6].single,
				inst.payload[7].single,
				inst.payload[8].single
			));
		}
	}

	void DataTransferController::ImuQuat(Tracker* target, Instruction& inst)
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

}	// namespace dkvr