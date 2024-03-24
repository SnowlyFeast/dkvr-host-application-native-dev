#include "con_config.h"

#include "instruction_set.h"

namespace dkvr {

	void ConfigurationController::Handle(Tracker* target, Instruction& inst)
	{
		switch (Opcode(inst.opcode))
		{
		case Opcode::Active:
			Behavior(target, inst);
			break;

		case Opcode::Inactive:
			Behavior(target, inst);
			break;

		case Opcode::Behavior:
			Behavior(target, inst);
			break;

		case Opcode::CalibrationGr:
			CalibrationGr(target, inst);
			break;

		case Opcode::CalibrationAc:
			CalibrationAc(target, inst);
			break;

		case Opcode::CalibrationMg:
			CalibrationMg(target, inst);
			break;

		default:
			break;
		}
	}

	void ConfigurationController::Behavior(Tracker* target, Instruction& inst)
	{
		if (target->IsConnected()) {
			target->Validate(ConfigurationKey::Behavior);
		}
	}

	void ConfigurationController::CalibrationGr(Tracker* target, Instruction& inst)
	{
		if (target->IsConnected()) {
			target->Validate(ConfigurationKey::CalibrationGr);
		}
	}

	void ConfigurationController::CalibrationAc(Tracker* target, Instruction& inst)
	{
		if (target->IsConnected()) {
			target->Validate(ConfigurationKey::CalibrationAc);
		}
	}

	void ConfigurationController::CalibrationMg(Tracker* target, Instruction& inst)
	{
		if (target->IsConnected()) {
			target->Validate(ConfigurationKey::CalibrationMg);
		}
	}

}	// namespace dkvr