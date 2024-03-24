#include "con_miscell.h"

#include "instruction_set.h"

namespace dkvr {

	void MiscellanousController::Handle(Tracker* target, Instruction& inst)
	{
		switch (Opcode(inst.opcode))
		{
		case Opcode::Locate:
			Locate(target, inst);
			break;

		default:
			break;
		}
	}

	void MiscellanousController::Locate(Tracker* target, Instruction& inst)
	{
		// host side opcode
		logger_.Debug("Host-side opcode(Locate) received.");
	}

}	// namespace dkvr