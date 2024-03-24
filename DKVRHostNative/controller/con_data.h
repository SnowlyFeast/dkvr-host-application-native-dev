#pragma once

#include "controller.h"

namespace dkvr {

	class DataTransferController : Controller
	{
	public:
		void Handle(Tracker* target, Instruction& inst) override;

	private:
		void Status(Tracker* target, Instruction& inst);
		void ImuRaw(Tracker* target, Instruction& inst);
		void ImuQuat(Tracker* target, Instruction& inst);

	};

}	// namespace dkvr