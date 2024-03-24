#pragma once

#include "controller.h"

namespace dkvr {

	class ConfigurationController : Controller
	{
	public:
		void Handle(Tracker* target, Instruction& inst) override;

	private:
		void Behavior(Tracker* target, Instruction& inst);
		void CalibrationGr(Tracker* target, Instruction& inst);
		void CalibrationAc(Tracker* target, Instruction& inst);
		void CalibrationMg(Tracker* target, Instruction& inst);
	};

}	// namespace dkvr