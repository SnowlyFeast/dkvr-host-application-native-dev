#pragma once

#include "controller.h"

namespace dkvr {

	class MiscellanousController : Controller
	{
	public:
		void Handle(Tracker* target, Instruction& inst) override;

	private:
		void Locate(Tracker* target, Instruction& inst);
	};

}	// namespace dkvr