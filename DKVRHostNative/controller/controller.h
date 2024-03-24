#pragma once

#include <string>

#include "instruction_format.h"
#include "tracker/tracker.h"
#include "util/logger.h"

namespace dkvr {

		class Controller
		{
		public:
			virtual void Handle(Tracker* target, Instruction& inst) = 0;

		protected:
			Logger& logger_ = Logger::GetInstance();
		};

}	// namespace dkvr