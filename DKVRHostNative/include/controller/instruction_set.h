#pragma once

#include <cstdint>

namespace dkvr {

	inline constexpr uint8_t kOpenerValue = 'D';

	enum class Opcode : uint8_t
	{
		// networking op
		Handshake1		= 0x01,
		Handshake2		= 0x02,
		Heartbeat		= 0x03,
		Ping			= 0x04,
		Pong			= 0x05,

		// miscellaneous op
		Locate			= 0x11,
		ClientName		= 0x12,

		// configuration op
		Behavior		= 0x21,
		GyrTransform	= 0x22,
		AccTransform	= 0x23,
		MagTransform	= 0x24,
		NoiseVariance	= 0x25,

		// data transfer op
		Status			= 0x31,
		Raw				= 0x32,
		Orientation		= 0x33,
		Statistic		= 0x34,
		Debug			= 0x3F
	};

	struct InstructionHint
	{
		Opcode opcode;
		uint8_t align;
		uint8_t arg_count;

		constexpr uint8_t length() const { return align * arg_count; }
	};

	class InstructionSet
	{
	public:
		static constexpr uint8_t kOpcodeClassMask = 0xF0;
		static constexpr uint8_t kClassNetworking = 0x00;
		static constexpr uint8_t kClassMiscellaneous = 0x10;
		static constexpr uint8_t kClassConfiguration = 0x20;
		static constexpr uint8_t kClassDataTransfer = 0x30;

		static constexpr InstructionHint Handshake1{ Opcode::Handshake1, 0, 0 };
		static constexpr InstructionHint Handshake2{ Opcode::Handshake2, 0, 0 };
		static constexpr InstructionHint Heartbeat{ Opcode::Heartbeat, 0, 0 };
		static constexpr InstructionHint Ping{ Opcode::Ping, 0, 0 };
		static constexpr InstructionHint Pong{ Opcode::Pong, 0, 0 };

		static constexpr InstructionHint Locate{ Opcode::Locate, 0, 0 };
		static constexpr InstructionHint ClientName{ Opcode::ClientName, 0, 0 };

		static constexpr InstructionHint Behavior{ Opcode::Behavior, 1, 1 };
		static constexpr InstructionHint CalibrationGr{ Opcode::GyrTransform, 4, 12 };
		static constexpr InstructionHint CalibrationAc{ Opcode::AccTransform, 4, 12 };
		static constexpr InstructionHint CalibrationMg{ Opcode::MagTransform, 4, 12 };
		static constexpr InstructionHint NoiseVariance{ Opcode::NoiseVariance, 4, 9 };

		static constexpr InstructionHint Status{ Opcode::Status, 0, 0 };
		static constexpr InstructionHint ImuRaw{ Opcode::Raw, 0, 0 };
		static constexpr InstructionHint ImuQuat{ Opcode::Orientation, 0, 0 };
		static constexpr InstructionHint Statistic{ Opcode::Statistic, 0, 0 };
		static constexpr InstructionHint Debug{ Opcode::Debug, 0, 0 };
	};

}	// namespace dkvr