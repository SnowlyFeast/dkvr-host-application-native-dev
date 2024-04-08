#pragma once

namespace dkvr {

	class NetResult
	{
	public:
		static constexpr int OK				= 0b0000;

		static constexpr int Failed			= 0b0001;
		static constexpr int InitRequired	= Failed | (1 << 1);
		static constexpr int InitFailed		= Failed | (2 << 1);
		static constexpr int ServerClosed	= Failed | (3 << 1);
		static constexpr int BindFailed		= Failed | (4 << 1);
		static constexpr int BindRequired	= Failed | (5 << 1);

		static bool AssertFailed(int res) { return res & Failed; }
	};

}	// namespace dkvr