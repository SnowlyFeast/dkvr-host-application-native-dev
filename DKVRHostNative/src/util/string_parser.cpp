#include "util/string_parser.h"

#include <sstream>

namespace dkvr {

	std::string StringParser::IPToString(unsigned long ip)
	{
		unsigned char* ptr = reinterpret_cast<unsigned char*>(&ip);
		std::stringstream ss;
		for (int i = 0; i < 4; i++) {
			unsigned char c = ptr[i];
			if (c >= 100) {
				ss << (c / 100) << ((c % 100) / 10) << (c % 10);
			}
			else if (c >= 10) {
				ss << (c / 10) << (c % 10);
			}
			else if (c == 0) {
				ss << '0';
			}
			else {
				ss << (int)c;
			}
			if (i < 3)
				ss << '.';
		}
		return ss.str();
	}

}	// namespace dkvr