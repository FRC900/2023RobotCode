#ifndef PIGEON2_CONVERT_INC__
#define PIGEON2_CONVERT_INC__

#include <ctre/phoenix/sensors/Pigeon2.h>
#include "pigeon2_interface/pigeon2_state_interface.h"

namespace pigeon2_convert
{
class Pigeon2Convert
{
	public:
		bool axisDirection(ctre::phoenix::sensors::AxisDirection input,
				hardware_interface::pigeon2::AxisDirection &output) const;
};

} // namespace pigeon2_convert

#endif