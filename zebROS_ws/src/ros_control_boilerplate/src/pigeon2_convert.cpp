#include "ros_control_boilerplate/pigeon2_convert.h"

namespace pigeon2_convert
{
bool Pigeon2Convert::axisDirection(ctre::phoenix::sensors::AxisDirection input,
		hardware_interface::pigeon2::AxisDirection &output) const
{
	switch (input)
	{
		case ctre::phoenix::sensors::AxisDirection::PositiveZ:
			output = hardware_interface::pigeon2::AxisDirection::PositiveZ;
			break;
		case ctre::phoenix::sensors::AxisDirection::PositiveY:
			output = hardware_interface::pigeon2::AxisDirection::PositiveY;
			break;
		case ctre::phoenix::sensors::AxisDirection::PositiveX:
			output = hardware_interface::pigeon2::AxisDirection::PositiveX;
			break;
		case ctre::phoenix::sensors::AxisDirection::NegativeZ:
			output = hardware_interface::pigeon2::AxisDirection::NegativeZ;
			break;
		case ctre::phoenix::sensors::AxisDirection::NegativeY:
			output = hardware_interface::pigeon2::AxisDirection::NegativeY;
			break;
		case ctre::phoenix::sensors::AxisDirection::NegativeX:
			output = hardware_interface::pigeon2::AxisDirection::NegativeX;
			break;
		default:
			ROS_ERROR("Invalid input in convertCANCoderMagnetFieldStrength");
			return false;
	}
	return true;
}

} // namespace pigeon2_convert
