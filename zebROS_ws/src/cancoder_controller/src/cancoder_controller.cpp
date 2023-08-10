#include "cancoder_controller/cancoder_controller.h"

namespace cancoder_controller
{
bool CANCoderController::init(hardware_interface::cancoder::CANCoderCommandInterface *hw,
		                      ros::NodeHandle                                        &/*root_nh*/,
						      ros::NodeHandle                                        &controller_nh)
{
	std::string joint_name;

	if (!controller_nh.getParam("joint_name", joint_name))
	{
		ROS_ERROR_STREAM("cancoder controller - could not read joint_name param");
		return false;
	}
	ROS_INFO("Got joint %s in CANCoder controller", joint_name.c_str());

	auto cancoder_handle = hw->getHandle(joint_name);
	interface_ = std::make_unique<cancoder_controller_interface::CANCoderControllerInterface>(controller_nh, joint_name, cancoder_handle);

	std::string initialization_strategy;
	hardware_interface::cancoder::SensorInitializationStrategy strategy = hardware_interface::cancoder::SensorInitializationStrategy::BootToAbsolutePosition;

	if (controller_nh.getParam("initialization_strategy", initialization_strategy))
	{	
		if (initialization_strategy == "boot_to_zero")
		{
			strategy = hardware_interface::cancoder::SensorInitializationStrategy::BootToZero;
		}
		else if (initialization_strategy == "boot_to_absolute_position")
		{
			strategy = hardware_interface::cancoder::SensorInitializationStrategy::BootToAbsolutePosition;
		}
		else
		{
			ROS_ERROR_STREAM("cancoder controller - invalid initialization_strategy param");
		}
	}

	interface_->setInitializationStrategy(strategy);
	return true;
}

void CANCoderController::starting(const ros::Time &/*time*/)
{
}

void CANCoderController::update(const ros::Time &/*time*/, const ros::Duration & /*period*/)
{
	interface_->update();
}

void CANCoderController::stopping(const ros::Time & /*time*/)
{}

} // namespace cancoder_controller

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cancoder_controller::CANCoderController, controller_interface::ControllerBase)
