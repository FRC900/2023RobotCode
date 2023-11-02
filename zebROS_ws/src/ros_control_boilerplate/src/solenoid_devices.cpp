#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include <hardware_interface/joint_command_interface.h>

#include "remote_joint_interface/remote_joint_interface.h"
#include "ros_control_boilerplate/solenoid_devices.h"
#include "ros_control_boilerplate/solenoid_device.h"
#include "ros_control_boilerplate/read_config_utils.h"

SolenoidDevices::SolenoidDevices(ros::NodeHandle &root_nh)
    : state_interface_{std::make_unique<hardware_interface::JointStateInterface>()}
    , command_interface_{std::make_unique<hardware_interface::JointCommandInterface>()}
    , position_joint_interface_{std::make_unique<hardware_interface::PositionJointInterface>()}
    , remote_joint_interface_{std::make_unique<hardware_interface::RemoteJointInterface>()}
    , mode_interface_{std::make_unique<hardware_interface::JointModeInterface>()}
    , remote_mode_interface_{std::make_unique<hardware_interface::RemoteJointModeInterface>()}
{
    // TODO : this shouldn't be hard-coded?
	ros::NodeHandle nh(root_nh, "hardware_interface");
    XmlRpc::XmlRpcValue joint_param_list;
    if (!nh.getParam("joints", joint_param_list))
    {
        throw std::runtime_error("No joints were specified->");
    }
	for (int i = 0; i < joint_param_list.size(); i++)
	{
 		const XmlRpc::XmlRpcValue &joint_params = joint_param_list[i];
        std::string joint_name;
        std::string joint_type;
        readStringRequired(joint_params, "name", joint_name);
        readStringRequired(joint_params, "type", joint_type, joint_name);

        if (joint_type == "solenoid")
        {
            bool local = true;
            const bool saw_local_keyword = readBooleanOptional(joint_params, "local", local, joint_name);
            bool local_update;
            bool local_hardware;
            readJointLocalParams(joint_params, joint_name, local, saw_local_keyword, local_update, local_hardware);
			// These don't matter for remote hardware
			frc::PneumaticsModuleType module_type = frc::PneumaticsModuleType::CTREPCM;
			int channel = -1;
			int module_id = -1;
            if (local_hardware)
            {
                readIntRequired(joint_params, "channel", channel, joint_name);
				readSolenoidModuleType(joint_params, local_hardware, module_type, joint_name);
                readIntRequired(joint_params, "module_id", module_id, joint_name);
            }

            devices_.emplace_back(std::make_unique<SolenoidDevice>(i, joint_name, channel, module_type, module_id, local_hardware, local_update));
        }
    }
}

SolenoidDevices::~SolenoidDevices() = default;

hardware_interface::InterfaceManager *SolenoidDevices::registerInterface()
{
    for (const auto &d : devices_)
    {
        d->registerInterfaces(*state_interface_, *command_interface_, *position_joint_interface_, *remote_joint_interface_, *mode_interface_, *remote_mode_interface_);
    }
    interface_manager_.registerInterface(state_interface_.get());
    interface_manager_.registerInterface(command_interface_.get());
    interface_manager_.registerInterface(position_joint_interface_.get());
    interface_manager_.registerInterface(remote_joint_interface_.get());
    interface_manager_.registerInterface(mode_interface_.get());
    interface_manager_.registerInterface(remote_mode_interface_.get());
    return &interface_manager_;
}

void SolenoidDevices::write(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("solenoids");
    for (const auto &d : devices_)
    {
        d->write(time, period);
    }
}
