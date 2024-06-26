
#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "frc_interfaces/pdp_state_interface.h"
#include "ros_control_boilerplate/pdp_devices.h"
#include "ros_control_boilerplate/pdp_device.h"
#include "ros_control_boilerplate/read_config_utils.h"

PDPDevices::PDPDevices(ros::NodeHandle &root_nh)
    : state_interface_{std::make_unique<hardware_interface::PDPStateInterface>()}
    , remote_state_interface_{std::make_unique<hardware_interface::RemotePDPStateInterface>()}
{
    ros::NodeHandle param_nh(root_nh, "generic_hw_control_loop"); // TODO : this shouldn't be hard-coded?
    if(!param_nh.param("pdp_read_hz", read_hz_, read_hz_)) 
    {
		ROS_WARN("Failed to read pdp_read_hz in frc_robot_interface");
	}
	ros::NodeHandle nh(root_nh, "hardware_interface"); // TODO : this shouldn't be hard-coded?
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

        if (joint_type == "pdp")
        {
            bool local = true;
            readBooleanOptional(joint_params, "local", local, joint_name);
            int module = 0;
            const bool has_module = readIntOptional(joint_params, "module", module, joint_name);
            if (!local && has_module)
            {
				throw std::runtime_error("A PDP module id was specified with local_hardware == false for joint " + joint_name);
            }
			if (local && !has_module)
			{
                throw std::runtime_error("A PDP module id was not specified for joint " + joint_name);
            }

            devices_.emplace_back(std::make_unique<PDPDevice>(root_nh.getNamespace(), i, joint_name, module, local, read_hz_));
        }
    }
}

PDPDevices::~PDPDevices() = default;

hardware_interface::InterfaceManager *PDPDevices::registerInterface()
{
    for (const auto &d : devices_)
    {
        d->registerInterfaces(*state_interface_, *remote_state_interface_);
    }
    interface_manager_.registerInterface(state_interface_.get());
    interface_manager_.registerInterface(remote_state_interface_.get());
    return &interface_manager_;
}

void PDPDevices::read(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("PDPs");
    for (const auto &d : devices_)
    {
        d->read(time, period);
    }
}
