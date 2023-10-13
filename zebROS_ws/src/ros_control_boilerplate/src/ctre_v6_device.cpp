#include "ros/console.h"
#include "hal/DriverStation.h"

#include "ctre/phoenix/StatusCodes.h"
#include "ros_control_boilerplate/ctre_v6_device.h"

CTREV6Device::CTREV6Device(const std::string &device_type,
                           const std::string &joint_name,
                           const int id)
    : device_type_{device_type}
    , name_{joint_name}
    , id_{id}
{
}

CTREV6Device::~CTREV6Device() = default;

bool CTREV6Device::safeCall(ctre::phoenix::StatusCode status_code, const std::string &method_name)
{
    if (status_code.IsOK())
    {
        can_error_count_ = 0;
        can_error_sent_ = false;
        // Only for debugging traces of successful calls ROS_INFO_STREAM("safeCall : " << name_ << " : " << method_name);
        return true; // Yay us!
    }
    ROS_ERROR_STREAM("Error : " << device_type_ << " " << name_ << " id = " << id_ << " calling " << method_name << " : " << status_code.GetName());
    can_error_count_++;
    if ((can_error_count_ > 1000) && !can_error_sent_)
    {
        HAL_SendError(true, -1, false, (device_type_ + " safeCall - too many CAN bus errors!").c_str(), "", "", true);
        can_error_sent_ = true;
    }
    return false;
}

const std::string &CTREV6Device::getName(void) const
{
    return name_;
}

int CTREV6Device::getId(void) const
{
    return id_;
}