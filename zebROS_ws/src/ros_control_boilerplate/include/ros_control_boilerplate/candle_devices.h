#ifndef CANDLE_DEVICES_H_INC__
#define CANDLE_DEVICES_H_INC__

#include "ros_control_boilerplate/devices.h"

class CANdleDevice;
namespace hardware_interface::candle
{
    class CANdleStateInterface;
    class CANdleCommandInterface;
    class RemoteCANdleStateInterface;
}

class CANdleDevices : public Devices
{
public:
    CANdleDevices(ros::NodeHandle &root_nh);
    CANdleDevices(const CANdleDevices &) = delete;
    CANdleDevices(CANdleDevices &&) noexcept = delete;
    virtual ~CANdleDevices();

    CANdleDevices &operator=(const CANdleDevices &) = delete;
    CANdleDevices &operator=(CANdleDevices &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void write(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

private:
    std::vector<std::unique_ptr<CANdleDevice>> devices_;
    std::unique_ptr<hardware_interface::candle::CANdleStateInterface> state_interface_;
    std::unique_ptr<hardware_interface::candle::CANdleCommandInterface> command_interface_;
    std::unique_ptr<hardware_interface::candle::RemoteCANdleStateInterface> remote_state_interface_;
    hardware_interface::InterfaceManager interface_manager_;
};

#endif