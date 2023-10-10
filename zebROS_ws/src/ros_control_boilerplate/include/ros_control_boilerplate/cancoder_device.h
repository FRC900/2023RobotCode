#ifndef CANCODER_DEVICE_INC__
#define CANCODER_DEVICE_INC__

#include <mutex>
#include <string>
#include <thread>

#include "ros_control_boilerplate/ctre_v6_device.h"
namespace ctre::phoenix6::hardware::core
{
    class CoreCANcoder;
}
namespace hardware_interface::cancoder
{
    class CANCoderHWState;
    class CANCoderHWCommand;
    class CANCoderStateInterface;
    class CANCoderCommandInterface;
    class RemoteCANCoderStateInterface;
}
class Tracer;

class CANCoderDevice : public CTREV6Device
{
public:
    CANCoderDevice(const std::string &name_space,
                   const int joint_index,
                   const std::string &joint_name,
                   const int cancoder_id,
                   const std::string &can_bus,
                   const bool local_hardware,
                   const bool local_update,
                   const double read_hz);
    CANCoderDevice(const CANCoderDevice &) = delete;
    CANCoderDevice(CANCoderDevice &&other) noexcept = delete;
    ~CANCoderDevice();
    CANCoderDevice &operator=(const CANCoderDevice &) = delete;
    CANCoderDevice &operator=(CANCoderDevice &&) noexcept = delete;

    void registerInterfaces(hardware_interface::cancoder::CANCoderStateInterface &state_interface,
                            hardware_interface::cancoder::CANCoderCommandInterface &command_interface,
                            hardware_interface::cancoder::RemoteCANCoderStateInterface &remote_state_interface);
    void read(const ros::Time &time, const ros::Duration &period);
    void write(const ros::Time &time, const ros::Duration &period);

private:
    const bool local_hardware_;
    const bool local_update_;

    std::unique_ptr<ctre::phoenix6::hardware::core::CoreCANcoder> cancoder_;

    std::unique_ptr<hardware_interface::cancoder::CANCoderHWState> state_;
    std::unique_ptr<hardware_interface::cancoder::CANCoderHWCommand> command_;

    std::unique_ptr<hardware_interface::cancoder::CANCoderHWState> read_thread_state_;
    std::unique_ptr<std::mutex> read_state_mutex_;
    std::unique_ptr<std::thread> read_thread_;
    void read_thread(std::unique_ptr<Tracer> tracer,
                     const double poll_frequency);
};

#endif