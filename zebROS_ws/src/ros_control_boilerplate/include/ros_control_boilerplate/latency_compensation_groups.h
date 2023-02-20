#ifndef INC_LATENCY_COMPENSATION_GROUPS_H_INC__
#define INC_LATENCY_COMPENSATION_GROUPS_H_INC__

#include "ros_control_boilerplate/devices.h"

class LatencyCompensationGroup;
namespace hardware_interface::latency_compensation
{
    class CTRELatencyCompensationStateInterface;
}

namespace ctre::phoenix6::hardware::core
{
    class CoreCANcoder;
    class CorePigeon2;
    class CoreTalonFX;
}

class LatencyCompensationGroups : public Devices
{
public:
    LatencyCompensationGroups(ros::NodeHandle &root_nh,
                              const std::map<std::string, ctre::phoenix6::hardware::core::CoreCANcoder *> &cancoders,
                              const std::map<std::string, ctre::phoenix6::hardware::core::CorePigeon2 *> &pigeon2s,
                              const std::map<std::string, ctre::phoenix6::hardware::core::CoreTalonFX *> &talon_fxs);
    LatencyCompensationGroups(const LatencyCompensationGroups &) = delete;
    LatencyCompensationGroups(LatencyCompensationGroups &&) noexcept = delete;
    virtual ~LatencyCompensationGroups();

    LatencyCompensationGroups &operator=(const LatencyCompensationGroups &) = delete;
    LatencyCompensationGroups &operator=(LatencyCompensationGroups &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void read(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

private:
    std::vector<std::unique_ptr<LatencyCompensationGroup>> devices_;
    std::unique_ptr<hardware_interface::latency_compensation::CTRELatencyCompensationStateInterface> state_interface_;
    hardware_interface::InterfaceManager interface_manager_;
};

#endif