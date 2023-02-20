#ifndef LATENCY_COMPENSATION_GROUP_INC__
#define LATENCY_COMPENSATION_GROUP_INC__

#include <map>
#include <mutex>
#include <string>
#include <thread>

#include "xmlrpcpp/XmlRpcValue.h"

namespace ctre::phoenix6
{
    class BaseStatusSignal;
    template <typename T> class StatusSignal;
    namespace hardware::core
    {
        class CoreCANcoder;
        class CorePigeon2;
        class CoreTalonFX;
    }
}
namespace hardware_interface::latency_compensation
{
    class CTRELatencyCompensationState;
    class CTRELatencyCompensationStateInterface;
}

class LatencyCompensationGroupEntryBase;

class LatencyCompensationGroup
{
public:
    LatencyCompensationGroup(const XmlRpc::XmlRpcValue &entry_array,
                             const std::string &name,
                             const double update_frequency,
                             const std::map<std::string, ctre::phoenix6::hardware::core::CoreCANcoder *> &cancoders,
                             const std::map<std::string, ctre::phoenix6::hardware::core::CorePigeon2 *> &pigeon2s,
                             const std::map<std::string, ctre::phoenix6::hardware::core::CoreTalonFX *> &talon_fxs);
    LatencyCompensationGroup(const LatencyCompensationGroup &) = delete;
    LatencyCompensationGroup(LatencyCompensationGroup &&other) noexcept = delete;
    virtual ~LatencyCompensationGroup();
    void registerInterfaces(hardware_interface::latency_compensation::CTRELatencyCompensationStateInterface &state_interface);

    LatencyCompensationGroup &operator=(const LatencyCompensationGroup &) = delete;
    LatencyCompensationGroup &operator=(LatencyCompensationGroup &&) noexcept = delete;

    void read(void);

private:
    template <typename VALUE_SIGNAL_TYPE, typename SLOPE_SIGNAL_TYPE>
    void create_group_entry(const std::string &name,
                            ctre::phoenix6::StatusSignal<VALUE_SIGNAL_TYPE> *value_signal,
                            ctre::phoenix6::StatusSignal<SLOPE_SIGNAL_TYPE> *slope_signal);
    void read_thread();
    const std::string name_;
    std::map<const std::string, std::unique_ptr<LatencyCompensationGroupEntryBase>> entries_;
    std::vector<ctre::phoenix6::BaseStatusSignal *> signals_;

    std::unique_ptr<hardware_interface::latency_compensation::CTRELatencyCompensationState> state_;

    std::mutex read_state_mutex_;
    std::unique_ptr<hardware_interface::latency_compensation::CTRELatencyCompensationState> read_thread_state_;
    std::thread read_thread_;
};

#endif
