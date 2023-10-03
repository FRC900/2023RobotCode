
#include "ctre_interfaces/latency_compensation_state_interface.h"

namespace hardware_interface::latency_compensation
{
class CTRELatencyCompensationEntry
{
public:
    CTRELatencyCompensationEntry()
    {
    }
    CTRELatencyCompensationEntry(const CTRELatencyCompensationEntry &) = delete;
    CTRELatencyCompensationEntry(CTRELatencyCompensationEntry &&) noexcept = default;
    ~CTRELatencyCompensationEntry() = default;
    CTRELatencyCompensationEntry operator=(const CTRELatencyCompensationEntry &) = delete;
    CTRELatencyCompensationEntry &operator=(CTRELatencyCompensationEntry &&) = delete;

    void setTimestamp(const ros::Time timestamp)
    {
        timestamp_ = timestamp;
    }
    ros::Time getTimestamp(void) const
    {
        return timestamp_;
    }

    void setValue(const double value)
    {
        value_ = value;
    }
    double getValue(void) const
    {
        return value_;
    }

    void setSlope(const double slope)
    {
        slope_ = slope;
    }
    double getSlope(void) const
    {
        return slope_;
    }

private:
    ros::Time timestamp_;
    double value_;
    double slope_;
};


CTRELatencyCompensationState::CTRELatencyCompensationState(const std::string &name)
    : name_{name}
{
}

CTRELatencyCompensationState::~CTRELatencyCompensationState(void) = default;

void CTRELatencyCompensationState::addEntry(const std::string &name)
{
    auto entry = entries_.find(name);
    if (entry != entries_.end())
    {
        ROS_WARN_STREAM("addEntry() called on existing latency group name " << name);
        return;
    }
    entries_.emplace(name, std::make_unique<CTRELatencyCompensationEntry>());
}

void CTRELatencyCompensationState::setEntry(const std::string &name, const ros::Time &timestamp, const double value, const double slope)
{
    auto entry = entries_.find(name);
    if (entry == entries_.end())
    {
        ROS_WARN_STREAM("setEntry() can't find latency group name " << name);
        return;
    }
    entry->second->setTimestamp(timestamp);
    entry->second->setValue(value);
    entry->second->setSlope(slope);
}
void CTRELatencyCompensationState::getEntry(const std::string &name, ros::Time &timestamp, double &value, double &slope)
{
    auto entry = entries_.find(name);
    if (entry == entries_.end())
    {
        ROS_WARN_STREAM("getEntry() can't find latency group name " << name);
        return;
    }
    timestamp = entry->second->getTimestamp();
    value = entry->second->getValue();
    slope = entry->second->getSlope();
}

double CTRELatencyCompensationState::getLatencyCompensatedValue(const std::string &name, const ros::Time &timestamp) const
{
    auto entry = entries_.find(name);
    if (entry == entries_.end())
    {
        ROS_WARN_STREAM("getLatencyCompensatedValue() can't find latency group name " << name);
        return 0;
    }
    const auto &e = entry->second;

    return e->getValue() + e->getSlope() * (timestamp - e->getTimestamp()).toSec();
}

} // namespace