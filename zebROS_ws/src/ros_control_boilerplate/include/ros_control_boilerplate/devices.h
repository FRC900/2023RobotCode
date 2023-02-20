#ifndef INC_DEVICES_H__
#define INC_DEVICES_H__

#include <ros/ros.h>
#include "ros_control_boilerplate/tracer.h"

namespace hardware_interface
{
    class InterfaceManager;
}

class Devices
{
public:
    Devices() = default;
    Devices(const Devices &) = delete;
    Devices(Devices &&) noexcept = delete;
    virtual ~Devices() = default;

    Devices& operator= (const Devices &) = delete;
    Devices& operator= (Devices &&) noexcept = delete;

    virtual hardware_interface::InterfaceManager *registerInterface() = 0;
    // Read and write functions that are shared between hardware and sim interfaces
    virtual void read(const ros::Time& /*time*/, const ros::Duration& /*period*/, Tracer& /*tracer*/) {}
    virtual void write(const ros::Time& /*time*/, const ros::Duration& /*period*/, Tracer& /*tracer*/) {}

    // Read and write functions which add additional sim features
    virtual void simInit(ros::NodeHandle /*nh*/) {}
    virtual void simRead(const ros::Time& /*time*/, const ros::Duration& /*period*/, Tracer& /*tracer*/) {}
    virtual void simWrite(const ros::Time& /*time*/, const ros::Duration& /*period*/, Tracer& /*tracer*/) {}

    // Read and write functions which add additional hardware-only features
    virtual void hwInit(ros::NodeHandle /*nh*/) {}
    virtual void hwRead(const ros::Time& /*time*/, const ros::Duration& /*period*/, Tracer& /*tracer*/) {}
    virtual void hwWrite(const ros::Time& /*time*/, const ros::Duration& /*period*/, Tracer& /*tracer*/) {}

    // Used to signal that all controllers are finished loading
    static void signalReady(void) { ready_ = true; }

    static void setEnabled(const bool enabled) { enabled_ = enabled; }

protected:
    static bool isEnabled(void) { return enabled_; }
    static bool isReady(void)   { return ready_; }

private:
    static inline bool enabled_{true};
    static inline bool ready_{false};
};

template<class T>
T* getDevicesOfType(const std::vector<std::shared_ptr<Devices>> &device_list)
{
    for (auto &d: device_list)
    {
        auto di = std::dynamic_pointer_cast<T>(d);
        if (di)
        {
            return di.get();
        }
    }
    return nullptr;
}

#endif