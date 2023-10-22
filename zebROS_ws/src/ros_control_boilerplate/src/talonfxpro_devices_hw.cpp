#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "ros_control_boilerplate/../../src/talonfxpro_devices.cpp"
#include "ros_control_boilerplate/../../src/talonfxpro_device.cpp"

template class TalonFXProDevices<false>;