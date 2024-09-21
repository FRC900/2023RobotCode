#ifndef SIMULATOR_INTERFACE_SIMULATOR_BASE_H_
#define SIMULATOR_INTERFACE_SIMULATOR_BASE_H_
#include "ctre/phoenix6/core/CoreTalonFX.hpp"
#include "ctre/phoenix6/core/CoreCANcoder.hpp"
#include "ctre_interfaces/talonfxpro_state_interface.h"
#include "ros/ros.h"

namespace simulator_base
{
  class Simulator
  {
    public:
      virtual void init(const XmlRpc::XmlRpcValue &simulator_info) {

      };
      virtual void update(const std::string &name, const ros::Time &time, const ros::Duration &period, ctre::phoenix6::sim::TalonFXSimState &state) {
        
      };
      virtual ~Simulator(){}
    
    protected:
      Simulator(){}
  };
};
#endif