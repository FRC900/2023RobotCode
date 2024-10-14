#include "ros/node_handle.h"
#include "ctre/phoenix6/core/CoreTalonFX.hpp"
#include "ctre/phoenix6/core/CoreCANcoder.hpp"
#include "ros_control_boilerplate/sim_talonfxpro_device.h"
#include "ctre_interfaces/talonfxpro_state_interface.h"
#include "gazebo/physics/Joint.hh"                          // for Joint
#include "gazebo/physics/Model.hh"                          // for Model
SimTalonFXProDevice::SimTalonFXProDevice(const std::string &name_space,
                                         const int joint_index,
                                         const std::string &joint_name,
                                         const int can_id,
                                         const std::string &can_bus,
                                         double read_hz)
    : TalonFXProDevice(name_space, joint_index, joint_name, can_id, can_bus, read_hz)
{
    this->joint_name_ = joint_name;
}

SimTalonFXProDevice::~SimTalonFXProDevice() = default;

void SimTalonFXProDevice::simRead(const ros::Time &time, const ros::Duration &period, Tracer &tracer, const units::voltage::volt_t battery_voltage)
{
    // tracer.start("talonfxpro cancoder check");
    using hardware_interface::talonfxpro::FeedbackSensorSource::FusedCANcoder;
    using hardware_interface::talonfxpro::FeedbackSensorSource::RemoteCANcoder;
    using hardware_interface::talonfxpro::FeedbackSensorSource::SyncCANcoder;
    if (state_->getFeedbackSensorSource() == FusedCANcoder || state_->getFeedbackSensorSource() == RemoteCANcoder || state_->getFeedbackSensorSource() == SyncCANcoder)
    {
        if (!cancoder_ || cancoder_->GetDeviceID() != state_->getFeedbackRemoteSensorID())
        {
            cancoder_ = std::make_unique<ctre::phoenix6::hardware::core::CoreCANcoder>(state_->getFeedbackRemoteSensorID());
        }
    }
    else if (cancoder_)
    {
        cancoder_.reset();
    }
    // tracer.stop("talonfxpro cancoder check");
    // tracer.start("talonfxpro gz sim");
    
    if (gazebo_joint_)
    {
        const double position = gazebo_joint_->Position(0);
        const double velocity = gazebo_joint_->GetVelocity(0);
        // get motor voltage, us as input to motor model to get torque out
        const double motor_voltage = state_->getMotorVoltage();
        ROS_INFO_STREAM_THROTTLE(2, "Gazebo : " << " Joint Name " << "p =" << position << " v = " << velocity << " voltage = " << motor_voltage);
        //gazebo_joint_->SetPosition(0, counter_);
        //counter_++;
        // Call gazebo_joint_->SetForce() with the torque calc'd from the motor model
    }

    // TODO DO NOT UPDATE SIMULATION STATE IF UNDER A SIMULATOR DEVICE
    // the writes will occasionally conflict

    auto &sim_state = talonfxpro_->GetSimState();
    // Note - since all of these are setting raw rotor positions but setpoints
    // are relative to mechanism positions, need to multiply the values written
    // to the raw positions/velocities by the sensor to mechanism ratio
    // TODO - maybe also rotor to sensor ratio?

    if (gazebo_joint_)
    {
        gazebo_joint_->SetPosition(0, state_->getRotorPosition()); // always set position
    }
    // tracer.stop("talonfxpro gz sim");

    // tracer.start("talonfxpro cancoder read");
    double cancoder_invert_ = 1.0;
    double cancoder_offset_ = 0.0;
    if (cancoder_ && !read_cancoder_configs_)
    {
        ctre::phoenix6::configs::MagnetSensorConfigs magnet_configs;
        cancoder_->GetConfigurator().Refresh(magnet_configs);
        cancoder_invert_ = magnet_configs.SensorDirection == ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive ? -1.0 : 1.0;
        cancoder_offset_ = units::radian_t{units::turn_t{magnet_configs.MagnetOffset}}.value();
        read_cancoder_configs_ = true;
    }

    const double invert = state_->getInvert() == hardware_interface::talonfxpro::Inverted::Clockwise_Positive ? -1.0 : 1.0;
    // tracer.stop("talonfxpro cancoder read");

    // Set simulation state supply voltages
    sim_state.SetSupplyVoltage(battery_voltage);
    if (cancoder_) { cancoder_->GetSimState().SetSupplyVoltage(battery_voltage); }

    // Update our motor state from simulation state
    state_->setMotorVoltage(sim_state.GetMotorVoltage().value());
    state_->setDutyCycle(sim_state.GetMotorVoltage() / battery_voltage);
    state_->setSupplyCurrent(sim_state.GetSupplyCurrent().value());
    state_->setTorqueCurrent(sim_state.GetTorqueCurrent().value());
}

bool SimTalonFXProDevice::setSimLimitSwitches(const bool forward_limit, const bool reverse_limit)
{
    auto &sim_state = talonfxpro_->GetSimState();
    sim_state.SetForwardLimit(forward_limit);
    sim_state.SetReverseLimit(reverse_limit);
    return true;
}

bool SimTalonFXProDevice::setSimCurrent(const double /*stator_current*/, const double /*supply_current*/)
{
    ROS_ERROR_STREAM("Error settings sim current on TalonFXPro device " << getName() << " : Not supported");
    return false;
}

bool SimTalonFXProDevice::gazeboInit(boost::shared_ptr<gazebo::physics::Model> parent_model)
{
    ROS_INFO_STREAM("Connecting TalonFXPro motor " << getName() << " to Gazebo");
    gazebo_joint_ = parent_model->GetJoint(getName());
    if (!gazebo_joint_)
    {
        ROS_ERROR_STREAM("Joint " << getName() << " not found in Gazebo");
        return false;
    }
    return true;
}
