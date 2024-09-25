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
    // https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/cpp/MotionMagic/src/main/cpp/sim/TalonFXSimProfile.cpp is the right way to do this
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

    switch (state_->getControlMode())
    {
    case hardware_interface::talonfxpro::TalonMode::DutyCycleOut:
        break;
    case hardware_interface::talonfxpro::TalonMode::TorqueCurrentFOC:
        break;
    case hardware_interface::talonfxpro::TalonMode::VoltageOut:
        break;
    case hardware_interface::talonfxpro::TalonMode::PositionDutyCycle:
    case hardware_interface::talonfxpro::TalonMode::PositionVoltage:
    case hardware_interface::talonfxpro::TalonMode::PositionTorqueCurrentFOC:
    {
        // tracer.start("talonfxpro position sim");
        // Gazebo crap
        if (gazebo_joint_)
        {
            gazebo_joint_->SetPosition(0, state_->getRotorPosition());
        }
        else
        {
            // Instantly get to desired position and velocity

            // Position control mode, add position delta and set velocity
            units::radian_t cancoder_position{state_->getControlPosition() * cancoder_invert_ - cancoder_offset_};
            units::radian_t position{invert * state_->getControlPosition() * state_->getSensorToMechanismRatio()};

            if (cancoder_) {
                cancoder_->GetSimState().SetRawPosition(cancoder_position);
                // cancoder_->GetSimState().SetVelocity(units::radians_per_second_t{sim_state.GetRotorVelocity()}.value() * cancoder_invert_});
            }
            sim_state.SetRawRotorPosition(position);

            // We'll also have a velocity setpoint, so set that here
            units::radians_per_second_t velocity{invert * state_->getControlVelocity() * state_->getSensorToMechanismRatio()};
            sim_state.SetRotorVelocity(velocity);
        }

        sim_state.SetSupplyVoltage(battery_voltage);
        if (cancoder_) { cancoder_->GetSimState().SetSupplyVoltage(battery_voltage); }

        // Update our motor state
        state_->setMotorVoltage(sim_state.GetMotorVoltage().value());
        state_->setDutyCycle(sim_state.GetMotorVoltage() / battery_voltage);
        state_->setSupplyCurrent(sim_state.GetSupplyCurrent().value());
        state_->setTorqueCurrent(sim_state.GetTorqueCurrent().value());
        state_->setRotorPosition(units::radian_t{talonfxpro_->GetRotorPosition().GetValue()}.value());
        // tracer.stop("talonfxpro position sim");
        
        // ROS_ERROR_STREAM("IN POSITION MODE");
        break;
    }
    case hardware_interface::talonfxpro::TalonMode::VelocityDutyCycle:
    case hardware_interface::talonfxpro::TalonMode::VelocityVoltage:
    case hardware_interface::talonfxpro::TalonMode::VelocityTorqueCurrentFOC:
    {
        // tracer.start("talonfxpro velocity sim");
        // Gazebo crap
        if (gazebo_joint_)
        {
            constexpr double kT = 0.0192; // Nm/A
            double kV = 509.2;            // convert from rpm/V to rad/s/V
            kV *= 2.0 * M_PI / 60.0;
            kV = 1.0 / kV;              // convert from (rad/s)/V to V/(rad/s)
            constexpr double R = 0.039; // ohms, resistance of motor
            constexpr double gear_ratio = 1.0;// / 6.75;
            /*
            T=kt * (V - kv*ω) / R
            */

            double torque_current = ((invert * sim_state.GetMotorVoltage().value()) - (kV * state_->getControlVelocity() * gear_ratio)) / R;

            //         idk  vvvv
            double torque = -1.0 * kT * torque_current;

            // (Nm / A) * (V - (rad/s/V * rad/s)) / (ohms) = Nm

            // gazebo_joint_->SetPosition(0, state_->getRotorPosition());
            gazebo_joint_->SetForce(0, torque);
            sim_state.SetRotorVelocity(units::angular_velocity::radians_per_second_t{gazebo_joint_->GetVelocity(0)});
            sim_state.AddRotorPosition(units::radian_t{gazebo_joint_->GetVelocity(0) * period.toSec()});
            // gazebo_joint_->SetVelocity(0, state_->getControlVelocity());
            ROS_ERROR_STREAM_THROTTLE_NAMED(1, std::to_string(state_->getCANID()), "IN VELOCITY MODE " << torque << " " << sim_state.GetMotorVoltage().value() << " " << state_->getControlVelocity() << " " << state_->getSensorToMechanismRatio());
        }
        else
        {
            // Calculate velocity setpoint and position delta, applying invert and sensor : mechanism ratio
            units::angular_velocity::radians_per_second_t velocity_setpoint{invert * state_->getControlVelocity() * state_->getSensorToMechanismRatio()};
            units::radian_t delta_position{velocity_setpoint * units::second_t{period.toSec()}};

            // Velocity control mode, add position delta and set velocity
            sim_state.SetRotorVelocity(velocity_setpoint);
            sim_state.AddRotorPosition(delta_position); // VERY IMPORTANT SO CTRE SIM KNOWS MOTORS MOVE
        }

        sim_state.SetSupplyVoltage(battery_voltage);
        if (cancoder_) { cancoder_->GetSimState().SetSupplyVoltage(battery_voltage); }
        
        // Update our motor state
        state_->setMotorVoltage(sim_state.GetMotorVoltage().value());
        state_->setDutyCycle(sim_state.GetMotorVoltage() / battery_voltage);
        state_->setSupplyCurrent(sim_state.GetSupplyCurrent().value());
        state_->setTorqueCurrent(sim_state.GetTorqueCurrent().value());
        // tracer.stop("talonfxpro velocity sim");
        break;
    }
    case hardware_interface::talonfxpro::TalonMode::MotionMagicDutyCycle:
    case hardware_interface::talonfxpro::TalonMode::MotionMagicVoltage: 
    case hardware_interface::talonfxpro::TalonMode::MotionMagicExpoVoltage:
    {
        // tracer.start("talonfxpro motion magic sim");
        // Gazebo crap
        if (gazebo_joint_)
        {
            const double invert = state_->getInvert() == hardware_interface::talonfxpro::Inverted::Clockwise_Positive ? -1.0 : 1.0;
            constexpr double kT = 0.0192; // Nm/A
            double kV = 509.2;            // convert from rpm/V to rad/s/V
            kV *= 2.0 * M_PI / 60.0;
            kV = 1.0 / kV;              // convert from (rad/s)/V to V/(rad/s)
            constexpr double R = 0.039; // ohms, resistance of motor
            constexpr double gear_ratio = 1.0;// / 6.75;
            /*
            T=kt * (V - kv*ω) / R
            */

            double torque_current = ((invert * sim_state.GetMotorVoltage().value()) - (kV * state_->getControlVelocity() * gear_ratio)) / R;

            //         idk  vvvv
            double torque = -1.0 * kT * torque_current;

            // (Nm / A) * (V - (rad/s/V * rad/s)) / (ohms) = Nm

            // gazebo_joint_->SetPosition(0, state_->getRotorPosition());
            gazebo_joint_->SetForce(0, torque);
            // Velocity control mode, add position delta and set velocity
            sim_state.SetRotorVelocity(units::angular_velocity::radians_per_second_t{gazebo_joint_->GetVelocity(0)});
            if (cancoder_) { cancoder_->GetSimState().SetRawPosition(units::radian_t{state_->getRotorPosition() * cancoder_invert_ - cancoder_offset_ - M_PI / 2}); }
            sim_state.AddRotorPosition(units::radian_t{gazebo_joint_->GetVelocity(0) * period.toSec()});
            // gazebo_joint_->SetVelocity(0, state_->getControlVelocity());
            ROS_ERROR_STREAM_THROTTLE_NAMED(1, std::to_string(state_->getCANID()), "IN MOTION MAGIC MODE " << torque << " " << sim_state.GetMotorVoltage().value() << " " << state_->getControlVelocity() << " " << state_->getSensorToMechanismRatio());
        }
        else
        {
            // Motion magic, controls both position and velocity
            units::radian_t position{invert * state_->getClosedLoopReference() * state_->getSensorToMechanismRatio()};
            units::radian_t cancoder_position{(state_->getClosedLoopReference() - cancoder_offset_) * cancoder_invert_}; // It seems like we should need to divide by rotor to sensor ratio here, but doing that makes it not work
            units::angular_velocity::radians_per_second_t velocity{state_->getClosedLoopReferenceSlope() * state_->getSensorToMechanismRatio()};
            
            sim_state.AddRotorPosition(invert * velocity * units::second_t{period.toSec()} * state_->getRotorToSensorRatio());
            sim_state.SetRotorVelocity(velocity * state_->getRotorToSensorRatio());

            // Set rotor position and velocity
            if (cancoder_) {
                auto cancoder_velocity = talonfxpro_->GetRotorVelocity().GetValue() / state_->getRotorToSensorRatio() * cancoder_invert_;
                cancoder_->GetSimState().SetVelocity(cancoder_velocity);
                cancoder_->GetSimState().AddPosition(cancoder_velocity * units::second_t{period.toSec()});
            }
        }

        sim_state.SetSupplyVoltage(battery_voltage);
        if (cancoder_) { cancoder_->GetSimState().SetSupplyVoltage(battery_voltage); }

        // Update our motor state
        state_->setMotorVoltage(sim_state.GetMotorVoltage().value());
        state_->setDutyCycle(sim_state.GetMotorVoltage() / battery_voltage);
        state_->setSupplyCurrent(sim_state.GetSupplyCurrent().value());
        state_->setTorqueCurrent(sim_state.GetTorqueCurrent().value());
        // tracer.stop("talonfxpro motion magic sim");
        break;
    }
    case hardware_interface::talonfxpro::TalonMode::MotionMagicTorqueCurrentFOC:
    case hardware_interface::talonfxpro::TalonMode::MotionMagicVelocityDutyCycle:
    case hardware_interface::talonfxpro::TalonMode::MotionMagicVelocityVoltage:
    case hardware_interface::talonfxpro::TalonMode::MotionMagicVelocityTorqueCurrentFOC:
    case hardware_interface::talonfxpro::TalonMode::DynamicMotionMagicDutyCycle:
    case hardware_interface::talonfxpro::TalonMode::DynamicMotionMagicVoltage:
    case hardware_interface::talonfxpro::TalonMode::DynamicMotionMagicTorqueCurrentFOC:
    {
        // TODO : debug, check sim Orientation field

        // Motion magic, controls both position and velocity
        units::radian_t target_position{invert * state_->getClosedLoopReference() * state_->getSensorToMechanismRatio()};
        units::radian_t cancoder_target_position{cancoder_invert_ * state_->getClosedLoopReference() - cancoder_offset_};
        units::angular_velocity::radians_per_second_t target_velocity{invert * state_->getClosedLoopReferenceSlope() * state_->getSensorToMechanismRatio()};

        // Set rotor position and velocity
        sim_state.AddRotorPosition(target_position - units::radian_t{state_->getRotorPosition()});
        if (cancoder_) { cancoder_->GetSimState().SetRawPosition(cancoder_target_position); }
        sim_state.SetRotorVelocity(target_velocity);

        sim_state.SetSupplyVoltage(battery_voltage);
        if (cancoder_) { cancoder_->GetSimState().SetSupplyVoltage(battery_voltage); }

        // Update our motor state
        state_->setMotorVoltage(sim_state.GetMotorVoltage().value());
        state_->setDutyCycle(sim_state.GetMotorVoltage() / battery_voltage);
        state_->setSupplyCurrent(sim_state.GetSupplyCurrent().value());
        state_->setTorqueCurrent(sim_state.GetTorqueCurrent().value());
        state_->setRotorPosition(target_position.value()); // unrealistic but idk how to make it better right now?
        break;
    }
    case hardware_interface::talonfxpro::TalonMode::Follower:
        break;
    case hardware_interface::talonfxpro::TalonMode::StrictFollower:
        break;
    case hardware_interface::talonfxpro::TalonMode::NeutralOut:
        break;
    case hardware_interface::talonfxpro::TalonMode::CoastOut:
        break;
    case hardware_interface::talonfxpro::TalonMode::StaticBrake:
        break;
    // TODO : support differential modes, somehow
    }
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
