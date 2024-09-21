#ifndef SIMULATOR_INTERFACE_FLYWHEEL_SIMULATOR_H_
#define SIMULATOR_INTERFACE_FLYWHEEL_SIMULATOR_H_
#include "simulator_interface/simulator_base.h"
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/DCMotor.h>
#include "wpimath/MathShared.h"

// THIS WILL NOT BUILD ON NORMAL WPILIB BECAUSE OF:
// 2023RobotCode/allwpilib/wpilibc/src/main/native/include/frc/simulation/LinearSystemSim.h:140 (GetInputVoltage)
/*
Vectord<Inputs> ClampInput(Vectord<Inputs> u) {
    return frc::DesaturateInputVector<Inputs>(
        u, frc::RobotController::GetInputVoltage());
  }

For testing, I am replacing frc::RobotController::GetInputVoltage() with a hardcoded value of 12.5V
*/

namespace general_simulators
{
  class FlywheelSimulator : public simulator_base::Simulator
  {
    public:
      FlywheelSimulator() {

      }

      void init(const XmlRpc::XmlRpcValue &simulator_info) override {
        // Get parameters from the parameter server in our namespace
        // Need: gearing and moment of inertia in kg m^2
        double gearing = simulator_info["gearing"];
        double moment_of_inertia = simulator_info["moment_of_inertia"];

        // Create a DCMotor object for the flywheel.
        // If we're not using Krakens here, we're doing something wrong :)
        frc::DCMotor motor = frc::DCMotor::KrakenX60(simulator_info["joints"].size());

        // Create a FlywheelSim object
        flywheel_sim_ = std::make_unique<frc::sim::FlywheelSim>(motor, gearing, units::kilogram_square_meter_t{moment_of_inertia});
        ROS_INFO_STREAM("Created flywheel sim");
      }

      void update(const std::string &name, const ros::Time &time, const ros::Duration &period, ctre::phoenix6::sim::TalonFXSimState &state) override {
        // ROS_INFO_STREAM("About to get motor voltage");
        // Get the motor voltage from the state
        auto motor_voltage = state.GetMotorVoltage();

        // ROS_INFO_STREAM("WPILib updates, object is " << flywheel_sim_ << " , motor voltage is " << motor_voltage.value() << "V");
        // Update the flywheel simulation
        flywheel_sim_->SetInputVoltage(motor_voltage);
        flywheel_sim_->Update(units::second_t{period.toSec()});

        // ROS_INFO_STREAM("WPILib outputs");
        // Get output angular velocity
        auto angular_velocity = flywheel_sim_->GetAngularVelocity();

        // ROS_INFO_STREAM("Write back to state");
        // Set the flywheel velocity of the simulated motor
        state.SetRotorVelocity(angular_velocity);

        // Add position delta
        state.AddRotorPosition(angular_velocity * units::second_t{period.toSec()});

        // ROS_INFO_STREAM("FLYWHEEL SIM IS BEING SIMMED YAYYYYYY");
      }

      ~FlywheelSimulator() override {

      }
    
    private:
      std::unique_ptr<frc::sim::FlywheelSim> flywheel_sim_;
  };

};
#endif
