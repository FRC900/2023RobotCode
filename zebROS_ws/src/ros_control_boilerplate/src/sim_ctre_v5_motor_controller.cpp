#include <ros/node_handle.h>

#include "gazebo_ros_control/robot_hw_sim.h"

#include <ctre/phoenix/motorcontrol/IMotorController.h>
#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"
#include "ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h"
#include "ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h"
#include "ctre_interfaces/talon_state_interface.h"

#include "ros_control_boilerplate/sim_ctre_v5_motor_controller.h"
#include "ros_control_boilerplate/get_conversion_factor.h"

SimCTREV5MotorController::SimCTREV5MotorController(const std::string &name_space,
                                                   const int joint_index,
                                                   const std::string &joint_name,
                                                   const std::string &joint_type,
                                                   const int can_id,
                                                   const std::string &can_bus,
                                                   const bool local,
                                                   const double read_hz_)
    : CTREV5MotorController(name_space, joint_index, joint_name, joint_type, can_id, can_bus, local, read_hz_)
{
}


// These always come from the main write thread
void SimCTREV5MotorController::setSimCollection(int position,
                                                int velocity,
                                                int delta_position) const
{
	if (talon_fx_)
    {
		setSimCollectionTalonFX(position, velocity, delta_position);
    }
	else if (talon_srx_)
    {
		setSimCollectionTalonSRX(position, velocity, delta_position);
    }
}

void SimCTREV5MotorController::setSimCollectionTalonFX(int position,
                                                       int velocity,
                                                       int delta_position) const
{
	auto &collection = talon_fx_->GetSimCollection();
	if (delta_position)
    {
		collection.AddIntegratedSensorPosition(delta_position);
    }
	else
    {
		collection.SetIntegratedSensorRawPosition(position);
    }
	collection.SetIntegratedSensorVelocity(velocity);
	collection.SetBusVoltage(12.5);
}

void SimCTREV5MotorController::setSimCollectionTalonSRX(int position,
                                                        int velocity,
                                                        int delta_position) const
{
	auto &collection = talon_srx_->GetSimCollection();
	if (delta_position)
    {
		collection.AddQuadraturePosition(delta_position);
    }
	else
    {
		collection.SetQuadratureRawPosition(position);
    }
	collection.SetQuadratureVelocity(velocity);
	collection.SetBusVoltage(12.5);
}


// This is the main function which adds sim support to this motor controller
// For now, it does a physics-less sim of a single controller. Depending on
// the motor mode, it copies setpoints directly into motor position and velocity
// state. This mimics a perfectly behaved system, which is typically good
// enough for basic functional testing of our code
//
// TODO - gazebo support.  This function could trigger off gazebo_joint_ being
// defined and use that to run a physics sim. The code would do two main things
// - read position and velocity from the gazebo sim joint and write it 
//   to the motor controller sim code
// - read motor voltage from ctre sim, calculate the torque generated by that
//   voltage, and write that value to the gazebo joint
//   see https://github.com/nilseuropa/gazebo_ros_motors for an example of
//   how to do that ... potentially borrow their code, or maybe 
//   https://github.com/snobotsim/SnobotSim/blob/master/snobot_sim/src/main/native/cpp/SnobotSim/MotorSim/DcMotorModel.cpp
//   See https://www.reca.lc/motors for constants for various FRC motors
void SimCTREV5MotorController::updateSimValues(const ros::Time &/*time*/, const ros::Duration &period)
{
    if (!talon_srx_ && !talon_fx_)
    {
        return;
    }
    hardware_interface::FeedbackDevice encoder_feedback = state_->getEncoderFeedback();
    const int encoder_ticks_per_rotation = state_->getEncoderTicksPerRotation();
    const double conversion_factor = state_->getConversionFactor();
    const double radians_scale = getConversionFactor(
                                     encoder_ticks_per_rotation,
                                     encoder_feedback,
                                     hardware_interface::TalonMode_Position) *
                                 conversion_factor;

    const double radians_per_second_scale = getConversionFactor(
                                                encoder_ticks_per_rotation,
                                                encoder_feedback,
                                                hardware_interface::TalonMode_Velocity) *
                                            conversion_factor;

    // Get current mode of the motor controller.
    const auto mode = state_->getTalonMode();

    // Set the encoder position based on the current motor mode.
    // auto &sim_motor = ctre_mc->GetSimCollection();
    if (mode == hardware_interface::TalonMode_Position)
    {
        // Set encoder position to set point.
        // Set velocity to 0.
        setSimCollection(static_cast<int>(state_->getSetpoint() / radians_scale), 0);
    }
    else if (mode == hardware_interface::TalonMode_Velocity)
    {
        // Set velocity to set point
        // Set encoder position to current position + velocity * dt
        setSimCollection(0,
                         static_cast<int>(state_->getSetpoint() / radians_per_second_scale),
                         static_cast<int>(state_->getSpeed() * radians_per_second_scale * period.toSec()));
    }
    else if (mode == hardware_interface::TalonMode_MotionMagic)
    {
        // TODO : maybe apply this to velocity and position PID modes above,
        // waiting on a motor where we actually need this function to test with
        const double invert = state_->getInvert() ? -1.0 : 1.0;
        // Do some ~magic~ to figure out position/velocity.
        setSimCollection(static_cast<int>(invert * state_->getActiveTrajectoryPosition() / radians_scale),
                         static_cast<int>(invert * state_->getActiveTrajectoryVelocity() / radians_per_second_scale));
    }
}

bool SimCTREV5MotorController::setSimLimitSwitches(const bool forward_limit, const bool reverse_limit)
{
    if (!local_)
    {
        ROS_ERROR_STREAM("Couldn't set sim limit switches for CTRE V5 MC " << getName() << " : device not local to this hardware interface");
        return false;
    }
    if (talon_srx_)
    {
        auto &collection = talon_srx_->GetSimCollection();
        collection.SetLimitFwd(forward_limit);
        collection.SetLimitRev(reverse_limit);
        return true;
    }
    if (talon_fx_)
    {
        auto &collection = talon_fx_->GetSimCollection();
        collection.SetLimitFwd(forward_limit);
        collection.SetLimitRev(reverse_limit);
        return true;
    }
    ROS_ERROR_STREAM("Couldn't set sim limit switches for CTRE V5 MC " << getName() << " : device not an FX or SRX");
    return false;
}

bool SimCTREV5MotorController::setSimCurrent(const double stator_current, const double supply_current)
{
    if (!local_)
    {
        ROS_ERROR_STREAM("Couldn't set sim current for CTRE V5 MC " << getName() << " : device not local to this hardware interface");
        return false;
    }
    if (talon_srx_)
    {
        auto &collection = talon_srx_->GetSimCollection();
        collection.SetStatorCurrent(stator_current);
        collection.SetSupplyCurrent(supply_current);
        return true;
    }
    if (talon_fx_)
    {
        auto &collection = talon_fx_->GetSimCollection();
        collection.SetStatorCurrent(stator_current);
        collection.SetSupplyCurrent(supply_current);
        return true;
    }
    ROS_ERROR_STREAM("Couldn't set sim current for CTRE V5 MC " << getName() << " : device not an FX or SRX");
    return false;
}

bool SimCTREV5MotorController::gazeboInit(boost::shared_ptr<gazebo::physics::Model> parent_model)
{
    if (local_)
    {
        ROS_INFO_STREAM("Connecting CTREV5 motor " << getName() << " to Gazebo");
        gazebo_joint_ = parent_model->GetJoint(getName());
        if (!gazebo_joint_)
        {
            ROS_ERROR_STREAM("Joint " << getName() << " not found in Gazebo");
            return false;
        }
    }
    return true;
}

// TODO - roll this code into updateSimValues above, remove this
// method from this, the base class, and frcrobot_gazebosim_interface.cpp
void SimCTREV5MotorController::gazeboRead()
{
    if (local_ && gazebo_joint_)
    {
        // Gazebo has an interesting API...
#if GAZEBO_MAJOR_VERSION >= 8
        const double position = gazebo_joint_->Position(0);
#else
        const double position = gazebo_joint_->GetAngle(0).Radian();
#endif
        state_->setPosition(position);
        // TODO : velocity mode?
        if (state_->getTalonMode() != hardware_interface::TalonMode_MotionMagic)
        {
            state_->setSpeed(gazebo_joint_->GetVelocity(0));
        }

#if 0
        if (joint_types_[j] == urdf::Joint::PRISMATIC)
        {
            joint_position_[j] = position;
        }
        else
        {
            joint_position_[j] += angles::shortest_angular_distance(joint_position_[j],
                    position);
        }
        joint_velocity_[j] = sim_joints_ctre_mcs_[j]->GetVelocity(0);
#endif
    }
}

// TODO - roll this code into updateSimValues above, remove this
// method from this, the base class, and frcrobot_gazebosim_interface.cpp
// Also, if we're just writing torque this can be (hopefully) greatly
// simplified.
// TODO - and then once that is done, can this be abstracted into 
// a simple class, and then just make an object of that class type
// a member of this, the V6 stuff and the REV code as well?
void SimCTREV5MotorController::gazeboWrite(const bool e_stop_active)
{
    if (local_ && gazebo_joint_)
    {
        const hardware_interface::TalonMode simulate_mode = state_->getTalonMode();
        double position;
        double velocity;
        bool set_position = false;
        bool set_velocity = false;
        if (simulate_mode == hardware_interface::TalonMode_Position)
        {
            // Assume instant velocity
            position = state_->getSetpoint();
            set_position = true;
            velocity = 0;
            set_velocity = true;
        }
        else if (simulate_mode == hardware_interface::TalonMode_Velocity)
        {
            // Assume instant acceleration for now
            set_position = false;
            velocity = state_->getSetpoint();
            set_velocity = true;
        }
        else if (simulate_mode == hardware_interface::TalonMode_MotionMagic)
        {
            position = state_->getPosition();
            set_position = true;
            velocity = state_->getSpeed();
            set_velocity = true;
        }
        else if (simulate_mode == hardware_interface::TalonMode_Disabled)
        {
            set_position = false;
            velocity = 0;
            set_velocity = true;
        }
        else if (simulate_mode == hardware_interface::TalonMode_PercentOutput)
        {
            position = state_->getPosition();
            set_position = true;
            velocity = state_->getSpeed();
            set_velocity = true;
        }
#if 0
        ROS_INFO_STREAM("talon " << getName() << 
                " setpoint=" << state_->getSetpoint() <<
                " pos=" << position <<
                " set_position=" << set_position <<
                " velocity=" << velocity <<
                " set_velocity=" << set_velocity <<
                " e_stop_active=" << e_stop_active);
#endif
        if (set_position)
        {
            gazebo_joint_->SetPosition(0, position, true);
        }
        if (set_velocity)
        {
            // if (physics_type_.compare("ode") == 0)
            //{
            //		sim_joints_ctre_mcs_[i]->SetParam("vel", 0, e_stop_active_ ? 0 : velocity);
            // }
            // else
            {
                gazebo_joint_->SetVelocity(0, e_stop_active ? 0 : velocity);
            }
        }
    }
}