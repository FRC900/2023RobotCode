#include "ros/node_handle.h"
#include "ctre/phoenix6/core/CoreTalonFX.hpp"
#include "ros_control_boilerplate/talonfxpro_device.h"
#include "ros_control_boilerplate/tracer.h"
#include "ctre_interfaces/talonfxpro_command_interface.h"

TalonFXProDevice::TalonFXProDevice(const std::string &name_space,
                                   const int joint_index,
                                   const std::string &joint_name,
                                   const int can_id,
                                   const std::string &can_bus,
                                   double read_hz)
: CTREV6Device("TalonFXPro", joint_name, can_id)
, can_bus_{can_bus}
, state_{std::make_unique<hardware_interface::talonfxpro::TalonFXProHWState>(can_id)}
, command_{std::make_unique<hardware_interface::talonfxpro::TalonFXProHWCommand>()}
, read_state_mutex_{std::make_unique<std::mutex>()}
, read_thread_state_{std::make_unique<hardware_interface::talonfxpro::TalonFXProHWState>(can_id)}
, talonfxpro_{std::make_unique<ctre::phoenix6::hardware::core::CoreTalonFX>(can_id, can_bus)}
, read_thread_{std::make_unique<std::thread>(&TalonFXProDevice::read_thread, this,
                                             std::make_unique<Tracer>("tfxpro_read_" + joint_name + " " + name_space),
                                             joint_index,
                                             read_hz)}
{
		ROS_INFO_STREAM_NAMED("frc_robot_interface",
							  "Loading joint " << joint_index << "=" << joint_name <<
							  " as TalonFXpro CAN id " << can_id << " on CAN bus \"" << can_bus << "\"");

        // Clear this out so we only get resets that occur after the controllers have been initialized
        static_cast<void>(talonfxpro_->HasResetOccurred());
        setParentDevice(talonfxpro_.get());
}

TalonFXProDevice::~TalonFXProDevice()
{
    if (read_thread_ && read_thread_->joinable())
    {
        read_thread_->join();
    }
}

void TalonFXProDevice::registerInterfaces(hardware_interface::talonfxpro::TalonFXProStateInterface &state_interface,
                                          hardware_interface::talonfxpro::TalonFXProCommandInterface &command_interface)
{
    ROS_INFO_STREAM("FRCRobotInterface: Registering interface for TalonFXPro : " << getName() << " at hw ID " << getId() << " on bus " << can_bus_);
    hardware_interface::talonfxpro::TalonFXProStateHandle state_handle(getName(), state_.get());
    hardware_interface::talonfxpro::TalonFXProCommandHandle command_handle(state_handle, command_.get());
    state_interface.registerHandle(state_handle);
    command_interface.registerHandle(command_handle);
}

void TalonFXProDevice::read(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    std::unique_lock<std::mutex> l(*read_state_mutex_, std::try_to_lock);
    if (!l.owns_lock())
    {
        return;
    }
    // Pass changes to enable read thread setting into the state
    // object checked by that thread
    read_thread_state_->setEnableReadThread(state_->getEnableReadThread());
    read_thread_state_->setDifferentialSensorSource(state_->getDifferentialSensorSource());

    // Copy the most recent state values from the read thread into the
    // one used in the main thread
    state_->setVersionMajor(read_thread_state_->getVersionMajor());
    state_->setVersionMinor(read_thread_state_->getVersionMinor());
    state_->setVersionBugfix(read_thread_state_->getVersionBugfix());
    state_->setVersionBuild(read_thread_state_->getVersionBuild());

    state_->setForwardLimit(read_thread_state_->getForwardLimit());
    state_->setReverseLimit(read_thread_state_->getReverseLimit());

    state_->setAppliedRotorPolarity(read_thread_state_->getAppliedRotorPolarity());

    state_->setDutyCycle(read_thread_state_->getDutyCycle());
    state_->setTorqueCurrent(read_thread_state_->getTorqueCurrent());
    state_->setStatorCurrent(read_thread_state_->getStatorCurrent());
    state_->setSupplyCurrent(read_thread_state_->getSupplyCurrent());
    state_->setSupplyVoltage(read_thread_state_->getSupplyVoltage());
    state_->setDeviceTemp(read_thread_state_->getDeviceTemp());
    state_->setProcessorTemp(read_thread_state_->getProcessorTemp());

    state_->setRotorVelocity(read_thread_state_->getRotorVelocity());
    state_->setRotorPosition(read_thread_state_->getRotorPosition());
    state_->setVelocity(read_thread_state_->getVelocity());
    state_->setPosition(read_thread_state_->getPosition());

    state_->setMotionMagicIsRunning(read_thread_state_->getMotionMagicIsRunning());
    state_->setDeviceEnable(read_thread_state_->getDeviceEnable());

    state_->setDifferentialControlMode(read_thread_state_->getDifferentialControlMode());
    state_->setDifferentialAverageVelocity(read_thread_state_->getDifferentialAverageVelocity());
    state_->setDifferentialAveragePosition(read_thread_state_->getDifferentialAveragePosition());
    state_->setDifferentialDifferenceVelocity(read_thread_state_->getDifferentialDifferenceVelocity());
    state_->setDifferentialDifferencePosition(read_thread_state_->getDifferentialDifferencePosition());
    state_->setBridgeOutput(read_thread_state_->getBridgeOutput());

    state_->setFaultHardware(read_thread_state_->getFaultHardware());
    state_->setFaultProcTemp(read_thread_state_->getFaultProcTemp());
    state_->setFaultDeviceTemp(read_thread_state_->getFaultDeviceTemp());
    state_->setFaultUndervoltage(read_thread_state_->getFaultUndervoltage());
    state_->setFaultBootDuringEnable(read_thread_state_->getFaultBootDuringEnable());
    state_->setFaultUnlicensedFeatureInUse(read_thread_state_->getFaultUnlicensedFeatureInUse());
    state_->setFaultRemoteSensorReset(read_thread_state_->getFaultRemoteSensorReset());
    state_->setFaultMissingDifferentialFX(read_thread_state_->getFaultMissingDifferentialFX());
    state_->setFaultRemoteSensorPosOverfow(read_thread_state_->getFaultRemoteSensorPosOverfow());
    state_->setFaultOverSupplyV(read_thread_state_->getFaultOverSupplyV());
    state_->setFaultUnstableSupplyV(read_thread_state_->getFaultUnstableSupplyV());
    state_->setFaultReverseHardLimit(read_thread_state_->getFaultReverseHardLimit());
    state_->setFaultForwardHardLimit(read_thread_state_->getFaultForwardHardLimit());
    state_->setFaultReverseSoftLimit(read_thread_state_->getFaultReverseSoftLimit());
    state_->setFaultForwardSoftLimit(read_thread_state_->getFaultForwardSoftLimit());
    state_->setFaultMissingRemoteSensor(read_thread_state_->getFaultMissingRemoteSensor());
    state_->setFaultFusedSensorOutOfSync(read_thread_state_->getFaultFusedSensorOutOfSync());
    state_->setFaultStatorCurrLimit(read_thread_state_->getFaultStatorCurrLimit());
    state_->setFaultSupplyCurrLimit(read_thread_state_->getFaultSupplyCurrLimit());

    state_->setStickyFaultHardware(read_thread_state_->getStickyFaultHardware());
    state_->setStickyFaultProcTemp(read_thread_state_->getStickyFaultProcTemp());
    state_->setStickyFaultDeviceTemp(read_thread_state_->getStickyFaultDeviceTemp());
    state_->setStickyFaultUndervoltage(read_thread_state_->getStickyFaultUndervoltage());
    state_->setStickyFaultBootDuringEnable(read_thread_state_->getStickyFaultBootDuringEnable());
    state_->setStickyFaultUnlicensedFeatureInUse(read_thread_state_->getStickyFaultUnlicensedFeatureInUse());
    state_->setStickyFaultRemoteSensorReset(read_thread_state_->getStickyFaultRemoteSensorReset());
    state_->setStickyFaultMissingDifferentialFX(read_thread_state_->getStickyFaultMissingDifferentialFX());
    state_->setStickyFaultRemoteSensorPosOverfow(read_thread_state_->getStickyFaultRemoteSensorPosOverfow());
    state_->setStickyFaultOverSupplyV(read_thread_state_->getStickyFaultOverSupplyV());
    state_->setStickyFaultUnstableSupplyV(read_thread_state_->getStickyFaultUnstableSupplyV());
    state_->setStickyFaultReverseHardLimit(read_thread_state_->getStickyFaultReverseHardLimit());
    state_->setStickyFaultForwardHardLimit(read_thread_state_->getStickyFaultForwardHardLimit());
    state_->setStickyFaultReverseSoftLimit(read_thread_state_->getStickyFaultReverseSoftLimit());
    state_->setStickyFaultForwardSoftLimit(read_thread_state_->getStickyFaultForwardSoftLimit());
    state_->setStickyFaultMissingRemoteSensor(read_thread_state_->getStickyFaultMissingRemoteSensor());
    state_->setStickyFaultFusedSensorOutOfSync(read_thread_state_->getStickyFaultFusedSensorOutOfSync());
    state_->setStickyFaultStatorCurrLimit(read_thread_state_->getStickyFaultStatorCurrLimit());
    state_->setStickyFaultSupplyCurrLimit(read_thread_state_->getStickyFaultSupplyCurrLimit());

    state_->setClosedLoopProportionalOutput(read_thread_state_->getClosedLoopProportionalOutput());
    state_->setClosedLoopIntegratedOutput(read_thread_state_->getClosedLoopIntegratedOutput());
    state_->setClosedLoopFeedForward(read_thread_state_->getClosedLoopFeedForward());
    state_->setClosedLoopDerivativeOutput(read_thread_state_->getClosedLoopDerivativeOutput());
    state_->setClosedLoopOutput(read_thread_state_->getClosedLoopOutput());
    state_->setClosedLoopReference(read_thread_state_->getClosedLoopReference());
    state_->setClosedLoopReferenceSlope(read_thread_state_->getClosedLoopReferenceSlope());
    state_->setClosedLoopError(read_thread_state_->getClosedLoopError());

    state_->setDifferentialOutput(read_thread_state_->getDifferentialOutput());
    state_->setDifferentialClosedLoopProportionalOutput(read_thread_state_->getDifferentialClosedLoopProportionalOutput());
    state_->setDifferentialClosedLoopIntegratedOutput(read_thread_state_->getDifferentialClosedLoopIntegratedOutput());
    state_->setDifferentialClosedLoopFeedForward(read_thread_state_->getDifferentialClosedLoopFeedForward());
    state_->setDifferentialClosedLoopDerivativeOutput(read_thread_state_->getDifferentialClosedLoopDerivativeOutput());
    state_->setDifferentialClosedLoopOutput(read_thread_state_->getDifferentialClosedLoopOutput());
    state_->setDifferentialClosedLoopReference(read_thread_state_->getDifferentialClosedLoopReference());
    state_->setDifferentialClosedLoopReferenceSlope(read_thread_state_->getDifferentialClosedLoopReferenceSlope());
    state_->setDifferentialClosedLoopError(read_thread_state_->getDifferentialClosedLoopError());
}

#define SAFE_READ(var, function) \
const auto var = safeRead(function, #function); \
if (!var) { tracer->stop() ; continue; }

#define SAFE_READ_INTO(var, function) \
const auto foo##var = safeRead(function, #function); \
if (!foo##var) { tracer->stop() ; continue; } \
var = *foo##var;

void TalonFXProDevice::read_thread(std::unique_ptr<Tracer> tracer,
                                   const int joint_idx,
                                   const double poll_frequency)
{
#ifdef __linux__
    std::stringstream thread_name;
    // Use abbreviations since pthread_setname will fail if name is >= 16 characters
    thread_name << "tfxpro_rd_" << read_thread_state_->getCANID();
    if (pthread_setname_np(pthread_self(), thread_name.str().c_str()))
    {
        ROS_ERROR_STREAM("Error setting thread name " << thread_name.str() << " " << errno);
    }
#endif
	ros::Duration(3.25 + joint_idx * 0.05).sleep(); // Sleep for a few seconds to let CAN start up
	ROS_INFO_STREAM("Starting talonfxpro " << read_thread_state_->getCANID() << " thread at " << ros::Time::now());
	for (ros::Rate r(poll_frequency); ros::ok(); r.sleep())
	{
		tracer->start("talonfxpro read main_loop");
        
        hardware_interface::talonfxpro::DifferentialSensorSource differential_sensor_source{hardware_interface::talonfxpro::DifferentialSensorSource::Disabled};
		{
			std::lock_guard<std::mutex> l(*read_state_mutex_);
			if (!read_thread_state_->getEnableReadThread())
            {
				return;
            }
            differential_sensor_source = read_thread_state_->getDifferentialSensorSource();
		}
        const bool in_differential_mode = differential_sensor_source != hardware_interface::talonfxpro::DifferentialSensorSource::Disabled;

        SAFE_READ(version_major, talonfxpro_->GetVersionMajor());
        SAFE_READ(version_minor, talonfxpro_->GetVersionMinor());
        SAFE_READ(version_bugfix, talonfxpro_->GetVersionBugfix());
        SAFE_READ(version_build, talonfxpro_->GetVersionBuild());

        SAFE_READ(forward_limit, talonfxpro_->GetForwardLimit());
        SAFE_READ(reverse_limit, talonfxpro_->GetReverseLimit());

        SAFE_READ(ctre_applied_rotor_polarity, talonfxpro_->GetAppliedRotorPolarity());
        hardware_interface::talonfxpro::Inverted applied_rotor_polarity{-1};
        if (ctre_applied_rotor_polarity == ctre::phoenix6::signals::AppliedRotorPolarityValue::PositiveIsCounterClockwise)
        {
            applied_rotor_polarity = hardware_interface::talonfxpro::Inverted::CounterClockwise_Positive;
        }
        else if (ctre_applied_rotor_polarity == ctre::phoenix6::signals::AppliedRotorPolarityValue::PositiveIsClockwise)
        {
            applied_rotor_polarity = hardware_interface::talonfxpro::Inverted::Clockwise_Positive;
        }

        SAFE_READ(duty_cycle, talonfxpro_->GetDutyCycle());
        SAFE_READ(torque_current, talonfxpro_->GetTorqueCurrent());
        SAFE_READ(stator_current, talonfxpro_->GetStatorCurrent());
        SAFE_READ(supply_current, talonfxpro_->GetSupplyCurrent());
        SAFE_READ(supply_voltage, talonfxpro_->GetSupplyVoltage());
        SAFE_READ(device_temp, talonfxpro_->GetDeviceTemp());
        SAFE_READ(processor_temp, talonfxpro_->GetProcessorTemp());
        SAFE_READ(rotor_velocity, talonfxpro_->GetRotorVelocity());
        SAFE_READ(rotor_position, talonfxpro_->GetRotorPosition());
        SAFE_READ(velocity, talonfxpro_->GetVelocity());
        SAFE_READ(position, talonfxpro_->GetPosition());
        SAFE_READ(device_enable, talonfxpro_->GetDeviceEnable());
        SAFE_READ(motion_magic_is_running, talonfxpro_->GetMotionMagicIsRunning());

        hardware_interface::talonfxpro::DifferentialControlMode differential_control_mode{hardware_interface::talonfxpro::DifferentialControlMode::DisabledOutput};

        units::radians_per_second_t differential_average_velocity{0};
        units::radian_t differential_average_position{0};
        units::radians_per_second_t differential_difference_velocity{0};
        units::radian_t differential_difference_position{0};

        if (in_differential_mode)
        {
            SAFE_READ(ctre_differential_control_mode, talonfxpro_->GetDifferentialControlMode());
            switch (ctre_differential_control_mode->value)
            {
            case ctre::phoenix6::signals::DifferentialControlModeValue::DisabledOutput:
                differential_control_mode = hardware_interface::talonfxpro::DifferentialControlMode::DisabledOutput;
                break;
            case ctre::phoenix6::signals::DifferentialControlModeValue::NeutralOut:
                differential_control_mode = hardware_interface::talonfxpro::DifferentialControlMode::NeutralOut;
                break;
            case ctre::phoenix6::signals::DifferentialControlModeValue::StaticBrake:
                differential_control_mode = hardware_interface::talonfxpro::DifferentialControlMode::StaticBrake;
                break;
            case ctre::phoenix6::signals::DifferentialControlModeValue::DutyCycleOut:
                differential_control_mode = hardware_interface::talonfxpro::DifferentialControlMode::DutyCycleOut;
                break;
            case ctre::phoenix6::signals::DifferentialControlModeValue::PositionDutyCycle:
                differential_control_mode = hardware_interface::talonfxpro::DifferentialControlMode::PositionDutyCycle;
                break;
            case ctre::phoenix6::signals::DifferentialControlModeValue::VelocityDutyCycle:
                differential_control_mode = hardware_interface::talonfxpro::DifferentialControlMode::VelocityDutyCycle;
                break;
            case ctre::phoenix6::signals::DifferentialControlModeValue::MotionMagicDutyCycle:
                differential_control_mode = hardware_interface::talonfxpro::DifferentialControlMode::MotionMagicDutyCycle;
                break;
            case ctre::phoenix6::signals::DifferentialControlModeValue::DutyCycleFOC:
                differential_control_mode = hardware_interface::talonfxpro::DifferentialControlMode::DutyCycleFOC;
                break;
            case ctre::phoenix6::signals::DifferentialControlModeValue::PositionDutyCycleFOC:
                differential_control_mode = hardware_interface::talonfxpro::DifferentialControlMode::PositionDutyCycleFOC;
                break;
            case ctre::phoenix6::signals::DifferentialControlModeValue::VelocityDutyCycleFOC:
                differential_control_mode = hardware_interface::talonfxpro::DifferentialControlMode::VelocityDutyCycleFOC;
                break;
            case ctre::phoenix6::signals::DifferentialControlModeValue::MotionMagicDutyCycleFOC:
                differential_control_mode = hardware_interface::talonfxpro::DifferentialControlMode::MotionMagicDutyCycleFOC;
                break;
            case ctre::phoenix6::signals::DifferentialControlModeValue::VoltageOut:
                differential_control_mode = hardware_interface::talonfxpro::DifferentialControlMode::VoltageOut;
                break;
            case ctre::phoenix6::signals::DifferentialControlModeValue::PositionVoltage:
                differential_control_mode = hardware_interface::talonfxpro::DifferentialControlMode::PositionVoltage;
                break;
            case ctre::phoenix6::signals::DifferentialControlModeValue::VelocityVoltage:
                differential_control_mode = hardware_interface::talonfxpro::DifferentialControlMode::VelocityVoltage;
                break;
            case ctre::phoenix6::signals::DifferentialControlModeValue::MotionMagicVoltage:
                differential_control_mode = hardware_interface::talonfxpro::DifferentialControlMode::MotionMagicVoltage;
                break;
            case ctre::phoenix6::signals::DifferentialControlModeValue::VoltageFOC:
                differential_control_mode = hardware_interface::talonfxpro::DifferentialControlMode::VoltageFOC;
                break;
            case ctre::phoenix6::signals::DifferentialControlModeValue::PositionVoltageFOC:
                differential_control_mode = hardware_interface::talonfxpro::DifferentialControlMode::PositionVoltageFOC;
                break;
            case ctre::phoenix6::signals::DifferentialControlModeValue::VelocityVoltageFOC:
                differential_control_mode = hardware_interface::talonfxpro::DifferentialControlMode::VelocityVoltageFOC;
                break;
            case ctre::phoenix6::signals::DifferentialControlModeValue::MotionMagicVoltageFOC:
                differential_control_mode = hardware_interface::talonfxpro::DifferentialControlMode::MotionMagicVoltageFOC;
                break;
            case ctre::phoenix6::signals::DifferentialControlModeValue::TorqueCurrentFOC:
                differential_control_mode = hardware_interface::talonfxpro::DifferentialControlMode::TorqueCurrentFOC;
                break;
            case ctre::phoenix6::signals::DifferentialControlModeValue::PositionTorqueCurrentFOC:
                differential_control_mode = hardware_interface::talonfxpro::DifferentialControlMode::PositionTorqueCurrentFOC;
                break;
            case ctre::phoenix6::signals::DifferentialControlModeValue::VelocityTorqueCurrentFOC:
                differential_control_mode = hardware_interface::talonfxpro::DifferentialControlMode::VelocityTorqueCurrentFOC;
                break;
            case ctre::phoenix6::signals::DifferentialControlModeValue::MotionMagicTorqueCurrentFOC:
                differential_control_mode = hardware_interface::talonfxpro::DifferentialControlMode::MotionMagicTorqueCurrentFOC;
                break;
            case ctre::phoenix6::signals::DifferentialControlModeValue::Follower:
                differential_control_mode = hardware_interface::talonfxpro::DifferentialControlMode::Follower;
                break;
            case ctre::phoenix6::signals::DifferentialControlModeValue::Reserved:
                differential_control_mode = hardware_interface::talonfxpro::DifferentialControlMode::Reserved;
                break;
            case ctre::phoenix6::signals::DifferentialControlModeValue::CoastOut:
                differential_control_mode = hardware_interface::talonfxpro::DifferentialControlMode::CoastOut;
                break;
            }
            SAFE_READ(ctre_differential_average_velocity, talonfxpro_->GetDifferentialAverageVelocity());
            SAFE_READ(ctre_differential_average_position, talonfxpro_->GetDifferentialAveragePosition());
            SAFE_READ(ctre_differential_difference_velocity, talonfxpro_->GetDifferentialDifferenceVelocity());
            SAFE_READ(ctre_differential_difference_position, talonfxpro_->GetDifferentialDifferencePosition());
            differential_average_velocity = *ctre_differential_average_velocity;
            differential_average_position = *ctre_differential_average_position;
            differential_difference_velocity = *ctre_differential_average_velocity;
            differential_difference_position = *ctre_differential_average_position;
        }

        SAFE_READ(ctre_bridge_output_value, talonfxpro_->GetBridgeOutput());
        hardware_interface::talonfxpro::BridgeOutput bridge_output_value;
        switch (ctre_bridge_output_value->value)
        {
            case ctre::phoenix6::signals::BridgeOutputValue::BridgeReq_Coast:
                bridge_output_value = hardware_interface::talonfxpro::BridgeOutput::Coast;
                break;
            case ctre::phoenix6::signals::BridgeOutputValue::BridgeReq_Brake:
                bridge_output_value = hardware_interface::talonfxpro::BridgeOutput::Brake;
                break;
            case ctre::phoenix6::signals::BridgeOutputValue::BridgeReq_Trapez:
                bridge_output_value = hardware_interface::talonfxpro::BridgeOutput::Trapez;
                break;
            case ctre::phoenix6::signals::BridgeOutputValue::BridgeReq_FOCTorque:
                bridge_output_value = hardware_interface::talonfxpro::BridgeOutput::FOCTorque;
                break;
            case ctre::phoenix6::signals::BridgeOutputValue::BridgeReq_MusicTone:
                bridge_output_value = hardware_interface::talonfxpro::BridgeOutput::MusicTone;
                break;
            case ctre::phoenix6::signals::BridgeOutputValue::BridgeReq_FOCEasy:
                bridge_output_value = hardware_interface::talonfxpro::BridgeOutput::FOCEasy;
                break;
            case ctre::phoenix6::signals::BridgeOutputValue::BridgeReq_FaultBrake:
                bridge_output_value = hardware_interface::talonfxpro::BridgeOutput::FaultBrake;
                break;
            case ctre::phoenix6::signals::BridgeOutputValue::BridgeReq_FaultCoast:
                bridge_output_value = hardware_interface::talonfxpro::BridgeOutput::FaultCoast;
                break;
        }

        SAFE_READ(fault_hardware, talonfxpro_->GetFault_Hardware());
        SAFE_READ(fault_proctemp, talonfxpro_->GetFault_ProcTemp());
        SAFE_READ(fault_devicetemp, talonfxpro_->GetFault_DeviceTemp());
        SAFE_READ(fault_undervoltage, talonfxpro_->GetFault_Undervoltage());
        SAFE_READ(fault_bootduringenable, talonfxpro_->GetFault_BootDuringEnable());
        SAFE_READ(fault_unlicensed_feature_in_use, talonfxpro_->GetFault_UnlicensedFeatureInUse());
        SAFE_READ(fault_remotesensorreset, talonfxpro_->GetFault_RemoteSensorReset());
        SAFE_READ(fault_missingdifferentialfx, talonfxpro_->GetFault_MissingDifferentialFX());
        SAFE_READ(fault_remotesensorposoverflow, talonfxpro_->GetFault_RemoteSensorPosOverflow());
        SAFE_READ(fault_oversupplyv, talonfxpro_->GetFault_OverSupplyV());
        SAFE_READ(fault_unstablesupplyv, talonfxpro_->GetFault_UnstableSupplyV());
        SAFE_READ(fault_reversehardlimit, talonfxpro_->GetFault_ReverseHardLimit());
        SAFE_READ(fault_forwardhardlimit, talonfxpro_->GetFault_ForwardHardLimit());
        SAFE_READ(fault_reversesoftlimit, talonfxpro_->GetFault_ReverseSoftLimit());
        SAFE_READ(fault_forwardsoftlimit, talonfxpro_->GetFault_ForwardSoftLimit());
        SAFE_READ(fault_missingremotesensor, talonfxpro_->GetFault_MissingRemoteSensor());
        SAFE_READ(fault_fusedsensoroutofsync, talonfxpro_->GetFault_FusedSensorOutOfSync());
        SAFE_READ(fault_statorcurrlimit, talonfxpro_->GetFault_StatorCurrLimit());
        SAFE_READ(fault_supplycurrlimit, talonfxpro_->GetFault_SupplyCurrLimit());

        SAFE_READ(sticky_fault_hardware, talonfxpro_->GetStickyFault_Hardware());
        SAFE_READ(sticky_fault_proctemp, talonfxpro_->GetStickyFault_ProcTemp());
        SAFE_READ(sticky_fault_devicetemp, talonfxpro_->GetStickyFault_DeviceTemp());
        SAFE_READ(sticky_fault_undervoltage, talonfxpro_->GetStickyFault_Undervoltage());
        SAFE_READ(sticky_fault_bootduringenable, talonfxpro_->GetStickyFault_BootDuringEnable());
        SAFE_READ(sticky_fault_unlicensed_feature_in_use, talonfxpro_->GetStickyFault_UnlicensedFeatureInUse());
        SAFE_READ(sticky_fault_remotesensorreset, talonfxpro_->GetStickyFault_RemoteSensorReset());
        SAFE_READ(sticky_fault_missingdifferentialfx, talonfxpro_->GetStickyFault_MissingDifferentialFX());
        SAFE_READ(sticky_fault_remotesensorposoverflow, talonfxpro_->GetStickyFault_RemoteSensorPosOverflow());
        SAFE_READ(sticky_fault_oversupplyv, talonfxpro_->GetStickyFault_OverSupplyV());
        SAFE_READ(sticky_fault_unstablesupplyv, talonfxpro_->GetStickyFault_UnstableSupplyV());
        SAFE_READ(sticky_fault_reversehardlimit, talonfxpro_->GetStickyFault_ReverseHardLimit());
        SAFE_READ(sticky_fault_forwardhardlimit, talonfxpro_->GetStickyFault_ForwardHardLimit());
        SAFE_READ(sticky_fault_reversesoftlimit, talonfxpro_->GetStickyFault_ReverseSoftLimit());
        SAFE_READ(sticky_fault_forwardsoftlimit, talonfxpro_->GetStickyFault_ForwardSoftLimit());
        SAFE_READ(sticky_fault_missingremotesensor, talonfxpro_->GetStickyFault_MissingRemoteSensor());
        SAFE_READ(sticky_fault_fusedsensoroutofsync, talonfxpro_->GetStickyFault_FusedSensorOutOfSync());
        SAFE_READ(sticky_fault_statorcurrlimit, talonfxpro_->GetStickyFault_StatorCurrLimit());
        SAFE_READ(sticky_fault_supplycurrlimit, talonfxpro_->GetStickyFault_SupplyCurrLimit());

        double closed_loop_proportional_output{0};
        double closed_loop_integrated_output{0};
        double closed_loop_feed_forward{0};
        double closed_loop_derivative_output{0};
        double closed_loop_output{0};
        units::radian_t closed_loop_reference{0};
        units::radians_per_second_t closed_loop_reference_slope{0};
        double closed_loop_error{0};

        SAFE_READ(control_mode, talonfxpro_->GetControlMode());

        if ((*control_mode == ctre::phoenix6::signals::ControlModeValue::PositionDutyCycle) ||
            (*control_mode == ctre::phoenix6::signals::ControlModeValue::VelocityDutyCycle) ||
            (*control_mode == ctre::phoenix6::signals::ControlModeValue::MotionMagicDutyCycle) ||
            (*control_mode == ctre::phoenix6::signals::ControlModeValue::PositionDutyCycleFOC) ||
            (*control_mode == ctre::phoenix6::signals::ControlModeValue::VelocityDutyCycleFOC) ||
            (*control_mode == ctre::phoenix6::signals::ControlModeValue::MotionMagicDutyCycleFOC) ||
            (*control_mode == ctre::phoenix6::signals::ControlModeValue::PositionVoltage) ||
            (*control_mode == ctre::phoenix6::signals::ControlModeValue::VelocityVoltage) ||
            (*control_mode == ctre::phoenix6::signals::ControlModeValue::MotionMagicVoltage) ||
            (*control_mode == ctre::phoenix6::signals::ControlModeValue::PositionVoltageFOC) ||
            (*control_mode == ctre::phoenix6::signals::ControlModeValue::VelocityVoltageFOC) ||
            (*control_mode == ctre::phoenix6::signals::ControlModeValue::MotionMagicVoltageFOC) ||
            (*control_mode == ctre::phoenix6::signals::ControlModeValue::PositionTorqueCurrentFOC) ||
            (*control_mode == ctre::phoenix6::signals::ControlModeValue::VelocityTorqueCurrentFOC) ||
            (*control_mode == ctre::phoenix6::signals::ControlModeValue::MotionMagicTorqueCurrentFOC))
        {
            SAFE_READ_INTO(closed_loop_proportional_output, talonfxpro_->GetClosedLoopProportionalOutput());
            SAFE_READ_INTO(closed_loop_integrated_output, talonfxpro_->GetClosedLoopIntegratedOutput());
            SAFE_READ_INTO(closed_loop_feed_forward, talonfxpro_->GetClosedLoopFeedForward());
            SAFE_READ_INTO(closed_loop_derivative_output, talonfxpro_->GetClosedLoopDerivativeOutput());
            SAFE_READ_INTO(closed_loop_output, talonfxpro_->GetClosedLoopOutput());
            SAFE_READ(closed_loop_reference_local, talonfxpro_->GetClosedLoopReference());
            closed_loop_reference = units::turn_t{*closed_loop_reference_local};
            SAFE_READ(closed_loop_reference_slope_local, talonfxpro_->GetClosedLoopReferenceSlope());
            closed_loop_reference_slope = units::turns_per_second_t{*closed_loop_reference_slope_local};
            SAFE_READ_INTO(closed_loop_error, talonfxpro_->GetClosedLoopError());
        }

        // TODO : either read these only if diff mode is active,
        // or turn the calls below into SAFE_READ ones
        double differential_output{0};
        double differential_closed_loop_proportional_output{0};
        double differential_closed_loop_integrated_output{0};
        double differential_closed_loop_feed_forward{0};
        double differential_closed_loop_derivative_output{0};
        double differential_closed_loop_output{0};
        units::radian_t differential_closed_loop_reference{0};
        units::radians_per_second_t differential_closed_loop_reference_slope{0};
        double differential_closed_loop_error{0};

        if (in_differential_mode)
        {
            SAFE_READ_INTO(differential_output, talonfxpro_->GetDifferentialOutput());
            SAFE_READ_INTO(differential_closed_loop_proportional_output, talonfxpro_->GetDifferentialClosedLoopProportionalOutput());
            SAFE_READ_INTO(differential_closed_loop_integrated_output, talonfxpro_->GetDifferentialClosedLoopIntegratedOutput());
            SAFE_READ_INTO(differential_closed_loop_feed_forward, talonfxpro_->GetDifferentialClosedLoopFeedForward());
            SAFE_READ_INTO(differential_closed_loop_derivative_output, talonfxpro_->GetDifferentialClosedLoopDerivativeOutput());
            SAFE_READ_INTO(differential_closed_loop_output, talonfxpro_->GetDifferentialClosedLoopOutput());
            SAFE_READ(differential_closed_loop_reference_local, talonfxpro_->GetDifferentialClosedLoopReference());
            differential_closed_loop_reference = units::turn_t{*differential_closed_loop_reference_local};
            SAFE_READ(differential_closed_loop_reference_slope_local, talonfxpro_->GetDifferentialClosedLoopReferenceSlope());
            differential_closed_loop_reference_slope = units::turns_per_second_t{*differential_closed_loop_reference_slope_local};
            SAFE_READ_INTO(differential_closed_loop_error, talonfxpro_->GetDifferentialClosedLoopError());
        }

        // Actually update the TalonHWState shared between
		// this thread and read()
		// Do this all at once so the code minimizes the amount
		// of time with mutex locked
		{
			// Lock the state entry to make sure writes
			// are atomic - reads won't grab data in
			// the middle of a write
			std::lock_guard<std::mutex> l(*read_state_mutex_);

            read_thread_state_->setVersionMajor(*version_major);
            read_thread_state_->setVersionMinor(*version_minor);
            read_thread_state_->setVersionBugfix(*version_bugfix);
            read_thread_state_->setVersionBuild(*version_build);

            read_thread_state_->setForwardLimit(*forward_limit == ctre::phoenix6::signals::ForwardLimitValue::ClosedToGround);
            read_thread_state_->setReverseLimit(*reverse_limit == ctre::phoenix6::signals::ReverseLimitValue::ClosedToGround);

            read_thread_state_->setAppliedRotorPolarity(applied_rotor_polarity);

            read_thread_state_->setDutyCycle(*duty_cycle);
            read_thread_state_->setTorqueCurrent(torque_current->value());
            read_thread_state_->setStatorCurrent(stator_current->value());
            read_thread_state_->setSupplyCurrent(supply_current->value());
            read_thread_state_->setSupplyVoltage(supply_voltage->value());
            read_thread_state_->setDeviceTemp(device_temp->value());
            read_thread_state_->setProcessorTemp(processor_temp->value());

            read_thread_state_->setRotorVelocity(units::radians_per_second_t{*rotor_velocity}.value());
            read_thread_state_->setRotorPosition(units::radian_t{*rotor_position}.value());
            read_thread_state_->setVelocity(units::radians_per_second_t{*velocity}.value());
            read_thread_state_->setPosition(units::radian_t{*position}.value());

            read_thread_state_->setDeviceEnable(*device_enable == ctre::phoenix6::signals::DeviceEnableValue::Enabled);
            read_thread_state_->setMotionMagicIsRunning(*motion_magic_is_running == ctre::phoenix6::signals::MotionMagicIsRunningValue::Enabled);

            read_thread_state_->setDifferentialControlMode(differential_control_mode);
            read_thread_state_->setDifferentialAverageVelocity(differential_average_velocity.value());
            read_thread_state_->setDifferentialAveragePosition(differential_average_position.value());
            read_thread_state_->setDifferentialDifferenceVelocity(differential_difference_velocity.value());
            read_thread_state_->setDifferentialDifferencePosition(differential_difference_position.value());

            read_thread_state_->setBridgeOutput(bridge_output_value);

            read_thread_state_->setFaultHardware(*fault_hardware);
            read_thread_state_->setFaultProcTemp(*fault_proctemp);
            read_thread_state_->setFaultDeviceTemp(*fault_devicetemp);
            read_thread_state_->setFaultUndervoltage(*fault_undervoltage);
            read_thread_state_->setFaultBootDuringEnable(*fault_bootduringenable);
            read_thread_state_->setFaultUnlicensedFeatureInUse(*fault_unlicensed_feature_in_use);
            read_thread_state_->setFaultRemoteSensorReset(*fault_remotesensorreset);
            read_thread_state_->setFaultMissingDifferentialFX(*fault_missingdifferentialfx);
            read_thread_state_->setFaultRemoteSensorPosOverfow(*fault_remotesensorposoverflow);
            read_thread_state_->setFaultOverSupplyV(*fault_oversupplyv);
            read_thread_state_->setFaultUnstableSupplyV(*fault_unstablesupplyv);
            read_thread_state_->setFaultReverseHardLimit(*fault_reversehardlimit);
            read_thread_state_->setFaultForwardHardLimit(*fault_forwardhardlimit);
            read_thread_state_->setFaultReverseSoftLimit(*fault_reversesoftlimit);
            read_thread_state_->setFaultForwardSoftLimit(*fault_forwardsoftlimit);
            read_thread_state_->setFaultMissingRemoteSensor(*fault_missingremotesensor);
            read_thread_state_->setFaultFusedSensorOutOfSync(*fault_fusedsensoroutofsync);
            read_thread_state_->setFaultStatorCurrLimit(*fault_statorcurrlimit);
            read_thread_state_->setFaultSupplyCurrLimit(*fault_supplycurrlimit);

            read_thread_state_->setStickyFaultHardware(*sticky_fault_hardware);
            read_thread_state_->setStickyFaultProcTemp(*sticky_fault_proctemp);
            read_thread_state_->setStickyFaultDeviceTemp(*sticky_fault_devicetemp);
            read_thread_state_->setStickyFaultUndervoltage(*sticky_fault_undervoltage);
            read_thread_state_->setStickyFaultBootDuringEnable(*sticky_fault_bootduringenable);
            read_thread_state_->setStickyFaultUnlicensedFeatureInUse(*sticky_fault_unlicensed_feature_in_use);
            read_thread_state_->setStickyFaultRemoteSensorReset(*sticky_fault_remotesensorreset);
            read_thread_state_->setStickyFaultMissingDifferentialFX(*sticky_fault_missingdifferentialfx);
            read_thread_state_->setStickyFaultRemoteSensorPosOverfow(*sticky_fault_remotesensorposoverflow);
            read_thread_state_->setStickyFaultOverSupplyV(*sticky_fault_oversupplyv);
            read_thread_state_->setStickyFaultUnstableSupplyV(*sticky_fault_unstablesupplyv);
            read_thread_state_->setStickyFaultReverseHardLimit(*sticky_fault_reversehardlimit);
            read_thread_state_->setStickyFaultForwardHardLimit(*sticky_fault_forwardhardlimit);
            read_thread_state_->setStickyFaultReverseSoftLimit(*sticky_fault_reversesoftlimit);
            read_thread_state_->setStickyFaultForwardSoftLimit(*sticky_fault_forwardsoftlimit);
            read_thread_state_->setStickyFaultMissingRemoteSensor(*sticky_fault_missingremotesensor);
            read_thread_state_->setStickyFaultFusedSensorOutOfSync(*sticky_fault_fusedsensoroutofsync);
            read_thread_state_->setStickyFaultStatorCurrLimit(*sticky_fault_statorcurrlimit);
            read_thread_state_->setStickyFaultSupplyCurrLimit(*sticky_fault_supplycurrlimit);

            read_thread_state_->setClosedLoopProportionalOutput(closed_loop_proportional_output);
            read_thread_state_->setClosedLoopIntegratedOutput(closed_loop_integrated_output);
            read_thread_state_->setClosedLoopFeedForward(closed_loop_feed_forward);
            read_thread_state_->setClosedLoopDerivativeOutput(closed_loop_derivative_output);
            read_thread_state_->setClosedLoopOutput(closed_loop_output);
            read_thread_state_->setClosedLoopReference(closed_loop_reference.value());
            read_thread_state_->setClosedLoopReferenceSlope(closed_loop_reference_slope.value());
            read_thread_state_->setClosedLoopError(closed_loop_error);

            read_thread_state_->setDifferentialOutput(differential_output);
            read_thread_state_->setDifferentialClosedLoopProportionalOutput(differential_closed_loop_proportional_output);
            read_thread_state_->setDifferentialClosedLoopIntegratedOutput(differential_closed_loop_integrated_output);
            read_thread_state_->setDifferentialClosedLoopFeedForward(differential_closed_loop_feed_forward);
            read_thread_state_->setDifferentialClosedLoopDerivativeOutput(differential_closed_loop_derivative_output);
            read_thread_state_->setDifferentialClosedLoopOutput(differential_closed_loop_output);
            read_thread_state_->setDifferentialClosedLoopReference(differential_closed_loop_reference.value());
            read_thread_state_->setDifferentialClosedLoopReferenceSlope(differential_closed_loop_reference_slope.value());
            read_thread_state_->setDifferentialClosedLoopError(differential_closed_loop_error);
        }

        tracer->report(60);
    }
}

void TalonFXProDevice::simRead(const ros::Time &/*time*/, const ros::Duration &period)
{
    auto &sim_state = talonfxpro_->GetSimState();

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
        units::radian_t setpoint{state_->getControlOutput()};
        sim_state.SetRawRotorPosition(setpoint);
        sim_state.SetRotorVelocity(units::angular_velocity::turns_per_second_t{0});
        sim_state.SetSupplyVoltage(units::voltage::volt_t{12.5});
        break;
    }
    case hardware_interface::talonfxpro::TalonMode::VelocityDutyCycle:
    case hardware_interface::talonfxpro::TalonMode::VelocityVoltage:
    case hardware_interface::talonfxpro::TalonMode::VelocityTorqueCurrentFOC:
    {
        units::angular_velocity::radians_per_second_t setpoint{state_->getControlOutput()};
        sim_state.SetRotorVelocity(setpoint);
        units::radian_t delta_position{state_->getVelocity() * period.toSec()};
        sim_state.AddRotorPosition(delta_position);
        sim_state.SetSupplyVoltage(units::voltage::volt_t{12.5});
        break;
    }
    case hardware_interface::talonfxpro::TalonMode::MotionMagicDutyCycle:
    case hardware_interface::talonfxpro::TalonMode::MotionMagicVoltage:
    case hardware_interface::talonfxpro::TalonMode::MotionMagicTorqueCurrentFOC:
    {
        // TODO : debug, check sim Orientation field
        const double invert = state_->getInvert() == hardware_interface::talonfxpro::Inverted::Clockwise_Positive ? -1.0 : 1.0;
        units::radian_t target_position{invert * state_->getClosedLoopReference()};
        units::angular_velocity::radians_per_second_t target_velocity{invert * state_->getClosedLoopReferenceSlope()};
        sim_state.SetRawRotorPosition(target_position);
        sim_state.SetRotorVelocity(target_velocity);
        sim_state.SetSupplyVoltage(units::voltage::volt_t{12.5});
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
    }
}

int TalonFXProDevice::getCANID(void) const
{
    return getId();
}

bool TalonFXProDevice::setSimLimitSwitches(const bool forward_limit, const bool reverse_limit)
{
    auto &sim_state = talonfxpro_->GetSimState();
    sim_state.SetForwardLimit(forward_limit);
    sim_state.SetReverseLimit(reverse_limit);
    return true;
}

bool TalonFXProDevice::setSimCurrent(const double /*stator_current*/, const double /*supply_current*/)
{
    ROS_ERROR_STREAM("Error settings sim current on TalonFXPro device " << getName() << " : Not supported");
    return false;
}

static bool convertGravityType(const hardware_interface::talonfxpro::GravityType in,
                            ctre::phoenix6::signals::GravityTypeValue &out)
{
    if (in == hardware_interface::talonfxpro::GravityType::Elevator_Static)
    {
        out = ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
        return true;
    }
    if (in == hardware_interface::talonfxpro::GravityType::Arm_Cosine)
    {
        out = ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine;
        return true;
    }
    ROS_ERROR_STREAM("Invalid GravityType value in TalonFXPro convertGravityType : in = " << static_cast<int>(in));
    return false;
}

static bool convertInverted(const hardware_interface::talonfxpro::Inverted in,
                            ctre::phoenix6::signals::InvertedValue &out)
{
    if (in == hardware_interface::talonfxpro::Inverted::CounterClockwise_Positive)
    {
        out = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
        return true;
    }
    if (in == hardware_interface::talonfxpro::Inverted::Clockwise_Positive)
    {
        out = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
        return true;
    }
    ROS_ERROR_STREAM("Invalid Inverted value in TalonFXPro convertInverted : in = " << static_cast<int>(in));
    return false;
}

static bool convertNeutralMode(const hardware_interface::talonfxpro::NeutralMode in,
                               ctre::phoenix6::signals::NeutralModeValue &out)
{

    if (in == hardware_interface::talonfxpro::NeutralMode::Brake)
    {
        out = ctre::phoenix6::signals::NeutralModeValue::Brake;
        return true;
    }
    if (in == hardware_interface::talonfxpro::NeutralMode::Coast)
    {
        out = ctre::phoenix6::signals::NeutralModeValue::Coast;
        return true;
    }
    ROS_ERROR_STREAM("Invalid Neutral Mode value in TalonFXPro convertNeutralMode : in = " << static_cast<int>(in));
    return false;
}

static bool convertFeedbackSensorSource(const hardware_interface::talonfxpro::FeedbackSensorSource in,
                                        ctre::phoenix6::signals::FeedbackSensorSourceValue &out)
{
    if (in == hardware_interface::talonfxpro::FeedbackSensorSource::RotorSensor)
    {
        out = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
        return true;
    }
    if (in == hardware_interface::talonfxpro::FeedbackSensorSource::RemoteCANCoder)
    {
        out = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
        return true;
    }
    if (in == hardware_interface::talonfxpro::FeedbackSensorSource::FusedCANcoder)
    {
        out = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
        return true;
    }
    ROS_ERROR_STREAM("Invalid Feedback Sensor Soruce value in TalonFXPro convertFeedbackSensorSource : in = " << static_cast<int>(in));
    return false;
}
static bool convertDifferentialSensorSource(const hardware_interface::talonfxpro::DifferentialSensorSource in,
                                            ctre::phoenix6::signals::DifferentialSensorSourceValue &out)
{
    switch(in)
    {
    case hardware_interface::talonfxpro::DifferentialSensorSource::Disabled:
        out = ctre::phoenix6::signals::DifferentialSensorSourceValue::Disabled;
        break;

    case hardware_interface::talonfxpro::DifferentialSensorSource::RemoteTalonFX_Diff:
        out = ctre::phoenix6::signals::DifferentialSensorSourceValue::RemoteTalonFX_Diff;
        break;
    case hardware_interface::talonfxpro::DifferentialSensorSource::RemotePigeon2_Yaw:
        out = ctre::phoenix6::signals::DifferentialSensorSourceValue::RemotePigeon2_Yaw;
        break;
    case hardware_interface::talonfxpro::DifferentialSensorSource::RemotePigeon2_Pitch:
        out = ctre::phoenix6::signals::DifferentialSensorSourceValue::RemotePigeon2_Pitch;
        break;
    case hardware_interface::talonfxpro::DifferentialSensorSource::RemotePigeon2_Roll:
        out = ctre::phoenix6::signals::DifferentialSensorSourceValue::RemotePigeon2_Roll;
        break;
    case hardware_interface::talonfxpro::DifferentialSensorSource::RemoteCANcoder:
        out = ctre::phoenix6::signals::DifferentialSensorSourceValue::RemoteCANcoder;
        break;
    default:
        ROS_ERROR_STREAM("Invalide Differential Sensor Source Value in TalonFXPro convertDifferentialSensorSource : in = " << static_cast<int>(in));
        return false;
    }

    return true;
}

static bool convertLimitType(const hardware_interface::talonfxpro::LimitType in,
                             ctre::phoenix6::signals::ForwardLimitTypeValue &out)
{
    if (in == hardware_interface::talonfxpro::LimitType::NormallyOpen)
    {
        out = ctre::phoenix6::signals::ForwardLimitTypeValue::NormallyOpen;
        return true;
    }
    if (in == hardware_interface::talonfxpro::LimitType::NormallyClosed)
    {
        out = ctre::phoenix6::signals::ForwardLimitTypeValue::NormallyClosed;
        return true;
    }
    ROS_ERROR_STREAM("Invalid Limit Type value in TalonFXPro convertLimitType : in = " << static_cast<int>(in));
    return false;
}

static bool convertLimitType(const hardware_interface::talonfxpro::LimitType in,
                             ctre::phoenix6::signals::ReverseLimitTypeValue &out)
{
    if (in == hardware_interface::talonfxpro::LimitType::NormallyOpen)
    {
        out = ctre::phoenix6::signals::ReverseLimitTypeValue::NormallyOpen;
        return true;
    }
    if (in == hardware_interface::talonfxpro::LimitType::NormallyClosed)
    {
        out = ctre::phoenix6::signals::ReverseLimitTypeValue::NormallyClosed;
        return true;
    }
    ROS_ERROR_STREAM("Invalid Limit Type value in TalonFXPro convertLimitType : in = " << static_cast<int>(in));
    return false;
}

static bool convertLimitSource(const hardware_interface::talonfxpro::LimitSource in,
                               ctre::phoenix6::signals::ForwardLimitSourceValue &out)
{
    if (in == hardware_interface::talonfxpro::LimitSource::LimitSwitchPin)
    {
        out = ctre::phoenix6::signals::ForwardLimitSourceValue::LimitSwitchPin;
        return true;
    }
    ROS_ERROR_STREAM("Invalid Limit Source value in TalonFXPro convertLimitSource : in = " << static_cast<int>(in));
    return false;
}

static bool convertLimitSource(const hardware_interface::talonfxpro::LimitSource in,
                               ctre::phoenix6::signals::ReverseLimitSourceValue &out)
{
    if (in == hardware_interface::talonfxpro::LimitSource::LimitSwitchPin)
    {
        out = ctre::phoenix6::signals::ReverseLimitSourceValue::LimitSwitchPin;
        return true;
    }
    ROS_ERROR_STREAM("Invalid Limit Source value in TalonFXPro convertLimitSource : in = " << static_cast<int>(in));
    return false;
}

void TalonFXProDevice::write(const ros::Time & /*time*/,
                             const ros::Duration & /*period*/,
                             const bool curr_robot_enabled,
                             bool prev_robot_enabled)
{
    // If a reset of the motor control has happened since the last time through,
    // force a rewrite of all of the configuration settings for this motor.
    if (talonfxpro_->HasResetOccurred())
    {
        command_->resetSlot(0);
        command_->resetSlot(1);
        command_->resetSlot(2);
        command_->resetMotorOutputConfig();
        command_->resetCurrentLimit();
        command_->resetVoltageConfigs();
        command_->resetTorqueCurrent();
        command_->resetFeedback();
        command_->resetDifferentialSensors();
        command_->resetDifferentialConstants();
        command_->resetOpenLoopRamps();
        command_->resetClosedLoopRamps();
        command_->resetLimit();
        command_->resetAudio();
        command_->resetSoftLimit();
        command_->resetMotionMagic();
        command_->resetContinuousWrap();
        command_->setClearStickyFaults();
        command_->resetSetPosition();
        command_->resetControl();
    }
    // For each config sub-section, check to see if any of the values have changed
    // since the previous control loop iteration. If any have, write the updated 
    // config values for that subsection to the motor controller.
    // The generic pattern for all the subsections
    // if a value has changed
    //    if writing to the motor controller is successful
    //        update state_ with the new, successfully written values
    //    else if writing was unsuccessful
    //        reset the changed flag for the config so it is retried the next time through
    //        return; (since the motor controller isn't in the expected state)
    ctre::phoenix6::configs::TalonFXConfiguration config;
    hardware_interface::talonfxpro::GravityType gravity_type;
    if (command_->slotChanged(config.Slot0.kP,
                              config.Slot0.kI,
                              config.Slot0.kD,
                              config.Slot0.kS,
                              config.Slot0.kV,
                              config.Slot0.kA,
                              config.Slot0.kG,
                              gravity_type,
                              0) &&
        convertGravityType(gravity_type, config.Slot0.GravityType))
    {
        if (safeCall(talonfxpro_->GetConfigurator().Apply(config.Slot0), "GetConfigurator().Apply(config.Slot0)"))
        {
            ROS_INFO_STREAM("Updated TalonFXPro id " <<  getId() << " = " << getName() << " Slot0 " << config.Slot0);
            state_->setkP(config.Slot0.kP, 0);
            state_->setkI(config.Slot0.kI, 0);
            state_->setkD(config.Slot0.kD, 0);
            state_->setkS(config.Slot0.kS, 0);
            state_->setkV(config.Slot0.kV, 0);
            state_->setkA(config.Slot0.kA, 0);
            state_->setkG(config.Slot0.kG, 0);
            state_->setGravityType(gravity_type, 0);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update TalonFXPro id " <<  getId() << " = " << getName() << " Slot0 " << config.Slot0);
            command_->resetSlot(0);
            return;
        }
    }
    if (command_->slotChanged(config.Slot1.kP,
                              config.Slot1.kI,
                              config.Slot1.kD,
                              config.Slot1.kS,
                              config.Slot1.kV,
                              config.Slot1.kA,
                              config.Slot1.kG,
                              gravity_type,
                              1) &&
        convertGravityType(gravity_type, config.Slot1.GravityType))
    {
        if (safeCall(talonfxpro_->GetConfigurator().Apply(config.Slot1), "GetConfigurator().Apply(config.Slot1)"))
        {
            ROS_INFO_STREAM("Updated TalonFXPro id " <<  getId() << " = " << getName() << " Slot1 " << config.Slot1);
            state_->setkP(config.Slot1.kP, 1);
            state_->setkI(config.Slot1.kI, 1);
            state_->setkD(config.Slot1.kD, 1);
            state_->setkS(config.Slot1.kS, 1);
            state_->setkV(config.Slot1.kV, 1);
            state_->setkV(config.Slot1.kA, 1);
            state_->setkV(config.Slot1.kG, 1);
            state_->setGravityType(gravity_type, 1);

        }
        else
        {
            ROS_INFO_STREAM("Failed to update TalonFXPro id " <<  getId() << " = " << getName() << " Slot1 " << config.Slot1);
            command_->resetSlot(1);
            return;
        }
    }
    if (command_->slotChanged(config.Slot2.kP,
                              config.Slot2.kI,
                              config.Slot2.kD,
                              config.Slot2.kS,
                              config.Slot2.kV,
                              config.Slot2.kA,
                              config.Slot2.kG,
                              gravity_type,
                              2) &&
        convertGravityType(gravity_type, config.Slot2.GravityType))
    {
        if (safeCall(talonfxpro_->GetConfigurator().Apply(config.Slot2), "GetConfigurator().Apply(config.Slot2)"))
        {
            ROS_INFO_STREAM("Updated TalonFXPro id " <<  getId() << " = " << getName() << " Slot2 " << config.Slot2);
            state_->setkP(config.Slot2.kP, 2);
            state_->setkI(config.Slot2.kI, 2);
            state_->setkD(config.Slot2.kD, 2);
            state_->setkS(config.Slot2.kS, 2);
            state_->setkV(config.Slot2.kV, 2);
            state_->setkV(config.Slot2.kA, 2);
            state_->setkV(config.Slot2.kG, 2);
            state_->setGravityType(gravity_type, 2);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update TalonFXPro id " <<  getId() << " = " << getName() << " Slot2 " << config.Slot2);
            command_->resetSlot(2);
            return;
        }
    }

    hardware_interface::talonfxpro::Inverted invert;
    hardware_interface::talonfxpro::NeutralMode neutral_mode;
    if (command_->motorOutputConfigChanged(invert,
                                           neutral_mode,
                                           config.MotorOutput.DutyCycleNeutralDeadband,
                                           config.MotorOutput.PeakForwardDutyCycle,
                                           config.MotorOutput.PeakReverseDutyCycle) &&
        convertInverted(invert, config.MotorOutput.Inverted) &&
        convertNeutralMode(neutral_mode, config.MotorOutput.NeutralMode))
    {
        if (safeCall(talonfxpro_->GetConfigurator().Apply(config.MotorOutput), "GetConfigurator().Apply(config.MotorOutput)"))
        {
            ROS_INFO_STREAM("Updated TalonFXPro id " <<  getId() << " = " << getName() << " MotorOutput " << config.MotorOutput);
            state_->setInvert(invert);
            state_->setNeutralMode(neutral_mode);
            state_->setDutyCycleNeutralDeadband(config.MotorOutput.DutyCycleNeutralDeadband);
            state_->setPeakForwardDutyCycle(config.MotorOutput.PeakForwardDutyCycle);
            state_->setPeakReverseDutyCycle(config.MotorOutput.PeakReverseDutyCycle);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update TalonFXPro id " <<  getId() << " = " << getName() << " MotorOutput " << config.MotorOutput);
            command_->resetMotorOutputConfig();
            return;
        }
    }

    if (command_->currentLimitChanged(config.CurrentLimits.StatorCurrentLimit,
                                      config.CurrentLimits.StatorCurrentLimitEnable,
                                      config.CurrentLimits.SupplyCurrentLimit,
                                      config.CurrentLimits.SupplyCurrentLimitEnable))
    {
        if (safeCall(talonfxpro_->GetConfigurator().Apply(config.CurrentLimits), "GetConfigurator().Apply(config.CurrentLimits)"))
        {
            ROS_INFO_STREAM("Updated TalonFXPro id " <<  getId() << " = " << getName() << " CurrentLimits " << config.CurrentLimits);
            state_->setStatorCurrentLimit(config.CurrentLimits.StatorCurrentLimit);
            state_->setStatorCurrentLimitEnable(config.CurrentLimits.StatorCurrentLimitEnable);
            state_->setSupplyCurrentLimit(config.CurrentLimits.SupplyCurrentLimit);
            state_->setSupplyCurrentLimitEnable(config.CurrentLimits.SupplyCurrentLimitEnable);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update TalonFXPro id " <<  getId() << " = " << getName() << " CurrentLimits " << config.CurrentLimits);
            command_->resetCurrentLimit();
            return;
        }
    }

    if (command_->voltageConfigsChanged(config.Voltage.SupplyVoltageTimeConstant,
                                        config.Voltage.PeakForwardVoltage,
                                        config.Voltage.PeakReverseVoltage))
    {
        if (safeCall(talonfxpro_->GetConfigurator().Apply(config.Voltage), "GetConfigurator().Apply(config.Voltage)"))
        {
            ROS_INFO_STREAM("Updated TalonFXPro id " <<  getId() << " = " << getName() << " Voltage " << config.Voltage);
            state_->setSupplyVoltageTimeConstant(config.Voltage.SupplyVoltageTimeConstant);
            state_->setPeakForwardVoltage(config.Voltage.PeakForwardVoltage);
            state_->setPeakReverseVoltage(config.Voltage.PeakReverseVoltage);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update TalonFXPro id " <<  getId() << " = " << getName() << " Voltage " << config.Voltage);
            command_->resetVoltageConfigs();
            return;
        }
    }
    if (command_->torqueCurrentChanged(config.TorqueCurrent.PeakForwardTorqueCurrent,
                                       config.TorqueCurrent.PeakReverseTorqueCurrent,
                                       config.TorqueCurrent.TorqueNeutralDeadband))
    {
        if (safeCall(talonfxpro_->GetConfigurator().Apply(config.TorqueCurrent), "GetConfigurator().Apply(config.TorqueCurrent)"))
        {
            ROS_INFO_STREAM("Updated TalonFXPro id " << getId() << " = " << getName() << " TorqueCurrent " << config.TorqueCurrent);
            state_->setPeakForwardTorqueCurrent(config.TorqueCurrent.PeakForwardTorqueCurrent);
            state_->setPeakReverseTorqueCurrent(config.TorqueCurrent.PeakReverseTorqueCurrent);
            state_->setTorqueNeutralDeadband(config.TorqueCurrent.TorqueNeutralDeadband);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update TalonFXPro id " << getId() << " = " << getName() << " TorqueCurrent " << config.TorqueCurrent);
            command_->resetTorqueCurrent();
            return;
        }
    }
    hardware_interface::talonfxpro::FeedbackSensorSource feedback_sensor_source;
    double feedback_rotor_offset_radians;
    if (command_->feebackChanged(feedback_rotor_offset_radians,
                                 config.Feedback.SensorToMechanismRatio,
                                 config.Feedback.RotorToSensorRatio,
                                 feedback_sensor_source,
                                 config.Feedback.FeedbackRemoteSensorID) &&
        convertFeedbackSensorSource(feedback_sensor_source, config.Feedback.FeedbackSensorSource))
    {
        config.Feedback.FeedbackRotorOffset = units::turn_t{units::radian_t{feedback_rotor_offset_radians}}.value();
        if (safeCall(talonfxpro_->GetConfigurator().Apply(config.Feedback), "GetConfigurator().Apply(config.Feedback)"))
        {
            ROS_INFO_STREAM("Updated TalonFXPro id " << getId() << " = " << getName() << " Feedback " << config.Feedback);
            state_->setFeedbackRotorOffset(config.Feedback.FeedbackRotorOffset);
            state_->setSensorToMechanismRatio(config.Feedback.SensorToMechanismRatio);
            state_->setRotorToSensorRatio(config.Feedback.RotorToSensorRatio);
            state_->setFeedbackSensorSource(feedback_sensor_source);
            state_->setFeedbackRemoteSensorID(config.Feedback.FeedbackRemoteSensorID);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update TalonFXPro id " << getId() << " = " << getName() << " Feedback " << config.Feedback);
            command_->resetFeedback();
            return;
        }
    }

    hardware_interface::talonfxpro::DifferentialSensorSource differential_sensor_source;
    if (command_->differentialSensorsChanged(differential_sensor_source,
                                 config.DifferentialSensors.DifferentialTalonFXSensorID,
                                 config.DifferentialSensors.DifferentialRemoteSensorID) &&
        convertDifferentialSensorSource(differential_sensor_source, config.DifferentialSensors.DifferentialSensorSource))
    {
        if (safeCall(talonfxpro_->GetConfigurator().Apply(config.DifferentialSensors), "GetConfigurator().Apply(config.DifferentialSensors)"))
        {
            ROS_INFO_STREAM("Updated TalonFXPro id " << getId() << " = " << getName() << " DifferentialSensors " << config.DifferentialSensors);
            state_->setDifferentialSensorSource(differential_sensor_source);
            state_->setDifferentialTalonFXSensorID(config.DifferentialSensors.DifferentialTalonFXSensorID);
            state_->setDifferentialRemoteSensorID(config.DifferentialSensors.DifferentialRemoteSensorID);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update TalonFXPro id " << getId() << " = " << getName() << " DifferentialSensors " << config.DifferentialSensors);
            command_->resetDifferentialSensors();
            return;
        }
    }

    if (command_->differentialConstantsChanged(config.DifferentialConstants.PeakDifferentialDutyCycle,
                                               config.DifferentialConstants.PeakDifferentialVoltage,
                                               config.DifferentialConstants.PeakDifferentialTorqueCurrent))
    {
        if (safeCall(talonfxpro_->GetConfigurator().Apply(config.DifferentialConstants), "GetConfigurator().Apply(config.DifferentialConstants)"))
        {
            ROS_INFO_STREAM("Updated TalonFXPro id " << getId() << " = " << getName() << " DifferentialConstants " << config.DifferentialConstants);
            state_->setPeakDifferentialDutyCycle(config.DifferentialConstants.PeakDifferentialDutyCycle);
            state_->setPeakDifferentialVoltage(config.DifferentialConstants.PeakDifferentialVoltage);
            state_->setPeakDifferentialTorqueCurrent(config.DifferentialConstants.PeakDifferentialTorqueCurrent);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update TalonFXPro id " << getId() << " = " << getName() << " DifferentialConstants " << config.DifferentialConstants);
            command_->resetDifferentialConstants();
            return;
        }
    }

    if (command_->openLoopRampsChanged(config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod,
                                       config.OpenLoopRamps.VoltageOpenLoopRampPeriod,
                                       config.OpenLoopRamps.TorqueOpenLoopRampPeriod))
    {
        if (safeCall(talonfxpro_->GetConfigurator().Apply(config.OpenLoopRamps), "GetConfigurator().Apply(config.OpenLoopRamps)"))
        {
            ROS_INFO_STREAM("Updated TalonFXPro id " << getId() << " = " << getName() << " OpenLoopRamps " << config.OpenLoopRamps);
            state_->setDutyCycleOpenLoopRampPeriod(config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod);
            state_->setVoltageOpenLoopRampPeriod(config.OpenLoopRamps.VoltageOpenLoopRampPeriod);
            state_->setTorqueOpenLoopRampPeriod(config.OpenLoopRamps.TorqueOpenLoopRampPeriod);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update TalonFXPro id " << getId() << " = " << getName() << " OpenLoopRamps " << config.OpenLoopRamps);
            command_->resetOpenLoopRamps();
            return;
        }
    }

    if (command_->closedLoopRampsChanged(config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod,
                                         config.ClosedLoopRamps.VoltageClosedLoopRampPeriod,
                                         config.ClosedLoopRamps.TorqueClosedLoopRampPeriod))
    {
        if (safeCall(talonfxpro_->GetConfigurator().Apply(config.ClosedLoopRamps), "GetConfigurator().Apply(config.ClosedLoopRamps)"))
        {
            ROS_INFO_STREAM("Updated TalonFXPro id " << getId() << " = " << getName() << " ClosedLoopRamps " << config.ClosedLoopRamps);
            state_->setDutyCycleClosedLoopRampPeriod(config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod);
            state_->setVoltageClosedLoopRampPeriod(config.ClosedLoopRamps.VoltageClosedLoopRampPeriod);
            state_->setTorqueClosedLoopRampPeriod(config.ClosedLoopRamps.TorqueClosedLoopRampPeriod);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update TalonFXPro id " << getId() << " = " << getName() << " ClosedLoopRamps " << config.ClosedLoopRamps);
            command_->resetClosedLoopRamps();
            return;
        }
    }
    hardware_interface::talonfxpro::LimitType forward_limit_type;
    hardware_interface::talonfxpro::LimitType reverse_limit_type;
    hardware_interface::talonfxpro::LimitSource forward_limit_source;
    hardware_interface::talonfxpro::LimitSource reverse_limit_source;
    if (command_->limitChanged(forward_limit_type,
                               config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable,
                               config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue,
                               config.HardwareLimitSwitch.ForwardLimitEnable,
                               forward_limit_source,
                               config.HardwareLimitSwitch.ForwardLimitRemoteSensorID,
                               reverse_limit_type,
                               config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable,
                               config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue,
                               config.HardwareLimitSwitch.ReverseLimitEnable,
                               reverse_limit_source,
                               config.HardwareLimitSwitch.ReverseLimitRemoteSensorID) &&
        convertLimitType(forward_limit_type, config.HardwareLimitSwitch.ForwardLimitType) &&
        convertLimitSource(forward_limit_source, config.HardwareLimitSwitch.ForwardLimitSource) &&
        convertLimitType(reverse_limit_type, config.HardwareLimitSwitch.ReverseLimitType) &&
        convertLimitSource(reverse_limit_source, config.HardwareLimitSwitch.ReverseLimitSource))
    {
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue *= state_->getSensorToMechanismRatio() / (2.0 * M_PI);
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue *= state_->getSensorToMechanismRatio() / (2.0 * M_PI);
        if (safeCall(talonfxpro_->GetConfigurator().Apply(config.HardwareLimitSwitch), "GetConfigurator().Apply(config.HardwareLimitSwitch)"))
        {
            ROS_INFO_STREAM("Updated TalonFXPro id " << getId() << " = " << getName() << " HardwareLimitSwitch " << config.HardwareLimitSwitch);
            state_->setForwardLimitType(forward_limit_type);
            state_->setForwardLimitAutosetPositionEnable(config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable);
            state_->setForwardLimitAutosetPositionValue(config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue);
            state_->setForwardLimitEnable(config.HardwareLimitSwitch.ForwardLimitEnable);
            state_->setForwardLimitSource(forward_limit_source);
            state_->setForwardLimitRemoteSensorID(config.HardwareLimitSwitch.ForwardLimitRemoteSensorID);
            state_->setReverseLimitType(reverse_limit_type);
            state_->setReverseLimitAutosetPositionEnable(config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable);
            state_->setReverseLimitAutosetPositionValue(config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue);
            state_->setReverseLimitEnable(config.HardwareLimitSwitch.ReverseLimitEnable);
            state_->setReverseLimitSource(reverse_limit_source);
            state_->setReverseLimitRemoteSensorID(config.HardwareLimitSwitch.ReverseLimitRemoteSensorID);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update TalonFXPro id " << getId() << " = " << getName() << " HardwareLimitSwitch " << config.HardwareLimitSwitch);
            command_->resetLimit();
            return;
        }
    }

    if (command_->audioChanged(config.Audio.BeepOnBoot, config.Audio.BeepOnConfig, config.Audio.AllowMusicDurDisable))
    {
        if (safeCall(talonfxpro_->GetConfigurator().Apply(config.Audio), "GetConfigurator().Apply(config.Audio)"))
        {
            ROS_INFO_STREAM("Updated TalonFXPro id " << getId() << " = " << getName() << " Audio " << config.Audio);
            state_->setBeepOnBoot(config.Audio.BeepOnBoot);
            state_->setBeepOnConfig(config.Audio.BeepOnConfig);
            state_->setAllowMusicDurDisable(config.Audio.AllowMusicDurDisable);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update TalonFXPro id " << getId() << " = " << getName() << " Audio " << config.Audio);
            command_->resetAudio();
            return;
        }
    }

    if (command_->softLimitChanged(config.SoftwareLimitSwitch.ForwardSoftLimitEnable,
                                   config.SoftwareLimitSwitch.ReverseSoftLimitEnable,
                                   config.SoftwareLimitSwitch.ForwardSoftLimitThreshold,
                                   config.SoftwareLimitSwitch.ReverseSoftLimitThreshold))
    {
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold *= state_->getSensorToMechanismRatio() / (2 * M_PI);
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold *= state_->getSensorToMechanismRatio() / (2 * M_PI);
        if (safeCall(talonfxpro_->GetConfigurator().Apply(config.SoftwareLimitSwitch), "GetConfigurator().Apply(config.SoftwareLimitSwitch)"))
        {
            ROS_INFO_STREAM("Updated TalonFXPro id " << getId() << " = " << getName() << " SoftwareLimitSwitch " << config.SoftwareLimitSwitch);
            state_->setForwardSoftLimitEnable(config.SoftwareLimitSwitch.ForwardSoftLimitEnable);
            state_->setForwardSoftLimitThreshold(config.SoftwareLimitSwitch.ForwardSoftLimitThreshold);
            state_->setReverseSoftLimitEnable(config.SoftwareLimitSwitch.ReverseSoftLimitEnable);
            state_->setReverseSoftLimitThreshold(config.SoftwareLimitSwitch.ReverseSoftLimitThreshold);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update TalonFXPro id " << getId() << " = " << getName() << " SoftwareLimitSwitch " << config.SoftwareLimitSwitch);
            command_->resetSoftLimit();
            return;
        }
    }
    double motion_magic_cruise_velocity;
    double motion_magic_acceleration;
    double motion_magic_jerk;
    if (command_->motionMagicChanged(motion_magic_cruise_velocity,
                                     motion_magic_acceleration,
                                     motion_magic_jerk))
    {
        config.MotionMagic.MotionMagicCruiseVelocity = motion_magic_cruise_velocity * state_->getSensorToMechanismRatio() / (2.0 * M_PI);
        config.MotionMagic.MotionMagicAcceleration = motion_magic_acceleration * state_->getSensorToMechanismRatio() / (2.0 * M_PI);
        config.MotionMagic.MotionMagicJerk = motion_magic_jerk * state_->getSensorToMechanismRatio() /  (2.0 * M_PI);
        if (safeCall(talonfxpro_->GetConfigurator().Apply(config.MotionMagic), "GetConfigurator().Apply(config.MotionMagic)"))
        {
            ROS_INFO_STREAM("Updated TalonFXPro id " << getId() << " = " << getName() << " MotionMagic " << config.MotionMagic);
            state_->setMotionMagicCruiseVelocity(motion_magic_cruise_velocity);
            state_->setMotionMagicAcceleration(motion_magic_acceleration);
            state_->setMotionMagicJerk(motion_magic_jerk);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update TalonFXPro id " << getId() << " = " << getName() << " MotionMagic " << config.MotionMagic);
            command_->resetMotionMagic();
            return;
        }
    }
    if (command_->continuousWrapChanged(config.ClosedLoopGeneral.ContinuousWrap))
    {
        if (safeCall(talonfxpro_->GetConfigurator().Apply(config.ClosedLoopGeneral), "GetConfigurator().Apply(config.ClosedLoopGeneral)"))
        {
            ROS_INFO_STREAM("Updated TalonFXPro id " << getId() << " = " << getName() << " ClosedLoopGeneral " << config.ClosedLoopGeneral);
            state_->setContinuousWrap(config.ClosedLoopGeneral.ContinuousWrap);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update TalonFXPro id " << getId() << " = " << getName() << " ClosedLoopGeneral " << config.ClosedLoopGeneral);
            command_->resetContinuousWrap();
            return;
        }
    }
    if (command_->clearStickyFaultsChanged())
    {
        if (safeCall(talonfxpro_->ClearStickyFaults(), "talonfxpro_->ClearStickyFaults()"))
        {
            ROS_INFO_STREAM("ClearStickyFaults for TalonFXPro id " << getId() << " = " << getName() << " successful");
        }
        else
        {
            ROS_INFO_STREAM("ClearStickyFaults for TalonFXPro id " << getId() << " = " << getName() << " failed");
            command_->setClearStickyFaults();
            return;
        }
    }

    double set_rotor_position;
    if (command_->setPositionChanged(set_rotor_position))
    {
        if (safeCall(talonfxpro_->SetPosition(units::radian_t{set_rotor_position}), "talonfxpro_->SetPosition"))
        {
            //ROS_INFO_STREAM("SetPosition for TalonFXPro id " << getId() << " = " << getName() << " to " << set_rotor_position << " radians successful");
        }
        else
        {
            ROS_INFO_STREAM("SetRotorPosition for TalonFXPro id " << getId() << " = " << getName() << " failed");
            command_->resetSetPosition();
            return;
        }
    }

    // Typically only a subset of these are valid for any particular
    // mode, but get them all and figure it out based on control_mode below
    hardware_interface::talonfxpro::TalonMode control_mode;
    double control_output;
    double control_position;
    double control_velocity;
    double control_acceleration;
    bool control_enable_foc;
    bool control_override_brake_dur_neutral;
    double control_max_abs_duty_cycle;
    double control_deadband;
    double control_feedforward;
    int control_slot;
    double control_differential_position;
    int control_differential_slot;
    bool control_oppose_master_direction;

    // This happens in a few places, so define it once so additions aren't missed
    // by one of the uses below
    auto updateControlStatus = [&]()
    {
        state_->setControlMode(control_mode);
        state_->setControlOutput(control_output);
        state_->setControlPosition(control_position);
        state_->setControlVelocity(control_velocity);
        state_->setControlAcceleration(control_acceleration);
        state_->setControlEnableFOC(control_enable_foc);
        state_->setControlOverrideBrakeDurNeutral(control_override_brake_dur_neutral);
        state_->setControlMaxAbsDutyCycle(control_max_abs_duty_cycle);
        state_->setControlDeadband(control_deadband);
        state_->setControlFeedforward(control_feedforward);
        state_->setControlSlot(control_slot);
        state_->setControlOpposeMasterDirection(control_oppose_master_direction);
        state_->setControlDifferentialPosition(control_differential_position);
        state_->setControlDifferentialSlot(control_differential_slot);
    };

    const bool control_changed = command_->controlChanged(control_mode,
                                                          control_output,
                                                          control_position,
                                                          control_velocity,
                                                          control_acceleration,
                                                          control_enable_foc,
                                                          control_override_brake_dur_neutral,
                                                          control_max_abs_duty_cycle,
                                                          control_deadband,
                                                          control_feedforward,
                                                          control_slot,
                                                          control_differential_position,
                                                          control_differential_slot,
                                                          control_oppose_master_direction);
    if (curr_robot_enabled)
    {
        if (control_changed)
        {
            bool success = true; // only update state_ on a successful SetControl call
            switch (control_mode)
            {
            case hardware_interface::talonfxpro::TalonMode::DutyCycleOut:
                if (!safeCall(talonfxpro_->SetControl(
                                ctre::phoenix6::controls::DutyCycleOut(control_output,
                                                                        control_enable_foc,
                                                                        control_override_brake_dur_neutral)),
                            "setControl(DutyCycleOut)"))
                {
                    command_->resetControl();
                    success = false;
                }
                break;
            case hardware_interface::talonfxpro::TalonMode::TorqueCurrentFOC:
                if (!safeCall(talonfxpro_->SetControl(
                                ctre::phoenix6::controls::TorqueCurrentFOC(static_cast<units::current::ampere_t>(control_output),
                                                                            control_max_abs_duty_cycle,
                                                                            static_cast<units::current::ampere_t>(control_deadband),
                                                                            control_override_brake_dur_neutral)),
                            "setControl(TorqueCurrentFOC)"))
                {
                    command_->resetControl();
                    success = false;
                }
                break;
            case hardware_interface::talonfxpro::TalonMode::VoltageOut:
                if (!safeCall(talonfxpro_->SetControl(
                                ctre::phoenix6::controls::VoltageOut(static_cast<units::voltage::volt_t>(control_output),
                                                                    control_enable_foc,
                                                                    control_override_brake_dur_neutral)),
                            "setControl(VoltageOut)"))
                {
                    command_->resetControl();
                    success = false;
                }
                break;
            case hardware_interface::talonfxpro::TalonMode::PositionDutyCycle:
                if (!safeCall(talonfxpro_->SetControl(
                                ctre::phoenix6::controls::PositionDutyCycle(units::radian_t{control_position},
                                                                            units::radians_per_second_t{control_velocity},
                                                                            control_enable_foc,
                                                                            control_feedforward,
                                                                            control_slot,
                                                                            control_override_brake_dur_neutral)),
                            "setControl(PositionDutyCycle)"))
                {
                    command_->resetControl();
                    success = false;
                }
                break;
            case hardware_interface::talonfxpro::TalonMode::PositionVoltage:
                if (!safeCall(talonfxpro_->SetControl(
                                ctre::phoenix6::controls::PositionVoltage(units::radian_t{control_position},
                                                                            units::radians_per_second_t{control_velocity},
                                                                            control_enable_foc,
                                                                            units::voltage::volt_t{control_feedforward},
                                                                            control_slot,
                                                                            control_override_brake_dur_neutral)),
                            "setControl(PositionVoltage)"))
                {
                    command_->resetControl();
                    success = false;
                }
                break;
            case hardware_interface::talonfxpro::TalonMode::PositionTorqueCurrentFOC:
                if (!safeCall(talonfxpro_->SetControl(
                                ctre::phoenix6::controls::PositionTorqueCurrentFOC(units::radian_t{control_position},
                                                                                    units::radians_per_second_t{control_velocity},
                                                                                    units::current::ampere_t{control_feedforward},
                                                                                    control_slot,
                                                                                    control_override_brake_dur_neutral)),
                            "setControl(PositionTorqueCurrentFOC)"))
                {
                    command_->resetControl();
                    success = false;
                }
                break;
            case hardware_interface::talonfxpro::TalonMode::VelocityDutyCycle:
                if (!safeCall(talonfxpro_->SetControl(
                                ctre::phoenix6::controls::VelocityDutyCycle(units::radians_per_second_t{control_velocity},
                                                                            units::radians_per_second_squared_t{control_acceleration},
                                                                            control_enable_foc,
                                                                            control_feedforward,
                                                                            control_slot,
                                                                            control_override_brake_dur_neutral)),
                            "setControl(VelocityDutyCycle)"))
                {
                    command_->resetControl();
                    success = false;
                }
                break;
            case hardware_interface::talonfxpro::TalonMode::VelocityVoltage:
                if (!safeCall(talonfxpro_->SetControl(
                                ctre::phoenix6::controls::VelocityVoltage(units::radians_per_second_t{control_velocity},
                                                                            units::radians_per_second_squared_t{control_acceleration},
                                                                            control_enable_foc,
                                                                            units::voltage::volt_t{control_feedforward},
                                                                            control_slot,
                                                                            control_override_brake_dur_neutral)),
                            "setControl(VelocityVoltage)"))
                {
                    command_->resetControl();
                    success = false;
                }
                break;
            case hardware_interface::talonfxpro::TalonMode::VelocityTorqueCurrentFOC:
                if (!safeCall(talonfxpro_->SetControl(
                                ctre::phoenix6::controls::VelocityTorqueCurrentFOC(units::radians_per_second_t{control_velocity},
                                                                                    units::radians_per_second_squared_t{control_acceleration},
                                                                                    units::current::ampere_t{control_feedforward},
                                                                                    control_slot,
                                                                                    control_override_brake_dur_neutral)),
                            "setControl(VelocityTorqueCurrentFOC)"))
                {
                    command_->resetControl();
                    success = false;
                }
                break;
            case hardware_interface::talonfxpro::TalonMode::MotionMagicDutyCycle:
                if (!safeCall(talonfxpro_->SetControl(
                                ctre::phoenix6::controls::MotionMagicDutyCycle(units::radian_t{control_position},
                                                                                control_enable_foc,
                                                                                control_feedforward,
                                                                                control_slot,
                                                                                control_override_brake_dur_neutral)),
                            "setControl(MotionMagicDutyCycle)"))
                {
                    command_->resetControl();
                    success = false;
                }
                break;
            case hardware_interface::talonfxpro::TalonMode::MotionMagicVoltage:
                if (!safeCall(talonfxpro_->SetControl(
                                ctre::phoenix6::controls::MotionMagicVoltage(units::radian_t{control_position},
                                                                            control_enable_foc,
                                                                            units::voltage::volt_t{control_feedforward},
                                                                            control_slot,
                                                                            control_override_brake_dur_neutral)),
                            "setControl(MotionMagicVoltage)"))
                {
                    command_->resetControl();
                    success = false;
                }
                break;
            case hardware_interface::talonfxpro::TalonMode::MotionMagicTorqueCurrentFOC:
                if (!safeCall(talonfxpro_->SetControl(
                                ctre::phoenix6::controls::MotionMagicTorqueCurrentFOC(units::radian_t{control_position},
                                                                                        units::ampere_t{control_feedforward},
                                                                                        control_slot,
                                                                                        control_override_brake_dur_neutral)),
                            "setControl(MotionMagicTorqueCurrentFOC)"))
                {
                    command_->resetControl();
                    success = false;
                }
                break;
            case hardware_interface::talonfxpro::TalonMode::Follower:
                if (!safeCall(talonfxpro_->SetControl(
                                ctre::phoenix6::controls::Follower(control_output,
                                                                    control_oppose_master_direction)),
                            "setControl(Follower)"))
                {
                    command_->resetControl();
                    success = false;
                }
                break;
            case hardware_interface::talonfxpro::TalonMode::StrictFollower:
                if (!safeCall(talonfxpro_->SetControl(
                                ctre::phoenix6::controls::StrictFollower(control_output)),
                            "setControl(StrictFollower)"))
                {
                    command_->resetControl();
                    success = false;
                }
                break;
            case hardware_interface::talonfxpro::TalonMode::NeutralOut:
                if (!safeCall(talonfxpro_->SetControl(
                                ctre::phoenix6::controls::NeutralOut()),
                            "setControl(NeutralOut)"))
                {
                    command_->resetControl();
                    success = false;
                }
                break;
            case hardware_interface::talonfxpro::TalonMode::CoastOut:
                if (!safeCall(talonfxpro_->SetControl(
                                ctre::phoenix6::controls::CoastOut()),
                            "setControl(CoastOut)"))
                {
                    command_->resetControl();
                    success = false;
                }
                break;
            case hardware_interface::talonfxpro::TalonMode::StaticBrake:
                if (!safeCall(talonfxpro_->SetControl(
                                ctre::phoenix6::controls::StaticBrake()),
                            "setControl(StaticBrake)"))
                {
                    command_->resetControl();
                    success = false;
                }
                break;
            case hardware_interface::talonfxpro::TalonMode::DifferentialDutyCycleOut:
                if (!safeCall(talonfxpro_->SetControl(
                                  ctre::phoenix6::controls::DifferentialDutyCycle(control_output,
                                                                                  units::radian_t{control_differential_position},
                                                                                  control_enable_foc,
                                                                                  control_differential_slot,
                                                                                  control_override_brake_dur_neutral)),
                              "setControl(DifferentialDutyCycle)"))
                {
                    command_->resetControl();
                    success = false;
                }
                break;
            case hardware_interface::talonfxpro::TalonMode::DifferentialVoltageOut:
                if (!safeCall(talonfxpro_->SetControl(
                                  ctre::phoenix6::controls::DifferentialVoltage(units::volt_t{control_output},
                                                                                units::radian_t{control_differential_position},
                                                                                control_enable_foc,
                                                                                control_differential_slot,
                                                                                control_override_brake_dur_neutral)),
                              "setControl(DifferentialVoltage)"))
                {
                    command_->resetControl();
                    success = false;
                }
                break;
            case hardware_interface::talonfxpro::TalonMode::DifferentialPositionDutyCycle:
                if (!safeCall(talonfxpro_->SetControl(
                                  ctre::phoenix6::controls::DifferentialPositionDutyCycle(units::radian_t{control_output},
                                                                                          units::radian_t{control_differential_position},
                                                                                          control_enable_foc,
                                                                                          control_slot,
                                                                                          control_differential_slot,
                                                                                          control_override_brake_dur_neutral)),
                              "setControl(DifferentialPositionDutyCycle)"))
                {
                    command_->resetControl();
                    success = false;
                }
                break;
            case hardware_interface::talonfxpro::TalonMode::DifferentialPositionVoltage:
                if (!safeCall(talonfxpro_->SetControl(
                                  ctre::phoenix6::controls::DifferentialPositionVoltage(units::radian_t{control_output},
                                                                                        units::radian_t{control_differential_position},
                                                                                        control_enable_foc,
                                                                                        control_slot,
                                                                                        control_differential_slot,
                                                                                        control_override_brake_dur_neutral)),
                              "setControl(DifferentialPositionVoltage)"))
                {
                    command_->resetControl();
                    success = false;
                }
                break;
            case hardware_interface::talonfxpro::TalonMode::DifferentialVelocityDutyCycle:
                if (!safeCall(talonfxpro_->SetControl(
                                  ctre::phoenix6::controls::DifferentialVelocityDutyCycle(units::radians_per_second_t{control_output},
                                                                                          units::radian_t{control_differential_position},
                                                                                          control_enable_foc,
                                                                                          control_slot,
                                                                                          control_differential_slot,
                                                                                          control_override_brake_dur_neutral)),
                              "setControl(DifferentialVelocityDutyCycle)"))
                {
                    command_->resetControl();
                    success = false;
                }
                break;
            case hardware_interface::talonfxpro::TalonMode::DifferentialVelocityVoltage:
                if (!safeCall(talonfxpro_->SetControl(
                                  ctre::phoenix6::controls::DifferentialVelocityVoltage(units::radians_per_second_t{control_output},
                                                                                        units::radian_t{control_differential_position},
                                                                                        control_enable_foc,
                                                                                        control_slot,
                                                                                        control_differential_slot,
                                                                                        control_override_brake_dur_neutral)),
                              "setControl(DifferentialVelocityVoltage)"))
                {
                    command_->resetControl();
                    success = false;
                }
                break;
            case hardware_interface::talonfxpro::TalonMode::DifferentialMotionMagicDutyCycle:
                if (!safeCall(talonfxpro_->SetControl(
                                  ctre::phoenix6::controls::DifferentialMotionMagicDutyCycle(units::radian_t{control_output},
                                                                                             units::radian_t{control_differential_position},
                                                                                             control_enable_foc,
                                                                                             control_slot,
                                                                                             control_differential_slot,
                                                                                             control_override_brake_dur_neutral)),
                              "setControl(DifferentialMotionMagicDutyCycle)"))
                {
                    command_->resetControl();
                    success = false;
                }
                break;
            case hardware_interface::talonfxpro::TalonMode::DifferentialMotionMagicVoltage:
                if (!safeCall(talonfxpro_->SetControl(
                                  ctre::phoenix6::controls::DifferentialMotionMagicVoltage(units::radian_t{control_output},
                                                                                           units::radian_t{control_differential_position},
                                                                                           control_enable_foc,
                                                                                           control_slot,
                                                                                           control_differential_slot,
                                                                                           control_override_brake_dur_neutral)),
                              "setControl(DifferentialMotionMagicDutyCycle)"))
                {
                    command_->resetControl();
                    success = false;
                }
                break;
            case hardware_interface::talonfxpro::TalonMode::DifferentialFollower:
                if (!safeCall(talonfxpro_->SetControl(
                                  ctre::phoenix6::controls::DifferentialFollower(control_output,
                                                                                 control_oppose_master_direction)),
                              "setControl(DifferentialFollower)"))
                {
                    command_->resetControl();
                    success = false;
                }
                break;
            case hardware_interface::talonfxpro::TalonMode::DifferentialStrictFollower:
                if (!safeCall(talonfxpro_->SetControl(
                                  ctre::phoenix6::controls::DifferentialStrictFollower(control_output)),
                              "setControl(DifferentialStrictFollower)"))
                {
                    command_->resetControl();
                    success = false;
                }
                break;
            default:
                ROS_ERROR_STREAM("Unknown control_mode " << static_cast<int>(control_mode) << " in talonfxpro write()");
                success = false;
                break;
            } // switch(control_mode)

            if (success)
            {
                updateControlStatus();
            }
            else
            {
                command_->resetControl();
            }
        }
    }
    else // robot not enabled
    {
        // Update talon state with requested control values for 
        // debugging. Don't actually write them to the physical
        // Talons until the robot is re-enabled, though.
        updateControlStatus();
        if (prev_robot_enabled)
        {
            // call resetMode() to queue up a change back to the correct mode / outputs / etc
            // when the robot switches from disabled back to enabled
            command_->resetControl();
            // Set the mode to Disabled to indicate the code knows the robot is disabled
            // This is a hack to avoid having everything also subscribe to match data to
            // figure out if the robot is enabled or not
            state_->setControlMode(hardware_interface::talonfxpro::TalonMode::Disabled);
            // Put the motors in neutral mode. This will respect the neutral mode brake or coast setting
            ROS_INFO_STREAM("Robot disabled - disabling motor " << getName() << " into neutral mode");
            safeCall(talonfxpro_->SetControl(ctre::phoenix6::controls::NeutralOut()), "setControl(NeutralOut for Disabled)");
        }
    }
}