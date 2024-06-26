#ifndef INC_TALON_STATE_INTERFACE__
#define INC_TALON_STATE_INTERFACE__

#include <array>

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <state_handle/state_handle.h>
#include "ctre_interfaces/absolute_sensor_range.h"
#include "ctre_interfaces/sensor_initialization_strategy.h"
#include "ctre_interfaces/talon_state_types.h"

namespace hardware_interface
{
// Class which contains state information
// about a given Talon SRX. This should include
// data about the mode the Talon is running in,
// current config and setpoint as well as data
// from the attached encoders, limit switches,
// etc.
// Each pass through read() in the low-level
// hardware interface should update the member
// vars of this class.
// The controllers can access the member variables
// as needed to make decisions in their update code
// The hardware_controller is responsible for keeping
// a master array of these classes - 1 entry per
// physical Talon controller in the robot
class TalonHWState
{
	public:
		explicit TalonHWState(int can_id);
		TalonHWState(const TalonHWState &) = default;
		TalonHWState(TalonHWState &&) = default;
		~TalonHWState();
		TalonHWState &operator=(const TalonHWState &) = default;
		TalonHWState &operator=(TalonHWState &&) = default;

		double getSetpoint(void) const;

		double getPosition(void) const;
		double getSpeed(void) const;
		double getOutputVoltage(void) const;
		int    getCANID(void) const;
		double getOutputCurrent(void) const;
		double getStatorCurrent(void) const;
		double getSupplyCurrent(void) const;
		double getBusVoltage(void) const;
		double getMotorOutputPercent(void) const;
		double getTemperature(void) const;
		double getPidfP(size_t index) const;
		double getPidfI(size_t index) const;
		double getPidfD(size_t index) const;
		double getPidfF(size_t index) const;
		double getPidfIzone(size_t index) const;
		double getAllowableClosedLoopError(size_t index) const;
		double getMaxIntegralAccumulator(size_t index) const;
		double getClosedLoopPeakOutput(size_t index) const;
		int getClosedLoopPeriod(size_t index) const;
		bool getAuxPidPolarity(void) const;
		double getClosedLoopError(void) const;
		double getIntegralAccumulator(void) const;
		double getErrorDerivative(void) const;
		double getClosedLoopTarget(void) const;
		double getPTerm(void) const;
		double getITerm(void) const;
		double getDTerm(void) const;
		double getFTerm(void) const;
		double getActiveTrajectoryPosition(void) const;
		double getActiveTrajectoryVelocity(void) const;
		double getActiveTrajectoryArbitraryFeedForward(void) const;
		double getActiveTrajectoryHeading(void) const;
		bool getForwardLimitSwitch(void) const;
		bool getReverseLimitSwitch(void) const;
		bool getForwardSoftlimitHit(void) const;
		bool getReverseSoftlimitHit(void) const;
		TalonMode getTalonMode(void) const;
		DemandType getDemand1Type(void) const;
		double getDemand1Value(void) const;
		int  getSlot(void) const;
		bool getInvert(void) const;
		bool getSensorPhase(void) const;
		NeutralMode getNeutralMode(void) const;
		FeedbackDevice getEncoderFeedback(void) const;
		RemoteFeedbackDevice getRemoteEncoderFeedback(void) const;
		double getFeedbackCoefficient(void) const;
		int getRemoteFeedbackDeviceId(size_t remote_ordinal) const;
		RemoteSensorSource getRemoteFeedbackFilter(size_t remote_ordinal) const;
		FeedbackDevice getSensorTerm(SensorTerm sensor_term) const;
		int getEncoderTicksPerRotation(void) const;
		unsigned int getFaults(void) const;
		unsigned int getStickyFaults(void) const;
		double getConversionFactor(void) const;
		bool getEnableReadThread(void) const;
		void setSetpoint(double setpoint);
		void setPosition(double position);
		void setSpeed(double speed);
		void setOutputVoltage(double output_voltage);
		void setOutputCurrent(double output_current);
		void setStatorCurrent(double stator_current);
		void setSupplyCurrent(double supply_current);
		void setBusVoltage(double bus_voltage);
		void setMotorOutputPercent(double motor_output_percent);
		void setTemperature(double temperature);
		void setClosedloopRamp(double close_loop_ramp);
		double getClosedloopRamp(void) const;
		void setOpenloopRamp(double open_loop_ramp);
		double getOpenloopRamp(void) const;
		void setPeakOutputForward(double peak_output_forward);
		double getPeakOutputForward(void) const;
		void setPeakOutputReverse(double peak_output_reverse);
		double getPeakOutputReverse(void) const;
		void setNominalOutputForward(double nominal_output_forward);
		double getNominalOutputForward(void) const;
		void setNominalOutputReverse(double nominal_output_reverse);
		double getNominalOutputReverse(void) const;
		void setNeutralDeadband(double neutral_deadband);
		double getNeutralDeadband(void) const;
		void setVoltageCompensationSaturation(double voltage_compensation_saturation);
		double getVoltageCompensationSaturation(void) const;
		void setVoltageMeasurementFilter(int voltage_measurement_filter);
		int getVoltageMeasurementFilter(void) const;
		void setVoltageCompensationEnable(bool voltage_compensation_enable);
		bool getVoltageCompensationEnable(void) const;
		void setVelocityMeasurementPeriod(hardware_interface::VelocityMeasurementPeriod period);
		bool getVelocityMeasurementPeriod(void) const;
		void setVelocityMeasurementWindow(int window);
		bool getVelocityMeasurementWindow(void) const;
		void setForwardLimitSwitchSource(LimitSwitchSource source, LimitSwitchNormal normal);
		void getForwardLimitSwitchSource(LimitSwitchSource &source, LimitSwitchNormal &normal) const;
		void setReverseLimitSwitchSource(LimitSwitchSource source, LimitSwitchNormal normal);
		void getReverseLimitSwitchSource(LimitSwitchSource &source, LimitSwitchNormal &normal) const;
		void setRemoteForwardLimitSwitchSource(RemoteLimitSwitchSource source, LimitSwitchNormal normal, unsigned int id);
		void getRemoteForwardLimitSwitchSource(RemoteLimitSwitchSource &source, LimitSwitchNormal &normal, unsigned int &id) const;
		void setRemoteReverseLimitSwitchSource(RemoteLimitSwitchSource source, LimitSwitchNormal normal, unsigned int id);
		void getRemoteReverseLimitSwitchSource(RemoteLimitSwitchSource &source, LimitSwitchNormal &normal, unsigned int &id) const;
		void setForwardSoftLimitThreshold(double threshold);
		double getForwardSoftLimitThreshold(void) const;
		void setForwardSoftLimitEnable(bool enable);
		bool getForwardSoftLimitEnable(void) const;
		void setReverseSoftLimitThreshold(double threshold);
		double getReverseSoftLimitThreshold(void) const;
		void setReverseSoftLimitEnable(bool enable);
		bool getReverseSoftLimitEnable(void) const;
		void setOverrideSoftLimitsEnable(bool enable);
		bool getOverrideSoftLimitsEnable(void) const;
		void setPeakCurrentLimit(int amps);
		int getPeakCurrentLimit(void) const;
		void setPeakCurrentDuration(int msec);
		int getPeakCurrentDuration(void) const;
		void setContinuousCurrentLimit(int amps);
		int getContinuousCurrentLimit(void) const;
		void setCurrentLimitEnable(bool enable);
		bool getCurrentLimitEnable(void) const;

		// Current Limits - Talon FX / Falcon 500
		void setSupplyCurrentLimit(double supply_current_limit);
		double getSupplyCurrentLimit(void) const;
		void setSupplyCurrentTriggerThresholdCurrent(double supply_current_trigger_threshold_current);
		double getSupplyCurrentTriggerThresholdCurrent(void) const;
		void setSupplyCurrentTriggerThresholdTime(double supply_current_trigger_threshold_time);
		double getSupplyCurrentTriggerThresholdTime(void) const;
		void setSupplyCurrentLimitEnable(bool supply_current_limit_enable);
		bool getSupplyCurrentLimitEnable(void) const;

		void setStatorCurrentLimit(double stator_current_limit);
		double getStatorCurrentLimit(void) const;
		void setStatorCurrentTriggerThresholdCurrent(double stator_current_trigger_threshold_current);
		double getStatorCurrentTriggerThresholdCurrent(void) const;
		void setStatorCurrentTriggerThresholdTime(double stator_current_trigger_threshold_time);
		double getStatorCurrentTriggerThresholdTime(void) const;
		void setStatorCurrentLimitEnable(bool stator_current_limit_enable);
		bool getStatorCurrentLimitEnable(void) const;

		void setMotionCruiseVelocity(double velocity);
		double getMotionCruiseVelocity(void) const;
		void setMotionAcceleration(double acceleration);
		double getMotionAcceleration(void) const;
		void setMotionSCurveStrength(int s_curve_strength);
		int getMotionSCurveStrength(void) const;
		void setMotionProfileTopLevelBufferCount(int count);
		int getMotionProfileTopLevelBufferCount(void) const;
		void setMotionProfileTopLevelBufferFull(bool is_full);
		bool getMotionProfileTopLevelBufferFull(void) const;
		void setMotionProfileStatus(const MotionProfileStatus &status);
		MotionProfileStatus getMotionProfileStatus(void) const;
		void setStatusFramePeriod(StatusFrame status_frame, uint8_t period);
		uint8_t getStatusFramePeriod(StatusFrame status_frame) const;
		void setControlFramePeriod(ControlFrame control_frame, uint8_t period);
		uint8_t getControlFramePeriod(ControlFrame control_frame) const;
		void setMotionProfileTrajectoryPeriod(int msec);
		int getMotionProfileTrajectoryPeriod(void) const;
		void setPidfP(double pidf_p, size_t index);
		void setPidfI(double pidf_i, size_t index);
		void setPidfD(double pidf_d, size_t index);
		void setPidfF(double pidf_f, size_t index);
		void setPidfIzone(double pidf_izone, size_t index);
		void setAllowableClosedLoopError(double allowable_closed_loop_error, size_t index);
		void setMaxIntegralAccumulator(double max_integral_accumulator, size_t index);
		void setClosedLoopPeakOutput(double closed_loop_peak_output, size_t index);
		void setClosedLoopPeriod(int closed_loop_period, size_t index);
		void setAuxPidPolarity(bool aux_pid_polarity);
		void setClosedLoopError(double closed_loop_error);
		void setIntegralAccumulator(double integral_accumulator);
		void setErrorDerivative(double error_derivative);
		void setClosedLoopTarget(double closed_loop_target);
		void setPTerm(double p_term);
		void setITerm(double i_term);
		void setDTerm(double d_term);
		void setFTerm(double f_term);
		void setActiveTrajectoryPosition(double active_trajectory_position);
		void setActiveTrajectoryVelocity(double active_trajectory_velocity);
		void setActiveTrajectoryArbitraryFeedForward(double active_trajectory_arbitrary_feed_forward);
		void setActiveTrajectoryHeading(double active_trajectory_heading);
		void setForwardLimitSwitch(bool forward_limit_switch_closed);
		void setReverseLimitSwitch(bool reverse_limit_switch_closed);
		void setForwardSoftlimitHit(bool forward_softlimit_hit);
		void setReverseSoftlimitHit(bool reverse_softlimit_hit);
		void setTalonMode(TalonMode talon_mode);
		void setDemand1Type(DemandType demand_type);
		void setDemand1Value(double value);
		void setSlot(int slot);
		void setInvert(bool invert);
		void setSensorPhase(bool sensor_phase);
		void setNeutralMode(NeutralMode neutral_mode);
		void setEncoderFeedback(FeedbackDevice encoder_feedback);
		void setRemoteEncoderFeedback(RemoteFeedbackDevice encoder_feedback_remote);
		void setFeedbackCoefficient(double feedback_coefficient);
		void setRemoteFeedbackDeviceId(int device_id, size_t remote_ordinal);
		void setRemoteFeedbackDeviceIds(const std::array<int, 2> &remote_feedback_device_ids);
		void setRemoteFeedbackFilter(RemoteSensorSource remote_sensor_source, size_t remote_ordinal);
		void setRemoteFeedbackFilters(const std::array<RemoteSensorSource, 2> &remote_sensor_sources);
		void setSensorTerm(FeedbackDevice feedback_device, SensorTerm sensor_term);
		void setSensorTerms(const std::array<FeedbackDevice, SensorTerm_Last> &sensor_terms);
		void setEncoderTicksPerRotation(int encoder_ticks_per_rotation);
		void setFaults(unsigned int faults);
		void setStickyFaults(unsigned int sticky_faults);
		void setConversionFactor(double conversion_factor);

		void setClearPositionOnLimitF(bool clear_position_on_limit_f);
		bool getClearPositionOnLimitF(void) const;
		void setClearPositionOnLimitR(bool clear_position_on_limit_r);
		bool getClearPositionOnLimitR(void) const;

		//TalonFX only
		void setMotorCommutation(hardware_interface::MotorCommutation motor_commutation);
		hardware_interface::MotorCommutation getMotorCommutation(void) const;

		//TalonFX only
		void setAbsoluteSensorRange(hardware_interface::AbsoluteSensorRange absolute_sensor_range);
		hardware_interface::AbsoluteSensorRange getAbsoluteSensorRange(void) const;

		//TalonFX only
		void setSensorInitializationStrategy(hardware_interface::SensorInitializationStrategy sensor_initialization_strategy);
		hardware_interface::SensorInitializationStrategy getSensorInitializationStrategy(void) const;

		void setEnableReadThread(bool enable_read_thread);
		void setFirmwareVersion(int firmware_version);
		int getFirmwareVersion(void) const;

	private:
		double setpoint_;
		double position_;
		double speed_;
		double output_voltage_;
		double output_current_;
		double stator_current_;
		double supply_current_;
		double bus_voltage_;
		double motor_output_percent_;
		double temperature_;
		std::array<double, TALON_PIDF_SLOTS> pidf_p_;
		std::array<double, TALON_PIDF_SLOTS> pidf_i_;
		std::array<double, TALON_PIDF_SLOTS> pidf_d_;
		std::array<double, TALON_PIDF_SLOTS> pidf_f_;
		std::array<double, TALON_PIDF_SLOTS> pidf_izone_;
		std::array<double, TALON_PIDF_SLOTS> allowable_closed_loop_error_;
		std::array<double, TALON_PIDF_SLOTS> max_integral_accumulator_;
		std::array<double, TALON_PIDF_SLOTS> closed_loop_peak_output_;
		std::array<int, TALON_PIDF_SLOTS>    closed_loop_period_;
		bool   aux_pid_polarity_;
		double closed_loop_error_;
		double integral_accumulator_;
		double error_derivative_;
		double closed_loop_target_;
		double p_term_;
		double i_term_;
		double d_term_;
		double f_term_;
		double active_trajectory_position_;
		double active_trajectory_velocity_;
		double active_trajectory_arbitrary_feed_forward_;
		double active_trajectory_heading_;
		bool   forward_limit_switch_closed_;
		bool   reverse_limit_switch_closed_;
		bool   forward_softlimit_hit_;
		bool   reverse_softlimit_hit_;

		TalonMode  talon_mode_;
		DemandType demand1_type_;
		double     demand1_value_;

		int can_id_;

		int  slot_;
		bool invert_;
		bool sensor_phase_;

		NeutralMode neutral_mode_;

		FeedbackDevice encoder_feedback_;
		double feedback_coefficient_;
		RemoteFeedbackDevice encoder_feedback_remote_;
		int encoder_ticks_per_rotation_;
		std::array<int, 2> remote_feedback_device_ids_;
		std::array<RemoteSensorSource, 2> remote_feedback_filters_;
		std::array<FeedbackDevice, SensorTerm_Last> sensor_terms_;

		// output shaping
		double close_loop_ramp_;
		double open_loop_ramp_;
		double peak_output_forward_;
		double peak_output_reverse_;
		double nominal_output_forward_;
		double nominal_output_reverse_;
		double neutral_deadband_;

		// voltage compensation
		double voltage_compensation_saturation_;
		int   voltage_measurement_filter_;
		bool  voltage_compensation_enable_;

		hardware_interface::VelocityMeasurementPeriod velocity_measurement_period_;
		int velocity_measurement_window_;

		LimitSwitchSource limit_switch_local_forward_source_;
		LimitSwitchNormal limit_switch_local_forward_normal_;
		LimitSwitchSource limit_switch_local_reverse_source_;
		LimitSwitchNormal limit_switch_local_reverse_normal_;
		RemoteLimitSwitchSource limit_switch_remote_forward_source_;
		LimitSwitchNormal limit_switch_remote_forward_normal_;
		unsigned int limit_switch_remote_forward_id_;
		RemoteLimitSwitchSource limit_switch_remote_reverse_source_;
		LimitSwitchNormal limit_switch_remote_reverse_normal_;
		unsigned int limit_switch_remote_reverse_id_;

		// soft limits
		double softlimit_forward_threshold_;
		bool softlimit_forward_enable_;
		double softlimit_reverse_threshold_;
		bool softlimit_reverse_enable_;
		bool softlimits_override_enable_;

		// current limiting
		int current_limit_peak_amps_;
		int current_limit_peak_msec_;
		int current_limit_continuous_amps_;
		bool current_limit_enable_;

		// TalonFX / Falcon500 only
		double supply_current_limit_;
		double supply_current_trigger_threshold_current_;
		double supply_current_trigger_threshold_time_;
		bool   supply_current_limit_enable_;

		double stator_current_limit_;
		double stator_current_trigger_threshold_current_;
		double stator_current_trigger_threshold_time_;
		bool   stator_current_limit_enable_;

		// Talon expects these in integral sensorUnitsPer100ms,
		// but at this level we're still dealing with
		// radians/sec (or /sec^2 for acceleration)
		double motion_cruise_velocity_;
		double motion_acceleration_;

		int motion_s_curve_strength_;

		// Motion profiling
		int motion_profile_top_level_buffer_count_;
		bool motion_profile_top_level_buffer_full_;
		MotionProfileStatus motion_profile_status_;
		int motion_profile_trajectory_period_;

		std::array<uint8_t, Status_Last> status_frame_periods_;
		std::array<uint8_t, Control_Last> control_frame_periods_;

		unsigned int faults_;
		unsigned int sticky_faults_;

		double conversion_factor_;

		bool clear_position_on_limit_f_{false};
		bool clear_position_on_limit_r_{false};

		// TalonFX / Falcon500 specific
		hardware_interface::MotorCommutation motor_commutation_;
		hardware_interface::AbsoluteSensorRange absolute_sensor_range_;
		hardware_interface::SensorInitializationStrategy sensor_initialization_strategy_;

		bool enable_read_thread_;

		int firmware_version_;
};

// Glue code to let this be registered in the list of
// hardware resources on the robot.  Since state is
// read-only, allow multiple controllers to register it.
using TalonStateHandle = StateHandle<const TalonHWState>;
using TalonWritableStateHandle = StateHandle<TalonHWState>;
class TalonStateInterface : public HardwareResourceManager<TalonStateHandle> {};
class RemoteTalonStateInterface : public HardwareResourceManager<TalonWritableStateHandle, ClaimResources> {};

}

#endif