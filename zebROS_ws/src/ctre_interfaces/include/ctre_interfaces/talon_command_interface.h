#pragma once

#include <memory>
#include <mutex>
#include "ctre_interfaces/talon_state_interface.h"
#include "state_handle/command_handle.h"

namespace hardware_interface
{

struct TrajectoryPoint
{
	TrajectoryPoint(void);
	double   position{0};
	double   velocity{0};
	double   headingRad{0};
	double   arbFeedFwd{0};
	double   auxiliaryPos{0};
	double   auxiliaryVel{0};
	double   auxiliaryArbFeedFwd{0};
	uint32_t profileSlotSelect0{0};
	uint32_t profileSlotSelect1{0};
	bool     isLastPoint{false};
	bool     zeroPos{false};
	int      timeDur{0};
	bool     useAuxPID{false};
};

// Class to buffer data needed to set the state of the
// Talon.  This should (eventually) include anything
// which might be set during runtime.  Config data
// which is set only once at startup can be handled
// in the hardware manager constructor/init rather than through
// this interface.
// Various controller code will set the member vars of
// this class depending on the needs of the motor
// being controlled
// Each pass through write() in the hardware interface
// will use this to re-configure (if necessary) and then
// update the setpoint on the associated Talon.
// The hardware_controller is responsible for keeping
// a master array of these classes - 1 entry per
// physical Talon controller in the robot
class TalonHWCommand
{
	public:
		TalonHWCommand(void);
		~TalonHWCommand();

		bool commandChanged(double &command);
		double get(void) const;

		TalonMode getMode(void) const;

		void setP(double oldP, size_t index);
		double getP(size_t index) const;

		void setI(double ii, size_t index);
		double getI(size_t index) const;

		void setD(double dd, size_t index);
		double getD(size_t index) const;

		void setF(double ff, size_t index);
		double getF(size_t index);

		void setIZ(int i_zone, size_t index);
		int getIZ(size_t index) const;

		void setAllowableClosedloopError(int allowable_closed_loop_error, size_t index);
		int getAllowableClosedloopError(size_t index) const;
		void setMaxIntegralAccumulator(int max_integral_accumulator, size_t index);
		int getMaxIntegralAccumulator(size_t index) const;
		void setClosedLoopPeakOutput(double closed_loop_peak_output, size_t index);
		double getClosedLoopPeakOutput(size_t index) const;

		void setClosedLoopPeriod(int closed_loop_period, size_t index);
		int getClosedLoopPeriod(size_t index) const;
		bool pidfChanged(double &p, double &i, double &d, double &f, int &iz, int &allowable_closed_loop_error, double &max_integral_accumulator, double &closed_loop_peak_output, int &closed_loop_period, size_t index);
		void resetPIDF(size_t index);

		void setAuxPidPolarity(bool aux_pid_polarity);
		bool getAuxPidPolarity(void) const;
		bool auxPidPolarityChanged(bool &aux_pid_polarity);
		void resetAuxPidPolarity(void);

		void setIntegralAccumulator(double iaccum);
		double getIntegralAccumulator(void) const;
		bool integralAccumulatorChanged(double &iaccum);
		void resetIntegralAccumulator(void);

		void set(double command);

		void setMode(TalonMode mode);
		// Check to see if mode changed since last call
		// If so, return true and set mode to new desired
		// talon mode
		// If mode hasn't changed, return false
		// Goal here is to prevent writes to the CAN
		// bus to repeatedly set the mode to the same value.
		// Instead, only send a setMode to a given Talon if
		// the mode has actually changed.
		bool modeChanged(TalonMode &mode);
		void resetMode(void);

		void setDemand1Type(DemandType demand_type);
		DemandType getDemand1Type(void) const;

		void setDemand1Value(double demand_value);
		double getDemand1Value(void) const;

		bool demand1Changed(DemandType &type, double &value);
		void resetDemand1(void);

		void setNeutralMode(NeutralMode neutral_mode);
		bool getNeutralMode(void);
		bool neutralModeChanged(NeutralMode &neutral_mode);

		void setPidfSlot(int pidf_slot);
		int getPidfSlot(void)const;
		bool slotChanged(int &newpidfSlot);
		void resetPidfSlot(void);

		void setInvert(bool invert);
		void setSensorPhase(bool invert);
		bool invertChanged(bool &invert, bool &sensor_phase);

		FeedbackDevice getEncoderFeedback(void) const;
		void setEncoderFeedback(FeedbackDevice encoder_feedback);
		double getFeedbackCoefficient(void) const;
		void setFeedbackCoefficient(double feedback_coefficient);
		bool encoderFeedbackChanged(FeedbackDevice &encoder_feedback, double &feedback_coefficient);
		void resetEncoderFeedback(void);

		RemoteFeedbackDevice getRemoteEncoderFeedback(void) const;
		void setRemoteEncoderFeedback(RemoteFeedbackDevice remote_encoder_feedback);
		bool remoteEncoderFeedbackChanged(RemoteFeedbackDevice &remote_encoder_feedback);
		void resetRemoteEncoderFeedback(void);
		int getRemoteFeedbackDeviceId(size_t remote_ordinal) const;
		RemoteSensorSource getRemoteFeedbackFilter(size_t remote_ordinal) const;

		void setRemoteFeedbackDeviceId(int remote_feedback_device_id, size_t remote_ordinal);
		void setRemoteFeedbackFilter(RemoteSensorSource remote_sensor_source, size_t remote_ordinal);
		bool remoteFeedbackFiltersChanged(std::array<int, 2> &remote_feedback_device_ids, std::array<RemoteSensorSource, 2> &remote_feedback_filters);
		void resetRemoteFeedbackFilters(void);

		FeedbackDevice getSensorTerm(SensorTerm sensor_terms) const;
		void setSensorTerm(FeedbackDevice feedback_device, SensorTerm sensor_terms);
		bool sensorTermsChanged(std::array<FeedbackDevice, SensorTerm_Last> &sensor_terms);
		void resetSensorTerms(void);

		int getEncoderTicksPerRotation(void) const;

		void setEncoderTicksPerRotation(int encoder_ticks_per_rotation);

		//output shaping
		void setClosedloopRamp(double closed_loop_ramp);
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
		bool outputShapingChanged(double &closed_loop_ramp,
								  double &open_loop_ramp,
								  double &peak_output_forward,
								  double &peak_output_reverse,
								  double &nominal_output_forward,
								  double &nominal_output_reverse,
								  double &neutral_deadband);
		void resetOutputShaping(void);

		void setVoltageCompensationSaturation(double voltage);
		double getVoltageCompensationSaturation(void) const;

		void setVoltageMeasurementFilter(int filterWindowSamples);
		int getVoltageMeasurementFilter(void) const;

		void setVoltageCompensationEnable(bool enable);

		bool getEnableVoltageCompenation(void) const;

		bool voltageCompensationChanged(double &voltage_compensation_saturation,
										int &voltage_measurement_filter,
										bool &voltage_compensation_enable);
		void resetVoltageCompensation(void);

		void setVelocityMeasurementPeriod(hardware_interface::VelocityMeasurementPeriod period);

		bool getVoltageMeasurementPeriod(void) const;

		void setVelocityMeasurementWindow(int window);

		bool getVoltageMeasurementWindow(void) const;

		bool velocityMeasurementChanged(hardware_interface::VelocityMeasurementPeriod &period,
										int &window);
		void resetVelocityMeasurement(void);

		void setSelectedSensorPosition(double position);
		double getSelectedSensorPosition(void) const;

		bool sensorPositionChanged(double &position);
		void resetSensorPosition(void);


		void setForwardLimitSwitchSource(LimitSwitchSource source, LimitSwitchNormal normal);

		void getForwardLimitSwitchSource(LimitSwitchSource &source, LimitSwitchNormal &normal) const;

		void setReverseLimitSwitchSource(LimitSwitchSource source, LimitSwitchNormal normal);

		void getReverseLimitSwitchSource(LimitSwitchSource &source, LimitSwitchNormal &normal) const;

		bool limitSwitchesSourceChanged(LimitSwitchSource &forward_source, LimitSwitchNormal &forward_normal, LimitSwitchSource &reverse_source, LimitSwitchNormal &reverse_normal);
		void resetLimitSwitchesSource(void);

		void setRemoteForwardLimitSwitchSource(RemoteLimitSwitchSource source, LimitSwitchNormal normal, unsigned int id);

		void getRemoteForwardLimitSwitchSource(RemoteLimitSwitchSource &source, LimitSwitchNormal &normal, unsigned int &id) const;

		void setRemoteReverseLimitSwitchSource(RemoteLimitSwitchSource source, LimitSwitchNormal normal, unsigned int id);

		void getRemoteReverseLimitSwitchSource(RemoteLimitSwitchSource &source, LimitSwitchNormal &normal, unsigned int &id) const;

		bool remoteLimitSwitchesSourceChanged(RemoteLimitSwitchSource &forward_source, LimitSwitchNormal &forward_normal, unsigned int &forward_id, RemoteLimitSwitchSource &reverse_source, LimitSwitchNormal &reverse_normal, unsigned int &reverse_id);
		void resetRemoteLimitSwitchesSource(void);

		// softlimits
		void setForwardSoftLimitThreshold(double threshold);
		double getForwardSoftLimitThreshold(void) const;

		void setForwardSoftLimitEnable(bool enable);
		bool getForwardSoftLimitEnable(void) const;
		void setReverseSoftLimitThreshold(double threshold);
		double getReverseSoftLimitThreshold(void) const;

		void setReverseSoftLimitEnable(bool enable);
		bool getReverseSoftLimitEnable(void) const;

		void setOverrideSoftLimitsEnable(bool enable);
		bool getOverrideSoftsLimitEnable(void) const;

		bool softLimitChanged(double &forward_threshold, bool &forward_enable, double &reverse_threshold, bool &reverse_enable, bool &override_enable);
		void resetSoftLimit(void);

		// current limits - Talon SRX only
		void setPeakCurrentLimit(int amps);
		int getPeakCurrentLimit(void) const;

		void setPeakCurrentDuration(int msec);
		int getPeakCurrentDuration(void) const;
		void setContinuousCurrentLimit(int amps);
		int getContinuousCurrentLimit(void) const;
		void setCurrentLimitEnable(bool enable);
		bool getCurrentLimitEnable(void) const;

		bool currentLimitChanged(int &peak_amps, int &peak_msec, int &continuous_amps, bool &enable);
		void resetCurrentLimit(void);

		// Current Limits - Talon FX / Falcon 500
		void setSupplyCurrentLimit(double supply_current_limit);
		double getSupplyCurrentLimit(void) const;
		void setSupplyCurrentTriggerThresholdCurrent(double supply_current_trigger_threshold_current);
		double getSupplyCurrentTriggerThresholdCurrent(void) const;
		void setSupplyCurrentTriggerThresholdTime(double supply_current_trigger_threshold_time);
		double getSupplyCurrentTriggerThresholdTime(void) const;
		void setSupplyCurrentLimitEnable(bool supply_current_limit_enable);
		bool getSupplyCurrentLimitEnable(void) const;
		bool supplyCurrentLimitChanged(double &stator_current_limit,
				double &supply_current_trigger_threshold_current,
				double &supply_time_trigger_threshold_time,
				bool &supply_current_limit_enable);
		void  resetSupplyCurrentLimit(void);

		void setStatorCurrentLimit(bool stator_current_limit);
		double getStatorCurrentLimit(void) const;
		void setStatorCurrentTriggerThresholdCurrent(double stator_current_trigger_threshold_current);
		double getStatorCurrentTriggerThresholdCurrent(void) const;
		void setStatorCurrentTriggerThresholdTime(double stator_current_trigger_threshold_time);
		double getStatorCurrentTriggerThresholdTime(void) const;
		void setStatorCurrentLimitEnable(bool stator_current_limit_enable);
		bool getStatorCurrentLimitEnable(void) const;
		bool statorCurrentLimitChanged(double &stator_current_limit,
				double &stator_current_trigger_threshold_current,
				double &stator_time_trigger_threshold_time,
				bool &stator_current_limit_enable);
		void  resetStatorCurrentLimit(void);

		void setMotionCruiseVelocity(double velocity);
		double getMotionCruiseVelocity(void) const;
		void setMotionAcceleration(double acceleration);
		double getMotionAcceleration(void) const;
		void setMotionSCurveStrength(int s_curve_strength);
		int getMotionSCurveStrength(void) const;

		bool motionCruiseChanged(double &velocity, double &acceleration, int &s_curve_strength);
		void resetMotionCruise(void);

		// This is a one shot - when set, it needs to
		// call the appropriate Talon function once
		// then clear itself
		void setClearMotionProfileTrajectories(void);
		bool getClearMotionProfileTrajectories(void) const;
		bool clearMotionProfileTrajectoriesChanged(void);
		void PushMotionProfileTrajectory(const TrajectoryPoint &traj_pt);
		std::vector<TrajectoryPoint> getMotionProfileTrajectories(void) const;
		bool motionProfileTrajectoriesChanged(std::vector<TrajectoryPoint> &points);

		// This is a one shot - when set, it needs to
		// call the appropriate Talon function once
		// then clear itself
		void setClearMotionProfileHasUnderrun(void);
		bool getClearMotionProfileHasUnderrun(void) const;
		bool clearMotionProfileHasUnderrunChanged(void);

		void setStatusFramePeriod(StatusFrame status_frame, uint8_t period);

		uint8_t getStatusFramePeriod(StatusFrame status_frame) const;

		bool statusFramePeriodChanged(StatusFrame status_frame, uint8_t &period);
		void resetStatusFramePeriod(StatusFrame status_frame);

		void setControlFramePeriod(ControlFrame control_frame, uint8_t period);

		uint8_t getControlFramePeriod(ControlFrame control_frame) const;

		bool controlFramePeriodChanged(ControlFrame control_frame, uint8_t &period);
		void resetControlFramePeriod(ControlFrame control_frame);

		void setMotionProfileTrajectoryPeriod(int msec);
		int getMotionProfileTrajectoryPeriod(void) const;
		bool motionProfileTrajectoryPeriodChanged(int &msec);
		void resetMotionProfileTrajectoryPeriod(void);

		void setClearStickyFaults(void);
		bool getClearStickyFaults(void) const;
		bool clearStickyFaultsChanged(void);

		void setClearPositionOnLimitF(bool clear_position_on_limit_f);
		bool getClearPositionOnLimitF(void) const;
		bool clearPositionOnLimitFChanged(bool &clear_position_on_limit_f);
		void resetClearPositionOnLimitF(void);

		void setClearPositionOnLimitR(bool clear_position_on_limit_r);
		bool getClearPositionOnLimitR(void) const;
		bool clearPositionOnLimitRChanged(bool &clear_position_on_limit_r);
		void resetClearPositionOnLimitR(void);

		void setConversionFactor(double conversion_factor);
		double getConversionFactor(void) const;

		//TalonFX only
		void setMotorCommutation(hardware_interface::MotorCommutation motor_commutation);
		hardware_interface::MotorCommutation getMotorCommutation(void) const;
		bool motorCommutationChanged(hardware_interface::MotorCommutation &motor_commutation);
		void resetMotorCommutation(void);

		//TalonFX only
		void setAbsoluteSensorRange(hardware_interface::AbsoluteSensorRange absolute_sensor_range);
		hardware_interface::AbsoluteSensorRange getAbsoluteSensorRange(void) const;
		bool absoluteSensorRangeChanged(hardware_interface::AbsoluteSensorRange &absolute_sensor_range);
		void resetAbsoluteSensorRange(void);

		//TalonFX only
		void setSensorInitializationStrategy(hardware_interface::SensorInitializationStrategy sensor_initialization_strategy);
		hardware_interface::SensorInitializationStrategy getSensorInitializationStrategy(void) const;
		bool sensorInitializationStrategyChanged(hardware_interface::SensorInitializationStrategy &sensor_initialization_strategy);
		void resetSensorInitializationStrategy(void);

		void setEnableReadThread(bool enable_read_thread);
		bool getEnableReadThread(void) const;

		bool enableReadThreadChanged(bool &enable_read_thread);

		void lock(void);
		std::shared_ptr<std::mutex> mutex(void);

	private:
		double    command_; // motor setpoint - % vbus, velocity, position, etc
		bool      command_changed_;
		TalonMode mode_;         // talon mode - % vbus, close loop, motion profile, etc
		bool      mode_changed_; // set if mode needs to be updated on the talon hw
		DemandType demand1_type_;
		double    demand1_value_;
		bool      demand1_changed_;
		//RG: shouldn't there be a variable for the peak voltage limits?
		int       pidf_slot_; // index 0 or 1 of the active PIDF slot
		bool      pidf_slot_changed_; // set to true to trigger a write to PIDF select on Talon
		double    iaccum_;
		bool      iaccum_changed_;

		bool      invert_;
		bool      sensor_phase_;
		bool      invert_changed_;

		NeutralMode neutral_mode_;
		bool        neutral_mode_changed_;

		FeedbackDevice encoder_feedback_;
		double         feedback_coefficient_;
		bool           encoder_feedback_changed_;
		RemoteFeedbackDevice remote_encoder_feedback_;
		bool                 remote_encoder_feedback_changed_;
		int                  encoder_ticks_per_rotation_;
		std::array<int, 2>                remote_feedback_device_ids_;
		std::array<RemoteSensorSource, 2> remote_feedback_filters_;
		bool                              remote_feedback_filters_changed_;
		std::array<FeedbackDevice, SensorTerm_Last> sensor_terms_;
		bool                                        sensor_terms_changed_;

		//output shaping
		double closed_loop_ramp_;
		double open_loop_ramp_;
		double peak_output_forward_;
		double peak_output_reverse_;
		double nominal_output_forward_;
		double nominal_output_reverse_;
		double neutral_deadband_;
		bool   output_shaping_changed_;

		double voltage_compensation_saturation_;
		int   voltage_measurement_filter_;
		bool  voltage_compensation_enable_;
		bool  voltage_compensation_changed_;

		hardware_interface::VelocityMeasurementPeriod velocity_measurement_period_;
		int velocity_measurement_window_;
		bool velocity_measurement_changed_;

		double sensor_position_value_;
		bool sensor_position_changed_;

		LimitSwitchSource limit_switch_local_forward_source_;
		LimitSwitchNormal limit_switch_local_forward_normal_;
		LimitSwitchSource limit_switch_local_reverse_source_;
		LimitSwitchNormal limit_switch_local_reverse_normal_;
		bool              limit_switch_local_changed_;
		RemoteLimitSwitchSource limit_switch_remote_forward_source_;
		LimitSwitchNormal       limit_switch_remote_forward_normal_;
		unsigned int            limit_switch_remote_forward_id_;
		RemoteLimitSwitchSource limit_switch_remote_reverse_source_;
		LimitSwitchNormal       limit_switch_remote_reverse_normal_;
		unsigned int            limit_switch_remote_reverse_id_;
		bool                    limit_switch_remote_changed_;

		double softlimit_forward_threshold_;
		bool softlimit_forward_enable_;
		double softlimit_reverse_threshold_;
		bool softlimit_reverse_enable_;
		bool softlimits_override_enable_;
		bool softlimit_changed_;

		// Talon SRX only
		int current_limit_peak_amps_;
		int current_limit_peak_msec_;
		int current_limit_continuous_amps_;
		bool current_limit_enable_;
		bool current_limit_changed_;

		// TalonFX / Falcon500 only
		double supply_current_limit_;
		double supply_current_trigger_threshold_current_;
		double supply_current_trigger_threshold_time_;
		bool   supply_current_limit_enable_;
		bool   supply_current_limit_changed_;

		double stator_current_limit_;
		double stator_current_trigger_threshold_current_;
		double stator_current_trigger_threshold_time_;
		bool   stator_current_limit_enable_;
		bool   stator_current_limit_changed_;

		// Talon expects the next two in integral sensorUnitsPer100ms,
		// but at this level we're still dealing with
		// radians/sec (or /sec^2 for acceleration)
		double motion_cruise_velocity_;
		double motion_acceleration_;
		int motion_s_curve_strength_;
		bool motion_cruise_changed_;

		bool motion_profile_clear_trajectories_;
		bool motion_profile_clear_has_underrun_;
		std::vector<TrajectoryPoint> motion_profile_trajectory_points_;
		int motion_profile_profile_trajectory_period_;
		bool motion_profile_profile_trajectory_period_changed_;
		std::array<uint8_t, Status_Last> status_frame_periods_;
		std::array<bool, Status_Last> status_frame_periods_changed_;

		std::array<uint8_t, Control_Last> control_frame_periods_;
		std::array<bool, Control_Last> control_frame_periods_changed_;

		bool clear_sticky_faults_;

		// TALON_PIDF_SLOTS entries in the Talon HW for each of these settings
		std::array<double, TALON_PIDF_SLOTS> p_;
		std::array<double, TALON_PIDF_SLOTS> i_;
		std::array<double, TALON_PIDF_SLOTS> d_;
		std::array<double, TALON_PIDF_SLOTS> f_;
		std::array<int, TALON_PIDF_SLOTS>    i_zone_;
		std::array<int, TALON_PIDF_SLOTS>    allowable_closed_loop_error_;
		std::array<double, TALON_PIDF_SLOTS> max_integral_accumulator_;
		std::array<double, TALON_PIDF_SLOTS> closed_loop_peak_output_;
		std::array<int, TALON_PIDF_SLOTS>    closed_loop_period_;
		std::array<bool, TALON_PIDF_SLOTS>   pidf_changed_;
		bool   aux_pid_polarity_;
		bool   aux_pid_polarity_changed_;

		double conversion_factor_;

		bool clear_position_on_limit_f_{false};
		bool clear_position_on_limit_f_changed_{false};
		bool clear_position_on_limit_r_{false};
		bool clear_position_on_limit_r_changed_{false};

		// TalonFX / Falcon500 specific
		hardware_interface::MotorCommutation motor_commutation_;
		bool                                 motor_commutation_changed_;

		hardware_interface::AbsoluteSensorRange absolute_sensor_range_;
		bool                                    absolute_sensor_range_changed_;

		hardware_interface::SensorInitializationStrategy sensor_initialization_strategy_;
		bool                                             sensor_initialization_strategy_changed_;

		bool enable_read_thread_;
		bool enable_read_thread_changed_;

		// Normally the read-update-write process will lead to
		// sequential operation.  controller init happens asynchronously,
		// though, so lock the command entry for a given talon when
		// that talon's controller is being initialized
		std::shared_ptr<std::mutex> mutex_;

		static constexpr double double_value_epsilon = 0.0001;
};

// Create a handle pointing to a type TalonHWCommand / TalonHWState pair
typedef CommandHandle<TalonHWCommand, TalonHWState, TalonStateHandle> TalonCommandHandle;

// Use ClaimResources here since we only want 1 controller
// to be able to access a given Talon at any particular time
class TalonCommandInterface : public HardwareResourceManager<TalonCommandHandle, ClaimResources> {};
}
