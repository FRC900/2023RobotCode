#include <cmath>
#include "pigeon2_controller/pigeon2_controller_interface.h"

namespace pigeon2_controller_interface
{
Pigeon2CIParams::Pigeon2CIParams(ros::NodeHandle n)
	: DDRUpdater(n)
{
	// Set values to sensible defaults
	velocity_meas_period_ = hardware_interface::pigeon2::SensorVelocityMeasPeriod::Sensor_Period_100Ms;
	velocity_meas_window_ = 64;
	absolute_sensor_range_ = hardware_interface::pigeon2::AbsoluteSensorRange::Unsigned_0_to_360;
	magnet_offset_ = 0;
	initialization_strategy_ = hardware_interface::pigeon2::SensorInitializationStrategy::BootToZero;
	feedback_coefficient_ = 2. * M_PI / 4096.;
	setUnitString("rad");
	time_base_ = hardware_interface::pigeon2::SensorTimeBase::PerSecond;
	direction_ = false;
	sensor_data_status_frame_period_ = -1;
	vbat_and_faults_status_frame_period_ = -1;
	conversion_factor_ = 1.0;

	// Then read in values from config file
	readIntoEnum(n, "velocity_meas_period", velocity_measurement_period_enum_map_, velocity_meas_period_);
	readIntoScalar(n, "velocity_meas_window", velocity_meas_window_);
	readIntoEnum(n, "absolute_sensor_range", absolute_sensor_range_enum_map_, absolute_sensor_range_);
	readIntoScalar(n, "magnet_offset", magnet_offset_);
	readIntoEnum(n, "initialization_strategy", sensor_initialization_strategy_enum_map_, initialization_strategy_);
	readIntoScalar(n, "feedback_coefficient", feedback_coefficient_);
	std::string tmp;
	setUnitString(tmp);
	readIntoEnum(n, "time_base", sensor_time_base_enum_map_, time_base_);
	readIntoScalar(n, "direction", direction_);
	readIntoScalar(n, "sensor_data_status_frame_period", sensor_data_status_frame_period_);
	readIntoScalar(n, "vbat_and_faults_status_frame_period", vbat_and_faults_status_frame_period_);
	readIntoScalar(n, "conversion_factor", conversion_factor_);

	// Then hook them up to dynamic reconfigure options
	ddr_.registerEnumVariable<int>("velocity_meas_period", boost::bind(&Pigeon2CIParams::getVelocityMeasPeriod, this), boost::bind(&Pigeon2CIParams::setVelocityMeasPeriod, this, _1, false), "Velocity Measurement Period", velocity_measurement_period_enum_map_);
	ddr_.registerVariable<int>("velocity_meas_window", boost::bind(&Pigeon2CIParams::getVelocityMeasWindow, this), boost::bind(&Pigeon2CIParams::setVelocityMeasWindow, this, _1, false), "Velocity Measurement Window", 0, 512);
	ddr_.registerEnumVariable<int>("absolute_sensor_range", boost::bind(&Pigeon2CIParams::getAbsoluteSensorRange, this), boost::bind(&Pigeon2CIParams::setAbsoluteSensorRange, this, _1, false), "Absolute Sensor Range", absolute_sensor_range_enum_map_);
	ddr_.registerVariable<double>("magnet_offset", boost::bind(&Pigeon2CIParams::getMagnetOffset, this), boost::bind(&Pigeon2CIParams::setMagnetOffset, this, _1, false), "Velocity Measurement Window", -180.0, 360.0);
	ddr_.registerEnumVariable<int>("sensor_initialization_strategy", boost::bind(&Pigeon2CIParams::getInitializationStrategy, this), boost::bind(&Pigeon2CIParams::setInitializationStrategy, this, _1, false), "Sensor Initialization Strategy", sensor_initialization_strategy_enum_map_);
	ddr_.registerVariable<double>("feedback_coefficient", boost::bind(&Pigeon2CIParams::getFeedbackCoefficient, this), boost::bind(&Pigeon2CIParams::setFeedbackCoefficient, this, _1, false), "Feedback Coefficient", 0., 4096.);
	ddr_.registerVariable<std::string>("unit_string", boost::bind(&Pigeon2CIParams::getUnitString, this), boost::bind(&Pigeon2CIParams::setUnitString, this, _1, false), "Unit String");
	ddr_.registerVariable<int>("sensor_data_status_frame_period", boost::bind(&Pigeon2CIParams::getSensorDataStatusFramePeriod, this), boost::bind(&Pigeon2CIParams::setSensorDataStatusFramePeriod, this, _1, false), "Sensor Data Status Frame Period", 0, 255);
	ddr_.registerVariable<int>("vbat_and_faults_status_frame_period", boost::bind(&Pigeon2CIParams::getVBatAndFaultsStatusFramePeriod, this), boost::bind(&Pigeon2CIParams::setVBatAndFaultsStatusFramePeriod, this, _1, false), "VBat and Faults Status Frame Period", 0, 255);
	ddr_.registerVariable<double>("conversion_factor", boost::bind(&Pigeon2CIParams::getConversionFactor, this), boost::bind(&Pigeon2CIParams::setConversionFactor, this, _1, false), "Conversion Factor", 0., 100.);
	ddr_.publishServicesTopics();
}

// Functions to update params from either DDR callbacks or the interface.
// First arg holds the value to set
// Second arg is true if coming from the interface, meaning it needs
// to force an update to the dynamic interface values to stay in sync
// Second arg is false if the source of the call is a DDR update. In
// that case, don't update the dynamic interface value since
// the value has already been updated there.
void Pigeon2CIParams::setVelocityMeasPeriod(int velocity_meas_period, bool update_dynamic)
{
	const bool publish_update = update_dynamic && (velocity_meas_period != velocity_meas_period_);
	velocity_meas_period_ = static_cast<hardware_interface::pigeon2::SensorVelocityMeasPeriod>(velocity_meas_period);
	if (publish_update)
	{
		triggerDDRUpdate();
	}
}
void Pigeon2CIParams::setVelocityMeasWindow(int velocity_meas_window, bool update_dynamic)
{
	const bool publish_update = update_dynamic && (velocity_meas_window != velocity_meas_window_);
	velocity_meas_window_ = velocity_meas_window;
	if (publish_update)
	{
		triggerDDRUpdate();
	}
}
void Pigeon2CIParams::setAbsoluteSensorRange(int absolute_sensor_range, bool update_dynamic)
{
	const bool publish_update = update_dynamic && (absolute_sensor_range != absolute_sensor_range_);
	absolute_sensor_range_ = static_cast<hardware_interface::pigeon2::AbsoluteSensorRange>(absolute_sensor_range);
	if (publish_update)
	{
		triggerDDRUpdate();
	}
}
void Pigeon2CIParams::setMagnetOffset(double magnet_offset, bool update_dynamic)
{
	const bool publish_update = update_dynamic && (magnet_offset != magnet_offset_);
	magnet_offset_ = magnet_offset;
	if (publish_update)
	{
		triggerDDRUpdate();
	}
}
void Pigeon2CIParams::setInitializationStrategy(int initialization_strategy, bool update_dynamic)
{
	const bool publish_update = update_dynamic && (initialization_strategy != initialization_strategy_);
	initialization_strategy_ = static_cast<hardware_interface::pigeon2::SensorInitializationStrategy>(initialization_strategy);
	if (publish_update)
	{
		triggerDDRUpdate();
	}
}
void Pigeon2CIParams::setFeedbackCoefficient(double feedback_coefficient, bool update_dynamic)
{
	const bool publish_update = update_dynamic && (feedback_coefficient != feedback_coefficient_);
	feedback_coefficient_ = feedback_coefficient;
	if (publish_update)
	{
		triggerDDRUpdate();
	}
}
void Pigeon2CIParams::setUnitString(const std::string &unit_string, bool update_dynamic)
{
	bool publish_update;
	{
		std::lock_guard<std::mutex> l(unit_string_mutex_);
		publish_update = update_dynamic && (unit_string != unit_string_);
		unit_string_ = unit_string;
	}
	if (publish_update)
	{
		triggerDDRUpdate();
	}
}
void Pigeon2CIParams::setTimeBase(int time_base, bool update_dynamic)
{
	const bool publish_update = update_dynamic && (time_base != time_base_);
	time_base_ = static_cast<hardware_interface::pigeon2::SensorTimeBase>(time_base);
	if (publish_update)
	{
		triggerDDRUpdate();
	}
}
void Pigeon2CIParams::setDirection(bool direction, bool update_dynamic)
{
	const bool publish_update = update_dynamic && (direction != direction_);
	direction_ = direction;
	if (publish_update)
	{
		triggerDDRUpdate();
	}
}
void Pigeon2CIParams::setSensorDataStatusFramePeriod(int sensor_data_status_frame_period, bool update_dynamic)
{
	const bool publish_update = update_dynamic && (sensor_data_status_frame_period != sensor_data_status_frame_period_);
	sensor_data_status_frame_period_ = sensor_data_status_frame_period;
	if (publish_update)
	{
		triggerDDRUpdate();
	}
}
void Pigeon2CIParams::setVBatAndFaultsStatusFramePeriod(int vbat_and_faults_status_frame_period, bool update_dynamic)
{
	const bool publish_update = update_dynamic && (vbat_and_faults_status_frame_period != vbat_and_faults_status_frame_period_);
	vbat_and_faults_status_frame_period_ = vbat_and_faults_status_frame_period;
	if (publish_update)
	{
		triggerDDRUpdate();
	}
}
void Pigeon2CIParams::setConversionFactor(double conversion_factor, bool update_dynamic)
{
	const bool publish_update = update_dynamic && (conversion_factor != conversion_factor_);
	conversion_factor_ = conversion_factor;
	if (publish_update)
	{
		triggerDDRUpdate();
	}
}

hardware_interface::pigeon2::SensorVelocityMeasPeriod Pigeon2CIParams::getVelocityMeasPeriod(void) const
{
	return velocity_meas_period_;
}
int Pigeon2CIParams::getVelocityMeasWindow(void) const
{
	return velocity_meas_window_;
}
hardware_interface::pigeon2::AbsoluteSensorRange Pigeon2CIParams::getAbsoluteSensorRange(void) const
{
	return absolute_sensor_range_;
}
double Pigeon2CIParams::getMagnetOffset(void) const
{
	return magnet_offset_;
}
hardware_interface::pigeon2::SensorInitializationStrategy Pigeon2CIParams::getInitializationStrategy(void) const
{
	return initialization_strategy_;
}
double Pigeon2CIParams::getFeedbackCoefficient(void) const
{
	return feedback_coefficient_;
}
std::string Pigeon2CIParams::getUnitString(void)
{
	std::lock_guard<std::mutex> l(unit_string_mutex_);
	return unit_string_;
}
hardware_interface::pigeon2::SensorTimeBase Pigeon2CIParams::getTimeBase(void) const
{
	return time_base_;
}
double Pigeon2CIParams::getDirection(void) const
{
	return direction_;
}
int Pigeon2CIParams::getSensorDataStatusFramePeriod(void) const
{
	return sensor_data_status_frame_period_;
}
int Pigeon2CIParams::getVBatAndFaultsStatusFramePeriod(void) const
{
	return vbat_and_faults_status_frame_period_;
}
double Pigeon2CIParams::getConversionFactor(void) const
{
	return conversion_factor_;
}

Pigeon2ControllerInterface::Pigeon2ControllerInterface(ros::NodeHandle &n, const std::string &joint_name, hardware_interface::pigeon2::Pigeon2CommandHandle handle)
	: params_(ros::NodeHandle(n, joint_name))
	, handle_(handle)
	, set_position_{false}
	, new_position_{0.0}
	, set_position_to_absolute_{false}
	, clear_sticky_faults_{false}
{
}

void Pigeon2ControllerInterface::update(void)
{
	handle_->setVelocityMeasPeriod(params_.getVelocityMeasPeriod());
	handle_->setVelocityMeasWindow(params_.getVelocityMeasWindow());
	handle_->setAbsoluteSensorRange(params_.getAbsoluteSensorRange());
	handle_->setMagnetOffset(params_.getMagnetOffset());
	handle_->setInitializationStrategy(params_.getInitializationStrategy());
	handle_->setFeedbackCoefficient(params_.getFeedbackCoefficient());
	handle_->setUnitString(params_.getUnitString());
	handle_->setTimeBase(params_.getTimeBase());
	handle_->setDirection(params_.getDirection());
	handle_->setSensorDataStatusFramePeriod(params_.getSensorDataStatusFramePeriod());
	handle_->setVBatAndFaultsStatusFramePeriod(params_.getVBatAndFaultsStatusFramePeriod());
	handle_->setConversionFactor(params_.getConversionFactor());
	if (set_position_)
	{
		handle_->setPosition(new_position_);
		set_position_ = false;
	}

	{
		std::unique_lock<std::mutex> l(set_position_mutex_, std::try_to_lock);
		if (l.owns_lock() && set_position_to_absolute_)
		{
			handle_->setPositionToAbsolute();
			set_position_to_absolute_ = false;
		}
	}

	if (clear_sticky_faults_)
	{
		handle_->setClearStickyFaults();
		clear_sticky_faults_ = false;
	}
}

void Pigeon2ControllerInterface::setPosition(double new_quadrature_position)
{
	std::lock_guard<std::mutex> l(set_position_mutex_);
	set_position_ = true;
	new_position_ = new_quadrature_position;
}

void Pigeon2ControllerInterface::setPositionToAbsolute(void)
{
	set_position_to_absolute_ = true;
}

void Pigeon2ControllerInterface::setClearStickyFaults(void)
{
	clear_sticky_faults_ = true;
}

void Pigeon2ControllerInterface::setVelocityMeasPeriod(hardware_interface::pigeon2::SensorVelocityMeasPeriod velocity_meas_period)
{
	params_.setVelocityMeasPeriod(velocity_meas_period);
}
void Pigeon2ControllerInterface::setVelocityMeasWindow(int velocity_meas_window)
{
	params_.setVelocityMeasWindow(velocity_meas_window);
}
void Pigeon2ControllerInterface::setAbsoluteSensorRange(hardware_interface::pigeon2::AbsoluteSensorRange absolute_sensor_range)
{
	params_.setAbsoluteSensorRange(absolute_sensor_range);
}
void Pigeon2ControllerInterface::setMagnetOffset(double magnet_offset)
{
	params_.setMagnetOffset(magnet_offset);
}
void Pigeon2ControllerInterface::setInitializationStrategy(hardware_interface::pigeon2::SensorInitializationStrategy initialization_strategy)
{
	params_.setInitializationStrategy(initialization_strategy);
}
void Pigeon2ControllerInterface::setFeedbackCoefficient(double feedback_coefficient)
{
	params_.setFeedbackCoefficient(feedback_coefficient);
}
void Pigeon2ControllerInterface::setUnitString(const std::string &unit_string)
{
	params_.setUnitString(unit_string);
}
void Pigeon2ControllerInterface::setTimeBase(hardware_interface::pigeon2::SensorTimeBase time_base)
{
	params_.setTimeBase(time_base);
}
void Pigeon2ControllerInterface::setDirection(bool direction)
{
	params_.setDirection(direction);
}
void Pigeon2ControllerInterface::setSensorDataStatusFramePeriod(int sensor_data_status_frame_period)
{
	params_.setSensorDataStatusFramePeriod(sensor_data_status_frame_period);
}
void Pigeon2ControllerInterface::setVBatAndFaultsStatusFramePeriod(int vbat_and_faults_status_frame_period)
{
	params_.setVBatAndFaultsStatusFramePeriod(vbat_and_faults_status_frame_period);
}
void Pigeon2ControllerInterface::setConversionFactor(double conversion_factor)
{
	params_.setConversionFactor(conversion_factor);
}
} // namespace pigeon2_controller_interface

