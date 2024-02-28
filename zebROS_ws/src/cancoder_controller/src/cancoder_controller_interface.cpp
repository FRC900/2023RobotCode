#include <cmath>
#include "cancoder_controller/cancoder_controller_interface.h"

namespace cancoder_controller_interface
{
CANCoderCIParams::CANCoderCIParams(const ros::NodeHandle &n)
	: DDRUpdater(n)
{

	bool dynamic_reconfigure = false;
	n.param<bool>("dynamic_reconfigure", dynamic_reconfigure, dynamic_reconfigure);
	if (!dynamic_reconfigure)
	{
		shutdownDDRUpdater();
	}
	else
	{
		// Then hook them up to dynamic reconfigure options
		// TODO : these can be lambda functions
		ddr_.registerEnumVariable<int>("sensor_direction", [this]() { return static_cast<int>(sensor_direction_.load()); }, boost::bind(&CANCoderCIParams::setSensorDirection, this, _1, false), "Sensor Direction", sensor_direction_enum_map_);
		ddr_.registerVariable<double>("magnet_offset", [this]() { return static_cast<double>(magnet_offset_.load()); }, boost::bind(&CANCoderCIParams::setMagnetOffset, this, _1, false), "Magnet Offset", -M_PI, M_PI);
		ddr_.registerEnumVariable<int>("absolute_sensor_range", [this]() { return static_cast<int>(absolute_sensor_range_.load()); }, boost::bind(&CANCoderCIParams::setAbsoluteSensorRange, this, _1, false), "Absolute Sensor Range", absolute_sensor_range_enum_map_);
		ddr_.registerVariable<double>("conversion_factor", [this]() { return static_cast<double>(conversion_factor_.load()); }, boost::bind(&CANCoderCIParams::setConversionFactor, this, _1, false), "Conversion Factor", 0., 1000.);
		ddr_.registerVariable<bool>("enable_read_thread", [this]() { return static_cast<bool>(enable_read_thread_.load()); }, boost::bind(&CANCoderCIParams::setEnableReadThread, this, _1, false), "Enable Read Thread");
		ddr_.publishServicesTopics();
	}

	// Override default values with config params, if present
	readIntoEnum(n, "sensor_direction", sensor_direction_enum_map_, sensor_direction_);
	readIntoScalar(n, "magnet_offset", magnet_offset_);
	readIntoEnum(n, "absolute_sensor_range", absolute_sensor_range_enum_map_, absolute_sensor_range_);
	readIntoScalar(n, "conversion_factor", conversion_factor_);
	readIntoScalar(n, "enable_read_thread", enable_read_thread_);
}

// Functions to update params from either DDR callbacks or the interface.
// First arg holds the value to set
// Second arg is true if coming from the interface, meaning it needs
// to force an update to the dynamic interface values to stay in sync
// Second arg is false if the source of the call is a DDR update. In
// that case, don't update the dynamic interface value since
// the value has already been updated there.
void CANCoderCIParams::setSensorDirection(const int sensor_direction, bool update_dynamic)
{
	const auto sensor_direction_enum = static_cast<hardware_interface::cancoder::SensorDirection>(sensor_direction);
	const bool publish_update = update_dynamic && (sensor_direction_enum != sensor_direction_);
	sensor_direction_ = sensor_direction_enum;
	if (publish_update)
	{
		triggerDDRUpdate();
	}
}
void CANCoderCIParams::setMagnetOffset(const double magnet_offset, bool update_dynamic)
{
	const bool publish_update = update_dynamic && (magnet_offset != magnet_offset_);
	magnet_offset_ = magnet_offset;
	if (publish_update)
	{
		triggerDDRUpdate();
	}
}
void CANCoderCIParams::setAbsoluteSensorRange(const int absolute_sensor_range, bool update_dynamic)
{
	const auto absolute_sensor_range_enum = static_cast<hardware_interface::cancoder::AbsoluteSensorRange>(absolute_sensor_range);
	const bool publish_update = update_dynamic && (absolute_sensor_range_enum != absolute_sensor_range_);
	absolute_sensor_range_ = absolute_sensor_range_enum;
	if (publish_update)
	{
		triggerDDRUpdate();
	}
}
void CANCoderCIParams::setConversionFactor(const double conversion_factor, bool update_dynamic)
{
	const bool publish_update = update_dynamic && (conversion_factor != conversion_factor_);
	conversion_factor_ = conversion_factor;
	if (publish_update)
	{
		triggerDDRUpdate();
	}
}
void CANCoderCIParams::setEnableReadThread(const bool enable_read_thread, bool update_dynamic)
{
	const bool publish_update = update_dynamic && (enable_read_thread != enable_read_thread_);
	enable_read_thread_ = enable_read_thread;
	if (publish_update)
	{
		triggerDDRUpdate();
	}
}

hardware_interface::cancoder::SensorDirection CANCoderCIParams::getSensorDirection(void) const
{
	return sensor_direction_;
}
double CANCoderCIParams::getMagnetOffset(void) const
{
	return magnet_offset_;
}
hardware_interface::cancoder::AbsoluteSensorRange CANCoderCIParams::getAbsoluteSensorRange(void) const
{
	return absolute_sensor_range_;
}
double CANCoderCIParams::getConversionFactor(void) const
{
	return conversion_factor_;
}
bool CANCoderCIParams::getEnableReadThread(void) const
{
	return enable_read_thread_;
}

CANCoderControllerInterface::CANCoderControllerInterface(const ros::NodeHandle &n, hardware_interface::cancoder::CANCoderCommandHandle handle)
	: params_(n)
	, handle_(handle)
{
}

void CANCoderControllerInterface::update(void)
{
	handle_->setSensorDirection(params_.getSensorDirection());
	handle_->setMagnetOffset(params_.getMagnetOffset());
	handle_->setAbsoluteSensorRange(params_.getAbsoluteSensorRange());
	handle_->setConversionFactor(params_.getConversionFactor());
	handle_->setEnableReadThread(params_.getEnableReadThread());

	{
		std::unique_lock<std::mutex> l(set_position_mutex_, std::try_to_lock);
		if (l.owns_lock() && set_position_flag_)
		{
			handle_->setSetPosition(set_position_value_);
			set_position_flag_ = false;
		}
	}

	if (clear_sticky_faults_.exchange(false))
	{
		handle_->setClearStickyFaults();
	}
}

void CANCoderControllerInterface::setPosition(const double new_position)
{
	std::lock_guard<std::mutex> l(set_position_mutex_);
	set_position_flag_ = true;
	set_position_value_ = new_position;
}

void CANCoderControllerInterface::setClearStickyFaults(void)
{
	clear_sticky_faults_ = true;
}

void CANCoderControllerInterface::setSensorDirection(const hardware_interface::cancoder::SensorDirection sensor_direction)
{
	params_.setSensorDirection(static_cast<int>(sensor_direction));
}
void CANCoderControllerInterface::setMagnetOffset(const double magnet_offset)
{
	params_.setMagnetOffset(magnet_offset);
}

void CANCoderControllerInterface::setAbsoluteSensorRange(const hardware_interface::cancoder::AbsoluteSensorRange absolute_sensor_range)
{
	params_.setAbsoluteSensorRange(static_cast<int>(absolute_sensor_range));
}
void CANCoderControllerInterface::setConversionFactor(const double conversion_factor)
{
	params_.setConversionFactor(conversion_factor);
}

} // namespace cancoder_controller_interface
