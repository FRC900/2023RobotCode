#include "pigeon2_interface/pigeon2_command_interface.h"

namespace hardware_interface
{
namespace pigeon2
{

Pigeon2HWCommand::Pigeon2HWCommand(void)
{
}

void Pigeon2HWCommand::setMountPoseForward(AxisDirection direction)
{
	if (mount_pose_forward_ != direction)
	{
		mount_pose_forward_ = direction;
		mount_pose_axis_changed_ = true;
	}
}
AxisDirection Pigeon2HWCommand::getMountPoseForward(void) const
{
	return mount_pose_forward_;
}

void Pigeon2HWCommand::setMountPoseUp(AxisDirection direction)
{
	if (mount_pose_up_ != direction)
	{
		mount_pose_up_ = direction;
		mount_pose_axis_changed_ = true;
	}
}
AxisDirection Pigeon2HWCommand::getMountPoseUp(void) const
{
	return mount_pose_up_;
}

bool Pigeon2HWCommand::mountPoseAxisChanged(AxisDirection &forward, AxisDirection &up)
{
	forward = mount_pose_forward_;
	up = mount_pose_up_;
	auto ret = mount_pose_axis_changed_;
	mount_pose_axis_changed_ = false;
	return true;
}

void Pigeon2HWCommand::resetMountPoseAxis(void)
{
	mount_pose_axis_changed_ = true;
}

void Pigeon2HWCommand::setMountPoseRoll(double direction)
{
	if (mount_pose_roll_ != direction)
	{
		mount_pose_roll_ = direction;
		mount_pose_axis_changed_ = true;
	}
}
double Pigeon2HWCommand::getMountPoseRoll(void) const
{
	return mount_pose_roll_;
}

void Pigeon2HWCommand::setMountPosePitch(double direction)
{
	if (mount_pose_pitch_ != direction)
	{
		mount_pose_pitch_ = direction;
		mount_pose_axis_changed_ = true;
	}
}
double Pigeon2HWCommand::getMountPosePitch(void) const
{
	return mount_pose_pitch_;
}

void Pigeon2HWCommand::setMountPoseYaw(double direction)
{
	if (mount_pose_yaw_ != direction)
	{
		mount_pose_yaw_ = direction;
		mount_pose_axis_changed_ = true;
	}
}
double Pigeon2HWCommand::getMountPoseYaw(void) const
{
	return mount_pose_yaw_;

}

bool Pigeon2HWCommand::mountPoseRPYChanged(double &roll, double &pitch, double &yaw)
{
	roll = mount_pose_roll_;
	pitch = mount_pose_pitch_;
	yaw = mount_pose_yaw_;
	auto ret = mount_pose_rpy_changed_;
	mount_pose_rpy_changed_ = false;
	return true;
}

void Pigeon2HWCommand::resetMountPoseRPY(void)
{
	mount_pose_rpy_changed_ = true;
}

void Pigeon2HWCommand::setXAxisGyroError(double x_axis_gyro_error)
{
    if (x_axis_gyro_error_ != x_axis_gyro_error)
	{
	x_axis_gyro_error_ = x_axis_gyro_error;
	x_axis_gyro_error_changed_ = false;
	}
}
double Pigeon2HWCommand::getXAxisGyroError(void) const
{
	return x_axis_gyro_error_;
}
bool Pigeon2HWCommand::xAxisGyroErrorChanged(double &x_axis_gyro_error)
{
	x_axis_gyro_error = x_axis_gyro_error_;
	auto rc = x_axis_gyro_error_changed_;
	x_axis_gyro_error_changed_ = false;
	return rc;
}
void Pigeon2HWCommand::resetXAxisGyroError(void)
{
	x_axis_gyro_error_changed_ = true;
}

void Pigeon2HWCommand::setYAxisGyroError(double y_axis_gyro_error)
{
    if (y_axis_gyro_error_ != y_axis_gyro_error)
	{
	y_axis_gyro_error_ = y_axis_gyro_error;
	y_axis_gyro_error_changed_ = false;
	}
}
double Pigeon2HWCommand::getYAxisGyroError(void) const
{
	return y_axis_gyro_error_;
}
bool Pigeon2HWCommand::yAxisGyroErrorChanged(double &y_axis_gyro_error)
{
	y_axis_gyro_error = y_axis_gyro_error_;
	auto rc = y_axis_gyro_error_changed_;
	y_axis_gyro_error_changed_ = false;
	return rc;
}
void Pigeon2HWCommand::resetYAxisGyroError(void)
{
	y_axis_gyro_error_changed_ = true;
}

void Pigeon2HWCommand::setZAxisGyroError(double z_axis_gyro_error)
{
    if (z_axis_gyro_error_ != z_axis_gyro_error)
	{
	z_axis_gyro_error_ = z_axis_gyro_error;
	z_axis_gyro_error_changed_ = false;
	}
}
double Pigeon2HWCommand::getZAxisGyroError(void) const
{
	return z_axis_gyro_error_;
}
bool Pigeon2HWCommand::zAxisGyroErrorChanged(double &z_axis_gyro_error)
{
	z_axis_gyro_error = z_axis_gyro_error_;
	auto rc = z_axis_gyro_error_changed_;
	z_axis_gyro_error_changed_ = false;
	return rc;
}
void Pigeon2HWCommand::resetZAxisGyroError(void)
{
	z_axis_gyro_error_changed_ = true;
}


void Pigeon2HWCommand::setCompassEnable(bool compass_enable)
{
    if (compass_enable_ != compass_enable)
	{
	compass_enable_ = compass_enable;
	compass_enable_changed_ = true;
	}
}
bool Pigeon2HWCommand::getCompassEnable(void) const
{
	return compass_enable_;
}
bool Pigeon2HWCommand::compassEnableChanged(bool &compass_enable)
{
	compass_enable = compass_enable_;
	auto ret = compass_enable_changed_;
	compass_enable_changed_ = false;
	return ret;
}
void Pigeon2HWCommand::resetCompassEnableChanged(void)
{
	compass_enable_changed_ = true;
}


void Pigeon2HWCommand::setTemperatureCompensation(bool temperature_compensation)
{
    if (temperature_compensation_ != temperature_compensation)
	{
	temperature_compensation_ = temperature_compensation;
	temperature_compensation_changed_ = true;
	}
}
bool Pigeon2HWCommand::getTemperatureCompensation(void) const
{
	return temperature_compensation_;
}
bool Pigeon2HWCommand::temperatureCompensationChanged(bool &temperature_compensation)
{
	temperature_compensation = temperature_compensation_;
	auto ret = temperature_compensation_changed_;
	temperature_compensation_changed_ = false;
	return ret;
}
void Pigeon2HWCommand::resetTemperatureCompensationChanged(void)
{
	temperature_compensation_changed_ = true;
}

void Pigeon2HWCommand::setDisableNoMotionCompensation(bool disable_no_motions_compensation)
{
    if (disable_no_motion_compensation_ != disable_no_motions_compensation)
	{
	disable_no_motion_compensation_ = disable_no_motions_compensation;
	disable_no_motion_compensation_changed_ = true;
	}
}
bool Pigeon2HWCommand::getDisableNoMotionCompensation(void) const
{
	return disable_no_motion_compensation_;
}
bool Pigeon2HWCommand::disableNoMotionCompensationChanged(bool &disable_no_motion_compensation)
{
	disable_no_motion_compensation = disable_no_motion_compensation_;
	auto ret = disable_no_motion_compensation_changed_;
	disable_no_motion_compensation_changed_ = false;
	return ret;
}
void Pigeon2HWCommand::resetDisableNoMotionCompensationChanged(void)
{
	disable_no_motion_compensation_changed_ = true;
}

void Pigeon2HWCommand::setClearStickyFaults(void)
{
	clear_sticky_faults_ = true;
}
bool Pigeon2HWCommand::getClearStickyFaults(void) const
{
	return clear_sticky_faults_;
}
bool Pigeon2HWCommand::clearStickyFaultsChanged(void)
{
	auto ret = clear_sticky_faults_;
	clear_sticky_faults_ = false;
	return ret;
}

void Pigeon2HWCommand::setZeroGyroBiasNow(void)
{
	zero_gyro_bias_now_ = true;
}
bool Pigeon2HWCommand::getZeroGyroBiasNow(void) const
{
	return zero_gyro_bias_now_;
}
bool Pigeon2HWCommand::zeroGyroBiasNowChanged(void)
{
	auto ret = zero_gyro_bias_now_;
	zero_gyro_bias_now_ = false;
	return ret;
}


void Pigeon2HWCommand::setSetYaw(double yaw)
{
	if (yaw != set_yaw_)
	{
		set_yaw_ = yaw;
		set_yaw_changed_ = true;
	}
}
double Pigeon2HWCommand::getSetYaw(void) const
{
	return set_yaw_;
}
bool Pigeon2HWCommand::setYawChanged(double &yaw)
{
	yaw = set_yaw_;
	auto ret = set_yaw_changed_;
	set_yaw_changed_ = false;
	return ret;
}
void Pigeon2HWCommand::resetSetYaw(void)
{
	set_yaw_changed_ = true;
}

void Pigeon2HWCommand::setAddYaw(double yaw)
{
	if (yaw != add_yaw_)
	{
		add_yaw_ = yaw;
		add_yaw_changed_ = true;
	}
}
double Pigeon2HWCommand::getAddYaw(void) const
{
	return add_yaw_;
}
bool Pigeon2HWCommand::addYawChanged(double &yaw)
{
	yaw = add_yaw_;
	bool rc = add_yaw_changed_;
	add_yaw_changed_ = false;
	return rc;
}
void Pigeon2HWCommand::resetAddYaw(void)
{
	add_yaw_changed_ = true;
}

void Pigeon2HWCommand::setSetYawToCompass(void)
{
	set_yaw_to_compass_ = true;

}
bool Pigeon2HWCommand::getSetYawToCompass(void) const
{
	return set_yaw_to_compass_;
}
bool Pigeon2HWCommand::setYawToCompassChanged(void)
{
	auto rc = set_yaw_to_compass_;
	set_yaw_to_compass_ = false;
}

void Pigeon2HWCommand::setSetAccumZAngle(void)
{
	set_accum_z_angle_ = true;
}
bool Pigeon2HWCommand::getSetAccumZAngle(void) const
{
	return set_accum_z_angle_;
}
bool Pigeon2HWCommand::setAccumZAngleChanged(void)
{
	auto ret = set_accum_z_angle_;
	set_accum_z_angle_ = false;
	return ret;
}

} // namespace hardware_interface
} // namespace cancoder