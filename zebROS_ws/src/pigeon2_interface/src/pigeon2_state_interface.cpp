#include "pigeon2_interface/pigeon2_state_interface.h"

namespace hardware_interface
{
namespace pigeon2
{

Pigeon2HWState::Pigeon2HWState(int device_number)
	: device_number_{device_number}
{
}

int Pigeon2HWState::getDeviceNumber(void) const
{
	return device_number_;
}

void Pigeon2HWState::setMountPoseForward(AxisDirection direction)
{
	mount_pose_forward_ = direction;
}
AxisDirection Pigeon2HWState::getMountPoseForward(void) const
{
	return mount_pose_forward_;
}

void Pigeon2HWState::setMountPoseUp(AxisDirection direction)
{
	mount_pose_up_ = direction;
}
AxisDirection Pigeon2HWState::getMountPoseUp(void) const
{
	return mount_pose_up_;
}

void Pigeon2HWState::setMountPoseRoll(double direction)
{
		mount_pose_roll_ = direction;
}
double Pigeon2HWState::getMountPoseRoll(void) const
{
	return mount_pose_roll_;
}

void Pigeon2HWState::setMountPosePitch(double direction)
{
	mount_pose_pitch_ = direction;
}
double Pigeon2HWState::getMountPosePitch(void) const
{
	return mount_pose_pitch_;
}

void Pigeon2HWState::setMountPoseYaw(double direction)
{
		mount_pose_yaw_ = direction;
}
double Pigeon2HWState::getMountPoseYaw(void) const
{
	return mount_pose_yaw_;
}

void Pigeon2HWState::setXAxisGyroError(double x_axis_gyro_error)
{
	x_axis_gyro_error_ = x_axis_gyro_error;
}
double Pigeon2HWState::getXAxisGyroError(void) const
{
	return x_axis_gyro_error_;
}

void Pigeon2HWState::setYAxisGyroError(double y_axis_gyro_error)
{
	y_axis_gyro_error_ = y_axis_gyro_error;
}
double Pigeon2HWState::getYAxisGyroError(void) const
{
	return y_axis_gyro_error_;
}

void Pigeon2HWState::setZAxisGyroError(double z_axis_gyro_error)
{
	z_axis_gyro_error_ = z_axis_gyro_error;
}
double Pigeon2HWState::getZAxisGyroError(void) const
{
	return z_axis_gyro_error_;
}

void Pigeon2HWState::setCompassEnable(bool compass_enable)
{
	compass_enable_ = compass_enable;
}
bool Pigeon2HWState::getCompassEnable(void) const
{
	return compass_enable_;
}

void Pigeon2HWState::setTemperatureCompensation(bool temperature_compensation)
{
	temperature_compensation_ = temperature_compensation;
}
bool Pigeon2HWState::getTemperatureCompensation(void) const
{
	return temperature_compensation_;
}

void Pigeon2HWState::setDisableNoMotionCompensation(bool disable_no_motions_compensation)
{
	disable_no_motion_compensation_ = disable_no_motions_compensation;
}
bool Pigeon2HWState::getDisableNoMotionCompensation(void) const
{
	return disable_no_motion_compensation_;
}

void Pigeon2HWState::setGravityVector(const std::array<double, 3> &gravity_vector)
{
	gravity_vector_ = gravity_vector;
}
std::array<double, 3> Pigeon2HWState::getGravityVector(void) const
{
	return gravity_vector_;
}

void Pigeon2HWState::setFaults(int faults)
{
	faults_ = faults;
}
int Pigeon2HWState::getFaults(void) const
{
	return faults_;
}


void Pigeon2HWState::setStickyFaults(int sticky_faults)
{
	sticky_faults_ = sticky_faults;
}
int Pigeon2HWState::getStickyFaults(void) const
{
	return sticky_faults_;
}

} //namespace hardware_interface
} // namespace cancoder
