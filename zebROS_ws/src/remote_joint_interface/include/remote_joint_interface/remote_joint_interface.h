#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_mode_interface.h>
#define private protected
#include <hardware_interface/imu_sensor_interface.h>
#undef private

// Create a separate type of joint interface for joints which
// are read from hardware on another controller_manager.  This
// is just an easy way for a controller to get a full list of
// joints which aren't local.
namespace hardware_interface
{
	class RemoteJointInterface : public JointCommandInterface {};
	class RemoteJointModeInterface : public JointModeInterface {};
	class ImuWritableSensorHandle: public ImuSensorHandle
	{
		public:
		ImuWritableSensorHandle() : ImuSensorHandle() {}
		explicit ImuWritableSensorHandle(const Data &data) : ImuSensorHandle(data) {}
		void setFrameId(const std::string &frame_id)
		{
			frame_id_ = frame_id;
		}
		void setOrientation(const double *o)
		{
			double *dst = (double *)orientation_;
			std::copy(o, o + 4, dst);
		}
		void setOrientationCovariance(const double *oc)
		{
			double *dst = (double *)orientation_covariance_;
			std::copy(oc, oc+ 9, dst);
		}
		void setAngularVelocity(const double *av)
		{
			double *dst = (double *)angular_velocity_;
			std::copy(av, av + 3, dst);
		}
		void setAngularVelocityCovariance(const double *avc)
		{
			double *dst = (double *)angular_velocity_covariance_;
			std::copy(avc, avc + 9, dst);
		}
		void setLinearAcceleration(const double *la)
		{
			double *dst = (double *)linear_acceleration_;
			std::copy(la, la + 3, dst);
		}
		void setLinearAccelerationCovariance(const double *lac)
		{
			double *dst = (double *)linear_acceleration_covariance_;
			std::copy(lac, lac + 9, dst);
		}
	};

	class RemoteImuSensorInterface  : public HardwareResourceManager<ImuWritableSensorHandle,  ClaimResources> {};
}
