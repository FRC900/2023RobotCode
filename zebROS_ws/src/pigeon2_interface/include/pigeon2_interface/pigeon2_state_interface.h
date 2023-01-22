#ifndef INC_PIGEON2_STATE_INTERFACE__
#define INC_PIGEON2_STATE_INTERFACE__

#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include "state_handle/state_handle.h"

namespace hardware_interface
{
namespace pigeon2
{

enum class AxisDirection {
	Undefined,
	PositiveZ,
	PositiveY,
	PositiveX,
	NegativeZ,
	NegativeY,
	NegativeX,
	AxisDirection_Last
};
class Pigeon2HWState
{
	public:
		Pigeon2HWState(int device_number);
		int getDeviceNumber(void) const;

		void setMountPoseForward(AxisDirection direction);
		AxisDirection getMountPoseForward(void) const;
		
		void setMountPoseUp(AxisDirection direction);
		AxisDirection getMountPoseUp(void) const;

		void setMountPoseRoll(double direction);
		double getMountPoseRoll(void) const;
		
		void setMountPosePitch(double direction);
		double getMountPosePitch(void) const;

		void setMountPoseYaw(double direction);
		double getMountPoseYaw(void) const;
		
		void setXAxisGyroError(double x_axis_gyro_error);
		double getXAxisGyroError(void) const;
		
		void setYAxisGyroError(double y_axis_gyro_error);
		double getYAxisGyroError(void) const;
		
		void setZAxisGyroError(double z_axis_gyro_error);
		double getZAxisGyroError(void) const;

		void setCompassEnable(bool compass_enable);
		bool getCompassEnable(void) const;

		void setTemperatureCompensation(bool temperature_compensation);
		bool getTemperatureCompensation(void) const;

		void setDisableNoMotionCompensation(bool disable_no_motion_compensation);
		bool getDisableNoMotionCompensation(void) const;

		void setGravityVector(const std::array<double, 3> &gravity_vector);
		std::array<double, 3> getGravityVector(void) const;

		int getFaults(void) const;
		void setFaults(int faults);

		int getStickyFaults(void) const;
		void setStickyFaults(int sticky_faults);

	private :
		int                          device_number_;
		AxisDirection                mount_pose_forward_{AxisDirection::Undefined};
		AxisDirection                mount_pose_up_{AxisDirection::Undefined};
		double                       mount_pose_roll_{0};
		double                       mount_pose_pitch_{0};
		double                       mount_pose_yaw_{0};
		double                       x_axis_gyro_error_{0};
		double                       y_axis_gyro_error_{0};
		double                       z_axis_gyro_error_{0};
		bool                         compass_enable_{false};
		bool                         temperature_compensation_{false};
		bool                         disable_no_motion_compensation_{false};
		std::array<double, 3>        gravity_vector_{0, 0, 0};
		int                          faults_{0};
		int                          sticky_faults_{0};
};
// Glue code to let this be registered in the list of
// hardware resources on the robot.  Since state is
// read-only, allow multiple controllers to register it.
typedef StateHandle<const Pigeon2HWState> Pigeon2StateHandle;
typedef StateHandle<Pigeon2HWState>       Pigeon2WritableStateHandle;
class Pigeon2StateInterface : public HardwareResourceManager<Pigeon2StateHandle> {};
class RemotePigeon2StateInterface : public HardwareResourceManager<Pigeon2WritableStateHandle> {};

} // namespace cancoder
} // namespace hardware_interface

#endif