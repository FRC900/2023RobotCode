#ifndef INC_PIGEON2_COMMAND_INTERFACE__
#define INC_PIGEON2_COMMAND_INTERFACE__

#include "pigeon2_interface/pigeon2_state_interface.h"
#include "state_handle/command_handle.h"

namespace hardware_interface
{
namespace pigeon2
{
class Pigeon2HWCommand
{
	public:
		Pigeon2HWCommand(void);

		void setMountPoseForward(AxisDirection direction);
		AxisDirection getMountPoseForward(void) const;

		void setMountPoseUp(AxisDirection direction);
		AxisDirection getMountPoseUp(void) const;

		bool mountPoseAxisChanged(AxisDirection &forward, AxisDirection &up);
		void resetMountPoseAxis(void);

		void setMountPoseRoll(double direction);
		double getMountPoseRoll(void) const;
		void setMountPosePitch(double direction);
		double getMountPosePitch(void) const;
		void setMountPoseYaw(double direction);
		double getMountPoseYaw(void) const;

		bool mountPoseRPYChanged(double &iroll, double &pitch, double &yaw);
		void resetMountPoseRPY(void);

		void setXAxisGyroError(double x_axis_gyro_error);
		double getXAxisGyroError(void) const;
		bool xAxisGyroErrorChanged(double &x_axis_gyro_error);
		void resetXAxisGyroError(void);
		
		void setYAxisGyroError(double y_axis_gyro_error);
		double getYAxisGyroError(void) const;
		bool yAxisGyroErrorChanged(double &y_axis_gyro_error);
		void resetYAxisGyroError(void);
		
		void setZAxisGyroError(double z_axis_gyro_error);
		double getZAxisGyroError(void) const;
		bool zAxisGyroErrorChanged(double &z_axis_gyro_error);
		void resetZAxisGyroError(void);
		
		void setCompassEnable(bool compass_enable);
		bool getCompassEnable(void) const;
		bool compassEnableChanged(bool &compass_enable);
		void resetCompassEnableChanged(void);

		void setTemperatureCompensation(bool temperature_compensation);
		bool getTemperatureCompensation(void) const;
		bool temperatureCompensationChanged(bool &temperature_compensation);
		void resetTemperatureCompensationChanged(void);

		void setDisableNoMotionCompensation(bool disable_no_motion_compensation);
		bool getDisableNoMotionCompensation(void) const;
		bool disableNoMotionCompensationChanged(bool &disable_no_motion_compensation);
		void resetDisableNoMotionCompensationChanged(void);

		void setZeroGyroBiasNow(void);
		bool getZeroGyroBiasNow(void) const;
		bool zeroGyroBiasNowChanged(void);

		void setClearStickyFaults(void);
		bool getClearStickyFaults(void) const;
		bool clearStickyFaultsChanged(void);

	private:
		AxisDirection                mount_pose_forward_{AxisDirection::Undefined};
		AxisDirection                mount_pose_up_{AxisDirection::Undefined};
		bool                         mount_pose_axis_changed_{true};
		double                       mount_pose_roll_{0};
		double                       mount_pose_pitch_{0};
		double                       mount_pose_yaw_{0};
		bool                         mount_pose_rpy_changed_{true};
		double                       x_axis_gyro_error_{0};
		bool                         x_axis_gyro_error_changed_{true};
		double                       y_axis_gyro_error_{0};
		bool                         y_axis_gyro_error_changed_{true};
		double                       z_axis_gyro_error_{0};
		bool                         z_axis_gyro_error_changed_{true};
		bool                         compass_enable_{false};
		bool                         compass_enable_changed_{true};
		bool                         temperature_compensation_{false};
		bool                         temperature_compensation_changed_{true};
		bool                         disable_no_motion_compensation_{false};
		bool                         disable_no_motion_compensation_changed_{true};

        bool                         zero_gyro_bias_now_;
		bool                         clear_sticky_faults_;
};

// Handle - used by each controller to get, by name of the
// corresponding joint, an interface with which to send commands
// to a Pigeon2
typedef CommandHandle<Pigeon2HWCommand, Pigeon2HWState, Pigeon2StateHandle> Pigeon2CommandHandle;


// Use ClaimResources here since we only want 1 controller
// to be able to access a given Pigeon2 at any particular time
class Pigeon2CommandInterface : public HardwareResourceManager<Pigeon2CommandHandle, ClaimResources> {};

} // namespace pigeon2
} // namespace hardware_interface

#endif