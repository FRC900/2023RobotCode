// TODO - consider tying pid enable pub to robot enabled?
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

#include "teleop_joystick_control/RobotOrientationDriver.h"

RobotOrientationDriver::RobotOrientationDriver(const ros::NodeHandle &nh)
	: nh_(nh)
	, orientation_command_sub_{nh_.subscribe("orientation_command", 2, &RobotOrientationDriver::orientationCmdCallback, this)}
	, pid_enable_pub_{nh_.advertise<std_msgs::Bool>("orient_strafing/pid_enable", 1)}
	, pid_state_pub_{nh_.advertise<std_msgs::Float64>("orient_strafing/state", 1)}
	, pid_setpoint_pub_{nh_.advertise<std_msgs::Float64>("orient_strafing/setpoint", 1)}
	, pid_control_effort_sub_{nh_.subscribe("orient_strafing/control_effort", 1, &RobotOrientationDriver::controlEffortCallback, this)}
{
}

void RobotOrientationDriver::setTargetOrientation(double angle, bool from_teleop)
{
	if (robot_enabled_)
	{
		target_orientation_ = angle;
	}
	else
	{
		// If the robot is disabled, set the desired orientation to the
		// current orientation to prevent the robot from snapping to a
		// random angle when reenabled
		target_orientation_ = robot_orientation_;
	}
	most_recent_is_teleop_ = from_teleop;
	// Publish desired robot orientation to the PID node
	std_msgs::Float64 pid_setpoint_msg;
	pid_setpoint_msg.data = target_orientation_;
	pid_setpoint_pub_.publish(pid_setpoint_msg);

	//ROS_INFO_STREAM(__FUNCTION__ << "pub setpoint = " << pid_setpoint_msg.data );
	// Make sure the PID node is enabled
	std_msgs::Bool enable_pub_msg;
	enable_pub_msg.data = true;
	pid_enable_pub_.publish(enable_pub_msg);
	//ROS_INFO_STREAM(__FUNCTION__ << "pub enable = " << (int)enable_pub_msg.data );
}

void RobotOrientationDriver::incrementTargetOrientation(double deltaAngle)
{
	//ROS_INFO_STREAM(__FUNCTION__ << " deltaAngle = " << deltaAngle);
	setTargetOrientation(target_orientation_ + deltaAngle);
}

// Set the desired orientation to the current IMU orientation
// to stop the robot at the current orientation
void RobotOrientationDriver::stopRotation(void)
{
	setTargetOrientation(robot_orientation_);
}

void RobotOrientationDriver::orientationCmdCallback(const std_msgs::Float64::ConstPtr &orient_msg)
{
	//ROS_INFO_STREAM(__FUNCTION__ << " angle = " << orient_msg->data);
	setTargetOrientation(orient_msg->data, false);
}

void RobotOrientationDriver::controlEffortCallback(const std_msgs::Float64::ConstPtr &control_effort)
{
	// Store a local copy of the output of the PID node. This
	// will be the z-orientation velocity from the PID controller
	orientation_command_effort_ = control_effort->data;
}

void RobotOrientationDriver::setRobotOrientation(double angle)
{
	//ROS_INFO_STREAM(__FUNCTION__ << " angle = " << angle);
	robot_orientation_ = angle;
	// If the robot is disabled, set the desired orientation to the
	// current orientation to prevent the robot from snapping to a
	// random angle when reenabled
	if (!robot_enabled_)
	{
		setTargetOrientation(robot_orientation_, false);
	}
	// Update PID plant state using this most recent
	// robot orientation value
	std_msgs::Float64 pid_state_msg;
	pid_state_msg.data = robot_orientation_;
	pid_state_pub_.publish(pid_state_msg);
}

void RobotOrientationDriver::setRobotEnabled(bool enabled)
{
	//ROS_INFO_STREAM(__FUNCTION__ << " robot_enabled = " << enabled);
	robot_enabled_ = enabled;
	// If the robot is disabled, set the desired orientation to the
	// current orientation to prevent the robot from snapping to a
	// random angle when reenabled
	if (!robot_enabled_)
	{
		setTargetOrientation(robot_orientation_, false);
	}
}

double RobotOrientationDriver::getTargetOrientation(void) const
{
	return target_orientation_;
}

double RobotOrientationDriver::getOrientationVelocityPIDOutput(void) const
{
	return orientation_command_effort_;
}

bool RobotOrientationDriver::mostRecentCommandIsFromTeleop(void) const
{
	return most_recent_is_teleop_;
}

