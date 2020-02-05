#include <optional>
#include "ros/console.h"
#include <Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include "teleop_joystick_control/rate_limiter.h"
#include "teleop_joystick_control/TeleopJoystickCompConfig.h"

struct StrafeSpeeds
{
	double x_;
	double y_;
	StrafeSpeeds(const double x, const double y)
		: x_{x}
		, y_{y}
	{
	}
};

template <class ConfigT>
class TeleopCmdVel
{
	public:
		wpi::interpolating_map<double, store_xy> joystick_values;

		TeleopCmdVel(const ConfigT &config):
			x_rate_limit_(-config.max_speed, config.max_speed, config.drive_rate_limit_time),
			y_rate_limit_(-config.max_speed, config.max_speed, config.drive_rate_limit_time),
			rot_rate_limit_(-config.max_rot, config.max_rot, config.rotate_rate_limit_time),
			last_stamp_{ros::Time::now()}
		{
		}

		void setRobotOrient(const bool robot_orient, const double offset_angle)
		{
			robot_orient_ = robot_orient;
			offset_angle_ = offset_angle;
		}

		void setSlowMode(const bool &slow_mode)
		{
			slow_mode_ = slow_mode;
		}

		// Note - updateRiseTimeInMsec() does nothing if the
		// requested time is the same as the current config
		void updateRateLimit(const ConfigT &config)
		{
			x_rate_limit_.updateRiseTimeInMsec(config.drive_rate_limit_time);
			y_rate_limit_.updateRiseTimeInMsec(config.drive_rate_limit_time);
			rot_rate_limit_.updateRiseTimeInMsec(config.rotate_rate_limit_time);
		}

		StrafeSpeeds generateCmdVel(const double translationX, const double translationY, const double navX_angle, const ros::Time &stamp, const ConfigT &config)
		{
			const double max_speed = slow_mode_ ? config.max_speed_slow : config.max_speed;

			x_rate_limit_.updateMinMax(-max_speed, max_speed);
			y_rate_limit_.updateMinMax(-max_speed, max_speed);

			//ROS_INFO_STREAM(__LINE__ << " "  << translationX << " " << translationY);

			// Convert to polar coordinates
			double direction = atan2(translationY, translationX);

			// Do a dead zone check on the magnitude of the velocity,
			// then scale it by a power function to increase resolution
			// of low-speed inputs. This will give an output range from
			// 0-100%
			// Finally, scale it so that 0% corresponds to the minimum
			// output needed to move the robot and 100% to the max
			// configured speed
			double magnitude = dead_zone_check(hypot(leftStickX, leftStickY), config.joystick_deadzone);
			//ROS_INFO_STREAM(__LINE__ << " magnitude:"  << magnitude);
			if (magnitude != 0)
			{
				const double xSpeed = x_rate_limit_.applyLimit(0, stamp);
				const double ySpeed = y_rate_limit_.applyLimit(0, stamp);
				//ROS_INFO(__LINE__ << " Dead zone check returned 0 velocity");
				return StrafeSpeeds{xSpeed, ySpeed};
			}

			magnitude  = pow(magnitude, config.joystick_pow);
			//ROS_INFO_STREAM(__LINE__ << " "  << magnitude << " " << direction);
			magnitude *= max_speed - config.min_speed;
			//ROS_INFO_STREAM(__LINE__ << " "  << magnitude);
			magnitude += config.min_speed;
			//ROS_INFO_STREAM(__LINE__ << " "  << magnitude);

			// This applies a ramp to the output - it limits the amount of change per
			// unit time that's allowed.  This helps limit extreme current draw in
			// cases where the robot is changing direction rapidly.

			// Convert back to rectangular coordinates for the x and Y velocity
			const double xSpeed = x_rate_limit_.applyLimit(magnitude * cos(direction), stamp);
			const double ySpeed = y_rate_limit_.applyLimit(magnitude * sin(direction), stamp);
			//ROS_INFO_STREAM(__LINE__ << " "  << xSpeed << " " << ySpeed);

			//Publish drivetrain messages and call servers
			Eigen::Vector2d joyVector;
			joyVector[0] = -xSpeed; //intentionally flipped
			joyVector[1] = -ySpeed;

			const Eigen::Rotation2Dd rotate(robot_orient_ ? -offset_angle_ : -navX_angle);
			const Eigen::Vector2d rotatedJoyVector = rotate.toRotationMatrix() * joyVector;

			return StrafeSpeeds{rotatedJoyVector[1], rotatedJoyVector[0]};
		}

		double generateAngleIncrement(const double rotationZ, const ros::Time &stamp, ConfigT &config)
		{
			const double max_rot = slow_mode_ ? config.max_rot_slow : config.max_rot;
			//rot_rate_limit_.updateMinMax(-max_rot, max_rot);

			double rotation = dead_zone_check(rotationZ, config.joystick_deadzone);

			// Scale the input by a power function to increase resolution
			// of the slower settings. Use copysign to preserve the sign
			// of the original input (keeps the direction correct)
			rotation  = pow(rotation, config.rotation_pow);
			rotation  = copysign(rotation, rotationZ);
			rotation *= max_rot;
			//rot_rate_limit_.applyLimit(rotation, stamp);

			// TODO - check that timestep isn't too large?
			const ros::Duration timestep = stamp - last_stamp_;
			last_stamp_ = stamp;

			return angles::normalize_angle(rotation*timestep.toSec());
		}

	private:
		bool robot_orient_{false};
		bool slow_mode_{false};

		double offset_angle_{M_PI / 2.0};

		rate_limiter::RateLimiter x_rate_limit_;
		rate_limiter::RateLimiter y_rate_limit_;
		rate_limiter::RateLimiter rot_rate_limit_;

		ros::Time last_stamp_;

		double dead_zone_check(const double test_axis, const double dead_zone)
		{
			// Less than dead_zone? Return 0
			if (fabs(test_axis) <= fabs(dead_zone))
				return 0;

			// Scale the remaining values - just above dead_zone
			// is 0, scaling up to 1.0 as the input value goes
			// to 1.0. This avoids a jerk from 0->0.x as the input
			// transitions from dead_zone to non-dead_zone
			return (fabs(test_axis) - dead_zone) / (1.0 - dead_zone);
		}
};
