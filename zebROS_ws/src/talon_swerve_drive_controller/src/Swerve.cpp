#include <angles/angles.h>
#include <talon_swerve_drive_controller/Swerve.h>

#include <ros/console.h>

struct WheelDirection {
    WheelDirection(const double angle, const double speed_multiplier) : angle(angle), speed_multiplier(speed_multiplier) {}
    double angle;
    double speed_multiplier;
};
static WheelDirection leastDistantAngleWithinHalfPi(const double currentAngle, const double targetAngle)
{
	//returns the closest angle to the current angle = to x*.5*M_PI + target angle where x is any integer
	//used for turning wheels to the target angle
	// const double normalizedDiff = angles::normalize_angle(targetAngle) - angles::normalize_angle(currentAngle);

	// const double withinPi = (fabs(normalizedDiff) < M_PI) ? normalizedDiff : (normalizedDiff - copysign(2. * M_PI, normalizedDiff));

	ROS_INFO_STREAM("currentAngle: " << currentAngle << " targetAngle: " << targetAngle);
	const double withinPi = angles::shortest_angular_distance(currentAngle, targetAngle);
	ROS_INFO_STREAM("withinPi: " << withinPi);

	// If within +/- 90 degress, turn to that angle and move in the requested direction
	if (fabs(withinPi) < (M_PI / 2.))
	{
		return WheelDirection(withinPi + currentAngle, 1.0);
	}
	// If outside +/- 90 degress, turn to the opposite angle and move in the opposite direction
	// to minimize the turning needed to move in the requested direction
	return WheelDirection(withinPi - copysign(M_PI, withinPi) + currentAngle, -1.0);
}

template <size_t WHEELCOUNT>
swerve<WHEELCOUNT>::swerve(const std::array<Eigen::Vector2d, WHEELCOUNT> &wheelCoordinates,
						   const swerveVar::ratios &ratio,
						   const swerveVar::encoderUnits &units,
						   const swerveVar::driveModel &drive)
	: wheelCoordinates_(wheelCoordinates), swerveMath_(swerveDriveMath<WHEELCOUNT>(wheelCoordinates_)), ratio_(ratio), units_(units), drive_(drive)
{
}

template <size_t WHEELCOUNT>
std::array<Eigen::Vector2d, WHEELCOUNT> swerve<WHEELCOUNT>::motorOutputs(Eigen::Vector2d velocityVector,
																		 double rotation,
																		 const std::array<double, WHEELCOUNT> &positionsNew,
																		 const bool norm,
																		 const Eigen::Vector2d &centerOfRotation,
																		 const bool useCosScaling)
{
	// See if the current centerOfRotation coords have been used before
	// If not, calculate the multiplers and matRotRate for them
	// If so, just reuse previously saved values
	auto mult_it = multiplierSets_.find(centerOfRotation);
	if (mult_it == multiplierSets_.end())
	{
		multiplierSet newSet;
		newSet.multipliers_ = swerveMath_.wheelMultipliersXY(centerOfRotation);
		newSet.maxRotRate_ = drive_.maxSpeed / furthestWheel(centerOfRotation);
		multiplierSets_[centerOfRotation] = newSet;
		mult_it = multiplierSets_.find(centerOfRotation);
		//ROS_INFO_STREAM("Added new swerve center of rotation: " << centerOfRotation[0] << "," << centerOfRotation[1]);
	}

	velocityVector /= drive_.maxSpeed;
	rotation       /= mult_it->second.maxRotRate_;

	//ROS_WARN_STREAM("max rate r/s: " <<  multiplierSets_[rotationCenterID].maxRotRate_);
	//ROS_INFO_STREAM("vel: " << velocityVector[0] << " " << velocityVector[1] << " rot: " << rotation);
	auto speedsAndAngles = swerveMath_.wheelSpeedsAngles(mult_it->second.multipliers_, velocityVector, rotation, norm);
	for (size_t i = 0; i < WHEELCOUNT; i++)
	{
		//ROS_INFO_STREAM("id: " << i << " PRE NORMalIZE pos/vel in direc: " << speedsAndAngles[i][0] << " rot: " <<speedsAndAngles[i][1] );
		const double currpos = getWheelAngle(positionsNew[i]);
		const auto wheel_direction = leastDistantAngleWithinHalfPi(currpos, speedsAndAngles[i][1]);


		//ROS_INFO_STREAM("wheel " << i << " currpos: " << currpos << " wheel_direction : " << wheel_direction.angle << " " << wheel_direction.speed_multiplier");
		// Slow down wheels the further they are from their target
		// angle. This will help to prevent wheels which are in the process
		// of getting to the correct orientation from dragging the robot
		// in random directions while turning to the expected direction
		// cos() shouldn't care about +/-, so don't worry about fabs()
		const double cosScaling = useCosScaling ? cos(currpos - wheel_direction.angle) : 1.0;

		speedsAndAngles[i][0] *= ((drive_.maxSpeed / drive_.wheelRadius) / ratio_.encodertoRotations) * units_.rotationSetV * wheel_direction.speed_multiplier * cosScaling;
		speedsAndAngles[i][1] = wheel_direction.angle * units_.steeringSet - M_PI;
		//ROS_INFO_STREAM("pos/vel in direc: " << speedsAndAngles[i][0] << " rot: " << speedsAndAngles[i][1] << " steeringSet: " << units_.steeringSet);
	}
	return speedsAndAngles;
}

template<size_t WHEELCOUNT>
std::array<double, WHEELCOUNT> swerve<WHEELCOUNT>::parkingAngles(const std::array<double, WHEELCOUNT> &positionsNew) const
{
	std::array<double, WHEELCOUNT> retAngles;
	for (size_t i = 0; i < WHEELCOUNT; i++)
	{
		const double currpos = getWheelAngle(positionsNew[i]);
		const auto wheel_direction = leastDistantAngleWithinHalfPi(currpos, swerveMath_.getParkingAngle(i));

		retAngles[i] = wheel_direction.angle * units_.steeringSet - M_PI;
		//ROS_INFO_STREAM(" id: " << i << " currpos: " << currpos << " target: " << nearestanglep);
		//ROS_INFO_STREAM("park[i]: " << swerveMath_.getParkingAngle(i) << " " << retAngles[i]);
	}
	return retAngles;
}

// Apply encoder offset and steering ratio to calculate desired
// measured wheel angle from a wheel angle setpoint
template<size_t WHEELCOUNT>
double swerve<WHEELCOUNT>::getWheelAngle(double pos) const
{
	return (pos - M_PI) * units_.steeringGet;
}

template<size_t WHEELCOUNT>
double swerve<WHEELCOUNT>::furthestWheel(const Eigen::Vector2d &centerOfRotation) const
{
	double maxD = 0;
	for (size_t i = 0; i < WHEELCOUNT; i++)
	{
		maxD = std::max(maxD, hypot(wheelCoordinates_[i][0] - centerOfRotation[0], wheelCoordinates_[i][1] - centerOfRotation[1]));
	}
	return maxD;
}

template class swerve<4>;
