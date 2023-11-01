//#include <ros/ros.h>
#include <talon_swerve_drive_controller/900Math.h>
#include <talon_swerve_drive_controller/Swerve.h>

#include <ros/console.h>

template <size_t WHEELCOUNT>
swerve<WHEELCOUNT>::swerve(const std::array<Eigen::Vector2d, WHEELCOUNT> &wheelCoordinates,
						   const std::array<double, WHEELCOUNT> &offsets,
						   const swerveVar::ratios &ratio,
						   const swerveVar::encoderUnits &units,
						   const swerveVar::driveModel &drive)
	: wheelCoordinates_(wheelCoordinates), swerveMath_(swerveDriveMath<WHEELCOUNT>(wheelCoordinates_)), offsets_(offsets), ratio_(ratio), units_(units), drive_(drive)
{
}

template <size_t WHEELCOUNT>
std::array<Eigen::Vector2d, WHEELCOUNT> swerve<WHEELCOUNT>::motorOutputs(Eigen::Vector2d velocityVector,
																		 double rotation,
																		 double angle,
																		 const std::array<double, WHEELCOUNT> &positionsNew,
																		 bool norm,
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
	auto speedsAndAngles = swerveMath_.wheelSpeedsAngles(mult_it->second.multipliers_, velocityVector, rotation, angle, norm);
	for (size_t i = 0; i < WHEELCOUNT; i++)
	{
		//ROS_INFO_STREAM("id: " << i << " PRE NORMalIZE pos/vel in direc: " << speedsAndAngles[i][0] << " rot: " <<speedsAndAngles[i][1] );
		const double currpos = getWheelAngle(i, positionsNew[i]);
		bool reverse;
		const double nearestangle = leastDistantAngleWithinHalfPi(currpos, speedsAndAngles[i][1], reverse);

		//ROS_INFO_STREAM("wheel " << i << " currpos: " << currpos << " nearestangle: " << nearestangle << " reverse: " << reverse);
		// Slow down wheels the further they are from their target
		// angle. This will help to prevent wheels which are in the process
		// of getting to the correct orientation from dragging the robot
		// in random directions while turning to the expected direction
		// cos() shouldn't care about +/-, so don't worry about fabs()
		const double cosScaling = useCosScaling ? cos(currpos - nearestangle) : 1.0;

		speedsAndAngles[i][0] *= ((drive_.maxSpeed / drive_.wheelRadius) / ratio_.encodertoRotations) * units_.rotationSetV * (reverse ? -1 : 1) * cosScaling;
		speedsAndAngles[i][1] = nearestangle * units_.steeringSet + offsets_[i];
		//ROS_INFO_STREAM("pos/vel in direc: " << speedsAndAngles[i][0] << " rot: " << speedsAndAngles[i][1] << " offset: " << offsets_[i] << " steeringSet: " << units_.steeringSet << " reverse: " << reverse);
	}
	return speedsAndAngles;
}

template<size_t WHEELCOUNT>
std::array<double, WHEELCOUNT> swerve<WHEELCOUNT>::parkingAngles(const std::array<double, WHEELCOUNT> &positionsNew) const
{
	std::array<double, WHEELCOUNT> retAngles;
	for (size_t i = 0; i < WHEELCOUNT; i++)
	{
		const double currpos = getWheelAngle(i, positionsNew[i]);
		bool reverse;
		const double nearestanglep = leastDistantAngleWithinHalfPi(currpos, swerveMath_.getParkingAngle(i), reverse);

		retAngles[i] = nearestanglep * units_.steeringSet + offsets_[i];
		//ROS_INFO_STREAM(" id: " << i << " currpos: " << currpos << " target: " << nearestanglep);
		//ROS_INFO_STREAM("park[i]: " << swerveMath_.getParkingAngle(i) << " " << retAngles[i]);
	}
	return retAngles;
}

// Apply encoder offset and steering ratio to calculate desired
// measured wheel angle from a wheel angle setpoint
template<size_t WHEELCOUNT>
double swerve<WHEELCOUNT>::getWheelAngle(int index, double pos) const
{
	return (pos - offsets_[index]) * units_.steeringGet;
}

template<size_t WHEELCOUNT>
double swerve<WHEELCOUNT>::furthestWheel(const Eigen::Vector2d &centerOfRotation) const
{
	double maxD = 0;
	for (size_t i = 0; i < WHEELCOUNT; i++)
		maxD = std::max(maxD, hypot(wheelCoordinates_[i][0] - centerOfRotation[0], wheelCoordinates_[i][1] - centerOfRotation[1]));
	return maxD;
}

template class swerve<4>;
