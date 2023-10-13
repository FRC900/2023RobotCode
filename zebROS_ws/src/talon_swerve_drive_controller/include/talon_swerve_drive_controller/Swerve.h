#pragma once

#include <array>
#include <cmath>
#include <functional>
#include <string>
#include <map>
#include <vector>
#include <Eigen/Dense>
#include "SwerveMath.h"

//meters, radians, newtons, kg
//This class will need access to the swerve drive talons
//Gets should be rotations/encoder unit
//Sets should be encoder unit/rotation (or rotations/second)

// Create an ordering function for Vectors to use them in a map
namespace std
{
template<>
struct less<Eigen::Vector2d>
{
	bool operator()(Eigen::Vector2d const& a, Eigen::Vector2d const& b) const
	{
		assert(a.size()==b.size());
		for(int i = 0; i < a.size(); ++i)
		{
			if (a[i] < b[i]) return true;
			if (a[i] > b[i]) return false;
		}
		return false;
	}
};
}

namespace swerveVar
{
struct ratios
{
	double encodertoRotations;
	//double motortoRotations;
	//double motortoSteering;
};

struct encoderUnits
{
	double steeringGet;
	double steeringSet;
	double rotationGetV;
	double rotationGetP;
	double rotationSetV;
	double rotationSetP;
};
struct driveModel
{
	double maxSpeed;
	double wheelRadius;
	//double mass;
	//double motorFreeSpeed;
	//double motorStallTorque;
	//int motorQuantity;
	//double speedLossConstant = .81; // Don't set this here
}; //more info should be added to this struct
}

template <size_t WHEELCOUNT>
class swerve
{
	public:
		swerve(const std::array<Eigen::Vector2d, WHEELCOUNT> &wheelCoordinates,
			   const std::array<double, WHEELCOUNT> &offsets,
			   const swerveVar::ratios &ratio,
			   const swerveVar::encoderUnits &units,
			   const swerveVar::driveModel &drive);

		//for non field centric drive set angle = pi/2
		std::array<Eigen::Vector2d, WHEELCOUNT> motorOutputs(Eigen::Vector2d velocityVector,
														     double rotation,
														     double angle,
														     const std::array<double, WHEELCOUNT> &positionsNew,
														     bool norm,
														     const Eigen::Vector2d &centerOfRotation = Eigen::Vector2d{0,0},
															 const bool useCosScaling = false);
		std::array<double, WHEELCOUNT> parkingAngles(const std::array<double, WHEELCOUNT> &positionsNew) const;

		double getWheelAngle(int index, double pos) const;
	private:
		std::array<Eigen::Vector2d, WHEELCOUNT> wheelCoordinates_;
		swerveDriveMath<WHEELCOUNT> swerveMath_; //this should be public
		//should we get them together instead?
		//the angle it passes out isn't normalized
		double furthestWheel(const Eigen::Vector2d &centerOfRotation) const;

		std::array<double, WHEELCOUNT> offsets_;

		//Second piece of data is here just for physics/modeling

		//std::array<double, WHEELCOUNT> savedEncoderVals_;
		//int8_t wheelAngleInvert_;

		struct multiplierSet
		{
			std::array<Eigen::Vector2d, WHEELCOUNT> multipliers_;
			double maxRotRate_;
		};
		std::map<Eigen::Vector2d, multiplierSet> multiplierSets_;
		swerveVar::ratios ratio_;
		swerveVar::encoderUnits units_;
		swerveVar::driveModel drive_;
};
