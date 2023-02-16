/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Original Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the FRCRobot
           For a more detailed simulation example, see sim_hw_interface.h
*/

#ifndef INC_FRCROBOT_SIM_INTERFACE
#define INC_FRCROBOT_SIM_INTERFACE

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include "ctre/phoenix/motorcontrol/can/TalonFX.h"
#include "ctre/phoenix/motorcontrol/can/TalonSRX.h"
#include "ctre/phoenix/motorcontrol/can/VictorSPX.h"

#include "frc_msgs/MatchSpecificData.h"

//#include "frc/simulation/FlywheelSim.h"

#include "ros_control_boilerplate/frc_robot_interface.h"
#include "ros_control_boilerplate/LineBreakSensors.h"
#include "ros_control_boilerplate/set_limit_switch.h"

namespace ros_control_boilerplate
{
/// \brief Hardware interface for a robot
class FRCRobotSimInterface : public ros_control_boilerplate::FRCRobotInterface
{
	public:
		/**
		 * \brief Constructor
		 * \param nh - Node handle for topics.
		 */
		FRCRobotSimInterface(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);
		~FRCRobotSimInterface();

		virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh) override;

		/** \brief Read the state from the robot hardware. */
		virtual void read(const ros::Time& time, const ros::Duration& period) override;

		/** \brief Write the command to the robot hardware. */
		virtual void write(const ros::Time& time, const ros::Duration& period) override;

		virtual bool setlimit(ros_control_boilerplate::set_limit_switch::Request &req,ros_control_boilerplate::set_limit_switch::Response &res);

	private:
        ros::Subscriber match_data_sub_;
		std::vector<ros::Subscriber> joystick_subs_;
        void match_data_callback(const frc_msgs::MatchSpecificData &match_data);
		void joystickCallback(const sensor_msgs::JoyConstPtr &msg, int32_t joystick_num);
		void imuOdomCallback(const nav_msgs::OdometryConstPtr &msg, int32_t imu_num);
		bool evaluateDigitalInput(ros_control_boilerplate::LineBreakSensors::Request &req, ros_control_boilerplate::LineBreakSensors::Response &res);

		void setSimCollection(
				std::shared_ptr<ctre::phoenix::motorcontrol::can::TalonSRX> talon_srx,
				std::shared_ptr<ctre::phoenix::motorcontrol::can::TalonFX> talon_fx,
				double position, double velocity, double delta_position = 0) const;
		void setSimCollectionTalonSRX(std::shared_ptr<ctre::phoenix::motorcontrol::can::TalonSRX> talon_srx,
				double position, double velocity, double delta_position) const;
		void setSimCollectionTalonFX(std::shared_ptr<ctre::phoenix::motorcontrol::can::TalonFX> talon_fx,
				double position, double velocity, double delta_position) const;


		ros::ServiceServer linebreak_sensor_srv_;
		ros::ServiceServer limit_switch_srv_;

		//std::unique_ptr<frc::sim::FlywheelSim> shooter_sim_;
		size_t shooter_sim_joint_index_;

		std::vector<std::shared_ptr<std::atomic<double>>> imu_sim_yaws_;
		std::vector<ros::Subscriber> imu_sim_subs_;
};  // class

}  // namespace

#endif