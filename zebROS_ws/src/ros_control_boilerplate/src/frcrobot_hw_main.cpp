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
   Desc:   Example ros_control main() entry point for controlling robots in ROS
*/


//PURPOSE: Pulls togther stuff needed to run hw interface

#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <ros_control_boilerplate/frcrobot_hw_interface.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "frcrobot_hw_interface");
	ros::NodeHandle nh;
	ros::NodeHandle robot_nh(nh, "hardware_interface");

	// NOTE: We run the ROS loop in a separate thread as external calls such
	// as service callbacks to load controllers can block the (main) control loop
	ros::AsyncSpinner spinner(2);
	spinner.start();

	// Create the hardware interface specific to your robot
	auto frcrobot_hw_interface = std::make_shared<ros_control_boilerplate::FRCRobotHWInterface>();
	if (!frcrobot_hw_interface->init(nh, robot_nh))
	{
		ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : frcrobot_hw_interface->init() returned false");
		return -1;
	}

	// Start the control loop - false for SIM template param means real hardware
	ros_control_boilerplate::GenericHWControlLoop<false> control_loop(nh, frcrobot_hw_interface);

	control_loop.run(); // Blocks until shutdown signal recieved

	return 0;
}
