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
   Desc:   Example control loop for reading, updating, and writing commands to a hardware interface
   using MONOTOIC system time
*/

#include <ros/ros.h>

#include "ros_control_boilerplate/generic_hw_control_loop.h"
#include "ros_control_boilerplate/frc_robot_interface.h"

// ROS parameter loading
#include "rosparam_shortcuts/rosparam_shortcuts.h"

namespace ros_control_boilerplate
{
GenericHWControlLoop::GenericHWControlLoop(
	ros::NodeHandle &nh, std::shared_ptr<ros_control_boilerplate::FRCRobotInterface> hardware_interface)
	: nh_(nh)
	, hardware_interface_(hardware_interface)
	, controller_manager_(hardware_interface_.get(), nh_)
	, tracer_("GenericHWControlLoop " + nh.getNamespace())
{
	// Load rosparams
	ros::NodeHandle rpsnh(nh, name_);
	std::size_t error = 0;
	error += !rosparam_shortcuts::get(name_, rpsnh, "loop_hz", loop_hz_);
	error += !rosparam_shortcuts::get(name_, rpsnh, "cycle_time_error_threshold", cycle_time_error_threshold_);
	rosparam_shortcuts::shutdownIfError(name_, error);

	// Get current time for use with first update
	last_time_        = std::chrono::steady_clock::now();
	last_time_update_ = last_time_;
	last_time_write_  = last_time_;

	desired_update_period_ = ros::Duration(1.0 / loop_hz_);
}

void GenericHWControlLoop::run(void)
{
	ros::Rate rate(loop_hz_);
	while(ros::ok())
	{
		update();
		rate.sleep();
	}
}

void GenericHWControlLoop::update(void)
{
	// Get change in time in seconds
	tracer_.start_unique("get_loop_time");
	auto current_time = std::chrono::steady_clock::now();
	std::chrono::duration<double> elapsed_seconds = current_time - last_time_;
	const auto elapsed_time = ros::Duration(elapsed_seconds.count());
	last_time_ = current_time;

	// ROS_DEBUG_STREAM_THROTTLE_NAMED(1, "generic_hw_main","Sampled update loop with elapsed time " << elapsed_time.toSec());

	// Error check cycle time
	if (const double cycle_time_error = (elapsed_time - desired_update_period_).toSec();
		cycle_time_error > cycle_time_error_threshold_)
		ROS_WARN_STREAM_NAMED(name_, "Cycle time exceeded error threshold by: "
							  << std::setprecision(3) << cycle_time_error
							  << ", cycle time: " << std::setprecision(3) << elapsed_time
							  << ", threshold: " << cycle_time_error_threshold_);

	// Input
	tracer_.start_unique("read");
	hardware_interface_->read(ros::Time::now(), elapsed_time);

	// Control
	tracer_.start_unique("update");
	// Might be overkill to worry about difference in elapsed time
	// added by read taking a variable amount of time?
	current_time = std::chrono::steady_clock::now();
	elapsed_seconds = current_time - last_time_update_;
	last_time_update_ = current_time;
	controller_manager_.update(ros::Time::now(), ros::Duration(elapsed_seconds.count()));

	// Output
	tracer_.start_unique("write");
	current_time = std::chrono::steady_clock::now();
	elapsed_seconds = current_time - last_time_write_;
	last_time_write_ = current_time;
	hardware_interface_->write(ros::Time::now(), ros::Duration(elapsed_seconds.count()));

	tracer_.report(20);
}

}  // namespace
