// ---------------------------------------------------------------------
// Package:    TopiCo (https://github.com/AIS-Bonn/TopiCo)
// Version:    2021-03-30 12:12:52
// Maintainer: Marius Beul (mbeul@ais.uni-bonn.de)
// License:    BSD
// ---------------------------------------------------------------------

// Software License Agreement (BSD License)
// Copyright (c) 2021, Computer Science Institute VI, University of Bonn
// All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of University of Bonn, Computer Science Institute
//   VI nor the names of its contributors may be used to endorse or
//   promote products derived from this software without specific
//   prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// --------------------------------------------------------------------

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <topico/TopiCoConfig.h>

#include "rt_nonfinite.h"
#include "topico_wrapper.h"
#include "topico_wrapper_terminate.h"
#include "topico_wrapper_types.h"
#include "coder_array.h"
#include <sstream>
#include <stdexcept>
#include <string>
#include "topico/Waypoints.h"

struct CTX {
  // Initial state of the axes
  coder::array<double, 2U> initial_state;
  // Target waypoints
  coder::array<double, 3U> waypoints;
  // Max/Min velocities/jerks/accelerations, per axis, per waypoint
  coder::array<double, 2U> velocity_max;
  coder::array<double, 2U> velocity_min;
  coder::array<double, 2U> acceleration_max;
  coder::array<double, 2U> acceleration_min;
  coder::array<double, 2U> jerk_max;
  coder::array<double, 2U> jerk_min;
  // Global acceleration (?)
  coder::array<double, 1U> acceleration_global;
  // If the velocity/acceleration/jerk should be synced between the fastest & slowest axes (slows fastest axes to sync slowest ones)
  coder::array<bool, 2U> sync_velocities;
  coder::array<bool, 2U> sync_accelerations;
  coder::array<bool, 2U> sync_jerks;
  // If axes should be synced by waiting
  coder::array<bool, 2U> sync_axes;
  // Rotates axes to point towards the movement between waypoints
  coder::array<bool, 2U> rotate_axes;
  // Minimize time allowed outside velocity limits (alternative: more time outside limits but smoother movements)
  coder::array<bool, 2U> strict_velocity_limits;
  // If synchronized axes should try to catch up with unsynchronized ones
  coder::array<bool, 2U> catch_up_synced_axes;
  // (?)
  coder::array<signed char, 2U> direction;
  // Timesteps when the trajectory is sampled
  double sampling_timestamps;
  // If there's been an update
  bool updated;
  // If the CTX was initialized
  bool initialized;

  CTX() {}

  // Resize all the arrays, eg for new waypoints/axes
  void resize(int axes, int waypoints) {
    this->initial_state.set_size(axes, 3);
    this->waypoints.set_size(axes, 5, waypoints);
    this->velocity_max.set_size(axes, waypoints);
    this->velocity_min.set_size(axes, waypoints);
    this->acceleration_max.set_size(axes, waypoints);
    this->acceleration_min.set_size(axes, waypoints);
    this->jerk_max.set_size(axes, waypoints);
    this->jerk_min.set_size(axes, waypoints);
    this->acceleration_global.set_size(axes);
    this->sync_velocities.set_size(axes, waypoints);
    this->sync_accelerations.set_size(axes, waypoints);
    this->sync_jerks.set_size(axes, waypoints);
    this->sync_axes.set_size(axes, waypoints);
    this->rotate_axes.set_size(axes - 1, waypoints);
    this->strict_velocity_limits.set_size(axes, waypoints);
    this->catch_up_synced_axes.set_size(axes, waypoints);
    this->direction.set_size(axes, waypoints);
  }

  // (Re)initialize the CTX
  void init(const topico::WaypointsConstPtr& waypoints) {
    // Number of axes in each waypoint
    const int AXES = waypoints->waypoints[0].size();
    const int WAYPOINTS = waypoints->waypoints.size();
    this->resize(AXES, WAYPOINTS);

    for (int i = 0; i < AXES; i++) {
        // Position X/Y/Z (respectively)
        this->initial_state[i + (AXES * 0)] = waypoints->waypoints[0][i];
        // Velocity X/Y/Z (respectively, starts at 0 since nothing's moving)
        this->initial_state[i + (AXES * 1)] = 0;
        // Acceleration X/Y/Z (respectively)
        this->initial_state[i + (AXES * 2)] = 0.0;
    }

    for (int waypoint = 1; waypoint < WAYPOINTS; waypoint++) {
        for (int axis = 0; axis < AXES; axis++) {
            // Pos
            this->waypoints[(axis + AXES * 0) + AXES * 5 * (waypoint - 1)] = waypoints->waypoints[waypoint][axis];
            // Vel
            this->waypoints[(axis + AXES * 1) + AXES * 5 * (waypoint - 1)] = std::numeric_limits<double>::quiet_NaN();
            // Accel
            this->waypoints[(axis + AXES * 2) + AXES * 5 * (waypoint - 1)] = std::numeric_limits<double>::quiet_NaN();
            // Movement vel
            this->waypoints[(axis + AXES * 3) + AXES * 5 * (waypoint - 1)] = 0.0;
            // Reserved
            this->waypoints[(axis + AXES * 4) + AXES * 5 * (waypoint - 1)] = 0.0;
        }
    }

    // It was updated
    this->updated = true;
    // It was initialized
    this->initialized = true;
  }
};


// void dynamic_reconfigure_callback(topico::TopiCoConfig &config, uint32_t level)
// {
//   for (int idx_dim = 0; idx_dim < num_dim; idx_dim++) {  
//     A_global[idx_dim] = 0.0; 
//     for (int idx_wayp = 0; idx_wayp < num_wayp; idx_wayp++) {
//       V_max[idx_dim + num_dim * idx_wayp]        = config.V_max;
//       V_min[idx_dim + num_dim * idx_wayp]        = config.V_min;
//       A_max[idx_dim + num_dim * idx_wayp]        = config.A_max;
//       A_min[idx_dim + num_dim * idx_wayp]        = config.A_min;
//       J_max[idx_dim + num_dim * idx_wayp]        = config.J_max;
//       J_min[idx_dim + num_dim * idx_wayp]        = config.J_min;
//       b_sync_V[idx_dim + num_dim * idx_wayp]     = config.b_sync_V;
//       b_sync_A[idx_dim + num_dim * idx_wayp]     = config.b_sync_A;
//       b_sync_J[idx_dim + num_dim * idx_wayp]     = config.b_sync_J;
//       b_sync_W[idx_dim + num_dim * idx_wayp]     = config.b_sync_W;
//       b_hard_V_lim[idx_dim + num_dim * idx_wayp] = config.b_hard_V_lim;
//       b_catch_up[idx_dim + num_dim * idx_wayp]   = config.b_catch_up;
//       direction[idx_dim + num_dim * idx_wayp]    = config.direction;
//     }
//   }  
//   for (int idx_dim = 0; idx_dim < num_dim-1; idx_dim++) {  
//     for (int idx_wayp = 0; idx_wayp < num_wayp; idx_wayp++) {
//       b_rotate[idx_dim + num_dim * idx_wayp]     = config.b_rotate;
//     }
//   }
//   ts_rollout = config.ts_rollout;
// }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "topico");
  ros::NodeHandle nh;
  ros::Rate loop_rate(100);
  
  ros::Subscriber wayp_sub = nh.subscribe("wayp_odometry", 1, wayp_callback,ros::TransportHints().tcpNoDelay());
  ros::Subscriber init_sub = nh.subscribe("init_odometry", 1, init_callback,ros::TransportHints().tcpNoDelay());
  ros::Publisher path_pub  = nh.advertise<nav_msgs::Path>("trajectory_rollout", 0);
  
  std::string map_frame;
  nh.param<std::string>( "frame_id", map_frame, "world" );

  // Declare Outputs
  coder::array<struct0_T, 2U> J_setp_struct;
  coder::array<int, 2U> solution_out;
  coder::array<double, 2U> T_waypoints;
  coder::array<double, 2U> P;
  coder::array<double, 2U> V;
  coder::array<double, 2U> A;
  coder::array<double, 2U> J;
  coder::array<double, 2U> t;
  
  bool b_initialized = false;
  
  dynamic_reconfigure::Server<topico::TopiCoConfig> server;
  dynamic_reconfigure::Server<topico::TopiCoConfig>::CallbackType f;
  f = boost::bind(&dynamic_reconfigure_callback, _1, _2);
  server.setCallback(f);
 
  nav_msgs::Path path_rollout;
  
  while (ros::ok())
  {
    ros::spinOnce();
    ros::Time t_now = ros::Time::now();
    
    if (b_wayp_updated && b_init_updated) {
      b_initialized = true;
    }

    if (b_initialized && (b_init_updated || b_wayp_updated)) { // only replan when new data arrived...
      b_wayp_updated = false;
      b_init_updated = false;
 
      topico_wrapper(State_start, Waypoints, V_max, V_min, A_max, A_min, J_max, J_min, A_global, b_sync_V, b_sync_A, b_sync_J, b_sync_W, b_rotate, b_hard_V_lim, b_catch_up, direction, ts_rollout, J_setp_struct,solution_out, T_waypoints, P, V, A, J, t);

      int size_rollout = P.size(1);
      path_rollout.poses.resize(size_rollout);
      path_rollout.header.stamp = t_now;
      path_rollout.header.frame_id = map_frame;
      for (int idx = 0; idx < size_rollout; idx++)
      {
        path_rollout.poses[idx].pose.position.x = P[3*idx+0];
        path_rollout.poses[idx].pose.position.y = P[3*idx+1];
        path_rollout.poses[idx].pose.position.z = P[3*idx+2];
      }
      path_pub.publish(path_rollout);
    } else if(!b_initialized) {
      printf("Warning: Initial state and/or waypoint not published yet!\n");       
    }
    loop_rate.sleep();
  }

  topico_wrapper_terminate();
  return 0;
}

