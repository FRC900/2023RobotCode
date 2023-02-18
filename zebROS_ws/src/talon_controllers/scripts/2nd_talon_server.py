#! /usr/bin/env python3
#*********************************************************************
# Software License Agreement (BSD License)
#
#  Copyright (c) 2009-2010, Willow Garage, Inc.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Willow Garage nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#********************************************************************/
from __future__ import print_function
from dynamic_reconfigure.client import Client as DynamicReconfigureClient
import roslib; roslib.load_manifest('dynamic_reconfigure')
import rospy
import dynamic_reconfigure.server
import dynamic_reconfigure.client
from talon_controllers.cfg import TalonConfigConfig
import time
global steering_joints
steering_joints = []
steering_joints.append(DynamicReconfigureClient('/frcrobot_jetson/swerve_drive_controller/steering_joint_fr', timeout=5))
steering_joints.append(DynamicReconfigureClient('/frcrobot_jetson/swerve_drive_controller/steering_joint_fl', timeout=9))
steering_joints.append(DynamicReconfigureClient('/frcrobot_jetson/swerve_drive_controller/steering_joint_br', timeout=7))
steering_joints.append(DynamicReconfigureClient('/frcrobot_jetson/swerve_drive_controller/steering_joint_bl', timeout=6))

def print_config(config):
    for k, v in config.items():
        print(k, ":", v)
    print('')

def config_callback(config):
    print("Got callback, configuration is: ")
    print_config(config)

def new_config_callback(client, config):
    #global old_callback
    #print("New callback is calling old callback...")
    #old_callback(config)
    global dummy_speed_joint
    global speed_joints
    print("New callback is done...")
    print("done")
    print_config(client.update_configuration(config))

def reconfigure(config, level):
    global steering_joints
    new_config_callback(steering_joints[0], config)
    new_config_callback(steering_joints[1], config)
    new_config_callback(steering_joints[2], config)
    new_config_callback(steering_joints[3], config)
    #runs new_config_callback function using the client as well as the config file from the cfg
    #all of tehese need to be intialized with callback so that the reconfigure server can
    #actaulyll find the updates on these joints

    return config  # Returns the updated configuration.


def main():
    rospy.init_node("talon_reconfigure_server_steering")
    
    dynamic_reconfigure.server.Server(TalonConfigConfig, reconfigure)
  
    while not rospy.is_shutdown():
        time.sleep(0.1)

if __name__ == '__main__':
    main()

