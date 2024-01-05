#!/bin/bash

cd ~/2023RobotCode/zebROS_ws/
echo INCOMPLETE > .native_build.status

if [ -z $ROS_ROOT ]; then
	source /opt/ros/noetic/setup.bash
	if [ ! -z devel/setup.bash ]; then
		source devel/setup.bash
	fi
elif [[ ! $ROS_ROOT = "/opt/ros/noetic/share/ros" ]]; then
	echo -e "\e[1m\e[31mROS is not configured for a native build (maybe set up for a cross build instead?)\e[0m"
	echo -e "\e[1m\e[31mRun ./native_build.sh in a new terminal window\e[0m"
	exit 1
fi
export PATH=$PATH:/usr/local/cuda/bin

EXTRA_SKIPLIST_PACKAGES=""
EXTRA_CMD_LINE=""
uname -a | grep -q x86_64
if [ $? -eq 1 ]; then
	EXTRA_SKIPLIST_PACKAGES="
		as726x_controllers \
		cancoder_controller \
		canifier_controller \
		demo_tf_node \
		field \
		frcrobot_description \
		frcrobot_gazebo \
		robot_characterization \
		robot_visualizer \
		rosbag_scripts \
		rospy_message_converter \
		rqt_driver_station_sim \
		stage_ros \
		template_controller \
		visualize_profile \
		zms_writer"
	EXTRA_CMD_LINE="--limit-status-rate 5"
fi

catkin config --skiplist \
	ackermann_steering_controller \
	adi_driver \
	adi_pico_driver \
	ar_track_alvar \
	color_spin \
	controllers_2019 \
	controllers_2019_msgs \
	controllers_2020 \
	controllers_2020_msgs \
	deeptag_ros \
	diff_drive_controller \
	effort_controllers \
	force_torque_sensor_controller \
	four_wheel_steering_controller \
	goal_detection \
	gripper_action_controller \
	navx_publisher \
	robot_characterization \
	rqt_joint_trajectory_controller \
	realsense2_camera \
	realsense2_description \
	robot_visualizer \
	rosserial_arduino \
	rosserial_chibios \
	rosserial_embeddedlinux \
	rosserial_mbed \
	rosserial_server \
	rosserial_test \
	rosserial_tivac \
	rosserial_vex_cortex \
	rosserial_vex_v5 \
	rosserial_windows \
	rosserial_xbee \
	teraranger_array \
	teraranger_array_converter \
	turing_smart_screen \
	velocity_controllers \
	zed_ros \
	$EXTRA_SKIPLIST_PACKAGES

catkin build -DCATKIN_ENABLE_TESTING=OFF -DBUILD_WITH_OPENMP=ON -DCMAKE_CXX_STANDARD=17 -DSETUPTOOLS_DEB_LAYOUT=OFF -DCMAKE_CXX_FLAGS="-DBOOST_BIND_GLOBAL_PLACEHOLDERS -Wno-psabi -DNON_POLLING" $EXTRA_CMD_LINE "$@"

if [ $? -ne 0 ] ; then
	echo FAIL > .native_build.status
	uname -a | grep -q x86_64
	if [ $? -eq 1 ]; then
		read -n 1 -s -r -p "Press any key to continue"
		echo
	fi
	/bin/false
else
	echo SUCCESS > .native_build.status
	/bin/true
fi

uname -a | grep -q x86_64
if [ $? -ne 1 ]; then
  ./merge_compile_commands.sh
  echo "build/compile_commands.json created"
fi

