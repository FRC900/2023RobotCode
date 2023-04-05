#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include "talon_state_msgs/TalonState.h"
#include "talon_state_msgs/TalonConfig.h"
#include <fstream>
#include <angles/angles.h>

/*
Procedure
- Bevel facing right
- Run old dump offsets  
- 2pi - each offset
- that + pi
*/

talon_state_msgs::TalonState talon_state_msg;
talon_state_msgs::TalonConfig talon_config_msg;

bool get_offsets_srv(std_srvs::Trigger::Request& /*req*/, std_srvs::Trigger::Response& /*res*/)
{
	ROS_INFO_STREAM("running get_offsets_srv");
	std::stringstream offsets_file_name;
	offsets_file_name <<  "/home/ubuntu/2023RobotCode/zebROS_ws/src/ros_control_boilerplate/config/offsets_" << std::to_string(ros::Time::now().toSec()) << ".yaml";
	std::ofstream offsets_file(offsets_file_name.str());

	// TODO : make these config items, maybe?
	std::map<std::string, std::string> offset_joint_names;
	offset_joint_names["fl_angle"] = "steering_joint_fl";
	offset_joint_names["fr_angle"] = "steering_joint_fr";
	offset_joint_names["bl_angle"] = "steering_joint_bl";
	offset_joint_names["br_angle"] = "steering_joint_br";

	offsets_file << "swerve_drive_controller:" << std::endl;
	for (size_t i = 0; i < talon_state_msg.name.size(); i++)
	{
		auto it = offset_joint_names.find(talon_state_msg.name[i]);
		ROS_INFO_STREAM("index for talon: " << i);
		if (it != offset_joint_names.end())
		{
			offsets_file << "    " << it->second << ":" << std::endl;
			double offset = angles::normalize_angle(M_PI - fmod(talon_state_msg.position[i] - M_PI / 2.0, 2.0 * M_PI));
			if (!talon_config_msg.invert[i]) {
				offset = fmod(talon_state_msg.position[i] - M_PI / 2., 2. * M_PI); 
			}

			// Subtracing from pi because that is what worked before
			ROS_INFO_STREAM("OFFSET: " << offset);
			offsets_file << "        offset: " << offset << std::endl;
		}
	}

	return true;
}

void talon_states_cb(const talon_state_msgs::TalonState& msg)
{
	talon_state_msg = msg;
}

void talon_config_cb(const talon_state_msgs::TalonConfig& msg)
{
	talon_config_msg = msg;
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "dump_wheel_offsets");
	ros::NodeHandle nh;

	ros::Subscriber talon_states_sub = nh.subscribe("/frcrobot_jetson/talon_states", 1, talon_states_cb);
	ros::ServiceServer offsets_srv = nh.advertiseService("dump_offsets", get_offsets_srv);

	ros::spin();

	return 0;
}
