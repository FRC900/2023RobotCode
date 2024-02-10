// Toggle between rotation using rightStickX and rightTrigger - leftTrigger
#define ROTATION_WITH_STICK

#include "ros/ros.h"
#include "frc_msgs/JoystickState.h"
//#define NEED_JOINT_STATES
#ifdef NEED_JOINT_STATES
#include "sensor_msgs/JointState.h"
#endif
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include <string>
#include <cmath>

#include "std_srvs/Empty.h"

#include <vector>
#include "teleop_joystick_control/RobotOrient.h"

#include "frc_msgs/MatchSpecificData.h"

#include "actionlib/client/simple_action_client.h"

#include "ddynamic_reconfigure/ddynamic_reconfigure.h"

#include "behavior_actions/AutoMode.h"

#include "path_follower_msgs/holdPositionAction.h"

#include <imu_zero_msgs/ImuZeroAngle.h>
#include <angles/angles.h>
#include "teleop_joystick_control/RobotOrientationDriver.h"
#include <talon_swerve_drive_controller_msgs/SetXY.h>
#include <talon_state_msgs/TalonFXProState.h>
#include <std_srvs/SetBool.h>

#include "teleop_joystick_control/teleop_joystick_comp_general.h"

struct DynamicReconfigVars config;

std::unique_ptr<TeleopCmdVel<DynamicReconfigVars>> teleop_cmd_vel;

std::unique_ptr<RobotOrientationDriver> robot_orientation_driver;

bool diagnostics_mode = false;

// array of joystick_states messages for multiple joysticks
std::vector <frc_msgs::JoystickState> joystick_states_array;
std::vector <std::string> topic_array;


ros::Publisher JoystickRobotVel;

ros::ServiceClient BrakeSrv;
ros::ServiceClient ParkSrv;
ros::ServiceClient IMUZeroSrv;
ros::ServiceClient SwerveOdomZeroSrv;

ros::Publisher auto_mode_select_pub;

bool joystick1_left_trigger_pressed = false;
bool joystick1_right_trigger_pressed = false;

double last_offset;
bool last_robot_orient;

// Diagnostic mode controls
int direction_x{};
int direction_y{};
int direction_z{};

bool no_driver_input{false};

uint8_t game_piece;
uint8_t node;

uint8_t auto_starting_pos = 1; // 1 indexed
uint8_t auto_mode = 0; // 0 indexed

bool sendRobotZero = false;
bool sendSetAngle = true;
double old_angular_z = 0.0;
bool use_pathing = false;
uint8_t grid_position = 0;
bool moved = false;
bool pathed = false;
bool last_no_driver_input = false;

uint8_t autoMode(int year) {
	// if ignoring starting positions, set the same auto modes for the three listed next to the switch position
	//        L  M      R
	// up =   1, 2, and 3
	// mid =  4, 5, and 6
	// down = 7, 8, and 9
	ROS_INFO_STREAM("teleop_joystick_comp_" << std::to_string(year) <<  ": auto_mode = " << std::to_string(auto_mode * 3 + auto_starting_pos));
	return auto_mode * 3 + auto_starting_pos;
}

uint8_t alliance_color{};
bool called_park_endgame = false;

void matchStateCallback(const frc_msgs::MatchSpecificData &msg)
{
	// TODO : if in diagnostic mode, zero all outputs on the
	// transition from enabled to disabled
	alliance_color = msg.allianceColor;
	if (!called_park_endgame && msg.matchTimeRemaining < config.match_time_to_park && msg.Autonomous == false && msg.Enabled == true && msg.matchTimeRemaining > 0) {
		// check for enabled and time != 0 so we don't trigger when the node starts (time defaults to 0, auto defaults to false)
		std_srvs::SetBool setBool;
		setBool.request.data = true;
		if (!ParkSrv.call(setBool))
		{
			ROS_ERROR("ParkSrv call failed in matchStateCallback");
		} else {
			called_park_endgame = true;
			ROS_INFO("ParkSrv called");
		}
	}
}

ros::ServiceClient setCenterSrv;

void moveDirection(int x, int y, int z) {
	geometry_msgs::Twist cmd_vel;
	direction_x += x;
	direction_y += y;
	direction_z += z;
	cmd_vel.linear.x = direction_x * config.button_move_speed;
	cmd_vel.linear.y = direction_y * config.button_move_speed;
	cmd_vel.linear.z = direction_z * config.button_move_speed;
	cmd_vel.angular.x = 0.0;
	cmd_vel.angular.y = 0.0;
	cmd_vel.angular.z = 0.0;

	JoystickRobotVel.publish(cmd_vel);
}

void sendDirection(void) {
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = direction_x * config.button_move_speed;
	cmd_vel.linear.y = direction_y * config.button_move_speed;
	cmd_vel.linear.z = direction_z * config.button_move_speed;
	cmd_vel.angular.x = 0.0;
	cmd_vel.angular.y = 0.0;
	cmd_vel.angular.z = 0.0;

	JoystickRobotVel.publish(cmd_vel);
}

void publish_diag_cmds(void)
{
	ROS_WARN("Called unimplemented function \"publish_diag_cmds\"");
	// should publish commands to the diagnostic mode nodes
}

void zero_all_diag_commands(void)
{
	ROS_WARN("Called unimplemented function \"zero_all_diag_commands\"");
	// should zero out all diagnostic mode commands
}

std::unique_ptr<actionlib::SimpleActionClient<path_follower_msgs::holdPositionAction>> distance_ac;

void preemptActionlibServers(void)
{
	ROS_WARN_STREAM("Preempting ALL actionlib servers!");
	// dont forget to add new actionlib servers here
}

bool orientCallback(teleop_joystick_control::RobotOrient::Request& req,
		teleop_joystick_control::RobotOrient::Response&/* res*/)
{
	last_offset = req.offset_angle;
	last_robot_orient = req.robot_orient;
	// Used to switch between robot orient and field orient driving
	teleop_cmd_vel->setRobotOrient(req.robot_orient, req.offset_angle);
	ROS_WARN_STREAM("Robot Orient = " << req.robot_orient << ", Offset Angle = " << req.offset_angle);
	return true;
}

void buttonBoxCallback(const ros::MessageEvent<std_msgs::Bool const>& event)
{
	ROS_WARN("Called unimplemented function \"buttonBoxCallback\"");
}

bool aligned_to_game_piece = false;

// TODO: Add joystick config system

ros::Time evalateDriverCommands(void) {
	static ros::Time last_header_stamp = joystick_states_array[0].header.stamp;

	teleop_cmd_vel->updateRateLimit(config);
	// TODO : make swerve invert the yaw so we can deal in ccw-positive angles
	// everywhere outside of that code
	geometry_msgs::Twist cmd_vel = teleop_cmd_vel->generateCmdVel(joystick_states_array[0], -robot_orientation_driver->getCurrentOrientation(), config);

	if (robot_orientation_driver->mostRecentCommandIsFromTeleop() || cmd_vel.linear.x != 0.0 || cmd_vel.linear.y != 0.0 || cmd_vel.angular.z != 0.0) {
		double original_angular_z = cmd_vel.angular.z;

		if (original_angular_z == 0.0 && old_angular_z != 0.0) {
			ROS_WARN_STREAM_THROTTLE(1, "Send set angle = false");
			sendSetAngle = false;
		}

		if (original_angular_z == 0.0 && !sendSetAngle) {
			ROS_INFO_STREAM("Old angular z " << old_angular_z << " signbit " << signbit(old_angular_z));
			double multiplier = 1;
			if (signbit(old_angular_z)) {
				multiplier = -1;
			}
			if (old_angular_z == 0.0) {
				ROS_WARN_STREAM("Old angular z is zero, wierd");
			}
			ROS_INFO_STREAM("Locking to current orientation!");
			robot_orientation_driver->setTargetOrientation(robot_orientation_driver->getCurrentOrientation() + multiplier * config.angle_to_add , true /* from telop */);
			sendSetAngle = true;
		}
		ROS_INFO_STREAM_THROTTLE(1, "CMD_VEL angular z" << cmd_vel.angular.z);

		if (cmd_vel.angular.z == 0.0)
		{
			cmd_vel.angular.z = robot_orientation_driver->getOrientationVelocityPIDOutput();
			if (fabs(cmd_vel.angular.z) < config.rotation_epsilon) {
				// COAST MODE
				//cmd_vel.angular.z = 0.001 * (cmd_vel.angular.z > 0 ? 1 : -1);
				cmd_vel.angular.z = 0.0;
			}
		}
		//ROS_WARN_STREAM("2023-Publishing " << cmd_vel.linear.x << " " << cmd_vel.linear.y << " " << cmd_vel.angular.z << " " << original_angular_z);
		
		/*original_angular_z == 0.0*/ 
		if((cmd_vel.linear.x == 0.0) && (cmd_vel.linear.y == 0.0) && ( cmd_vel.angular.z == 0.0 ) && !sendRobotZero)
		{
			no_driver_input = true;
			if (std_srvs::Empty empty; !BrakeSrv.call(empty))
			{
				ROS_ERROR("BrakeSrv call failed in sendRobotZero_");
			}
			ROS_INFO("BrakeSrv called");

			JoystickRobotVel.publish(cmd_vel);
			sendRobotZero = true;
		}
		// 0.002 is slightly more than the 0.001 we set for coast mode
		else if((cmd_vel.linear.x != 0.0) || (cmd_vel.linear.y != 0.0) || (fabs(cmd_vel.angular.z) >= 0.002))
		{
			JoystickRobotVel.publish(cmd_vel);
			sendRobotZero = false;
			no_driver_input = false;
			// if the original command was not zero, then teleop was controlling rotation

		}
		old_angular_z = original_angular_z;
	}
	return last_header_stamp;
}




#ifdef NEED_JOINT_STATES
void jointStateCallback(const sensor_msgs::JointState &joint_state)
{
	ROS_WARN("Called unimplemented function \"jointStateCallback\" ");
	// TODO - remove this if not used
}
#endif

int init(int argc, char **argv, void (*callback)(const ros::MessageEvent<frc_msgs::JoystickState const>&) )
{
	ros::init(argc, argv, "Joystick_controller");
	ros::NodeHandle n;
	ros::NodeHandle n_params(n, "teleop_params");
	ros::NodeHandle n_diagnostics_params(n, "teleop_diagnostics_params");
	ros::NodeHandle n_swerve_params(n, "/frcrobot_jetson/swerve_drive_controller");

	int num_joysticks = 1;
	if(!n_params.getParam("num_joysticks", num_joysticks))
	{
		ROS_ERROR("Could not read num_joysticks in teleop_joystick_comp");
	}
	if(!n_params.getParam("joystick_deadzone", config.joystick_deadzone))
	{
		ROS_ERROR("Could not read joystick_deadzone in teleop_joystick_comp");
	}
	if(!n_params.getParam("joystick_pow", config.joystick_pow))
	{
		ROS_ERROR("Could not read joystick_pow in teleop_joystick_comp");
	}
	if(!n_params.getParam("rotation_pow", config.rotation_pow))
	{
		ROS_ERROR("Could not read rotation_pow in teleop_joystick_comp");
	}
	if(!n_params.getParam("min_speed", config.min_speed))
	{
		ROS_ERROR("Could not read min_speed in teleop_joystick_comp");
	}
	if(!n_params.getParam("max_speed", config.max_speed))
	{
		ROS_ERROR("Could not read max_speed in teleop_joystick_comp");
	}
	if(!n_params.getParam("max_speed_slow", config.max_speed_slow))
	{
		ROS_ERROR("Could not read max_speed_slow in teleop_joystick_comp");
	}
	if(!n_params.getParam("button_move_speed", config.button_move_speed))
	{
		ROS_ERROR("Could not read button_move_speed in teleop_joystick_comp");
	}
	if(!n_params.getParam("max_rot", config.max_rot))
	{
		ROS_ERROR("Could not read max_rot in teleop_joystick_comp");
	}
	if(!n_params.getParam("max_rot_slow", config.max_rot_slow))
	{
		ROS_ERROR("Could not read max_rot_slow in teleop_joystick_comp");
	}
	if(!n_params.getParam("drive_rate_limit_time", config.drive_rate_limit_time))
	{
		ROS_ERROR("Could not read drive_rate_limit_time in teleop_joystick_comp");
	}
	if(!n_params.getParam("rotate_rate_limit_time", config.rotate_rate_limit_time))
	{
		ROS_ERROR("Could not read rotate_rate_limit_time in teleop_joystick_comp");
	}
	if(!n_params.getParam("trigger_threshold", config.trigger_threshold))
	{
		ROS_ERROR("Could not read trigger_threshold in teleop_joystick_comp");
	}
	if(!n_params.getParam("stick_threshold", config.stick_threshold))
	{
		ROS_ERROR("Could not read stick_threshold in teleop_joystick_comp");
	}
	if(!n_params.getParam("imu_zero_angle", config.imu_zero_angle))
	{
		ROS_ERROR("Could not read imu_zero_angle in teleop_joystick_comp");
	}
	if(!n_params.getParam("rotation_axis_scale", config.rotation_axis_scale))
	{
		ROS_ERROR("Could not read rotation_axis_scale in teleop_joystick_comp");
	}
	if(!n_params.getParam("rotation_epsilon", config.rotation_epsilon))
	{
		ROS_ERROR("Could not read rotation_epsilon in teleop_joystick_comp");
	}

	if(!n_params.getParam("angle_to_add", config.angle_to_add))
	{
		ROS_ERROR("Could not read angle_to_add in teleop_joystick_comp");
	}

	if(!n_params.getParam("match_time_to_park", config.match_time_to_park))
	{
		ROS_ERROR("Could not read match_time_to_park in teleop_joystick_comp");
	}

	ddynamic_reconfigure::DDynamicReconfigure ddr(n_params);

	ddr.registerVariable<double>("joystick_deadzone", &config.joystick_deadzone, "Joystick deadzone, in percent", 0., 1.);
	ddr.registerVariable<double>("min_speed", &config.min_speed, "Min linear speed to get robot to overcome friction, in m/s", 0, 1);
	ddr.registerVariable<double>("max_speed", &config.max_speed, "Max linear speed, in m/s", 0, 10.);
	ddr.registerVariable<double>("max_speed_slow", &config.max_speed_slow, "Max linear speed in slow mode, in m/s", 0., 5.);
	ddr.registerVariable<double>("max_rot", &config.max_rot, "Max angular speed", 0., 13.);
	ddr.registerVariable<double>("max_rot_slow", &config.max_rot_slow, "Max angular speed in slow mode", 0., 3.);
	ddr.registerVariable<double>("button_move_speed", &config.button_move_speed, "Linear speed when move buttons are pressed, in m/s", 0, 5);
	ddr.registerVariable<double>("joystick_pow", &config.joystick_pow, "Joystick Scaling Power, linear", 0., 5.);
	ddr.registerVariable<double>("rotation_pow", &config.rotation_pow, "Joystick Scaling Power, rotation", 0., 5.);
	ddr.registerVariable<double>("drive_rate_limit_time", &config.drive_rate_limit_time, "msec to go from full back to full forward", 0., 2000.);
	ddr.registerVariable<double>("rotate_rate_limit_time", &config.rotate_rate_limit_time, "msec to go from full counterclockwise to full clockwise", 0., 2000.);
	ddr.registerVariable<double>("trigger_threshold", &config.trigger_threshold, "Amount trigger has to be pressed to trigger action", 0., 1.);
	ddr.registerVariable<double>("stick_threshold", &config.stick_threshold, "Amount stick has to be moved to trigger diag mode action", 0., 1.);
	ddr.registerVariable<double>("imu_zero_angle", &config.imu_zero_angle, "Value to pass to imu/set_zero when zeroing", -360., 360.);
	ddr.registerVariable<double>("rotation_epsilon", &config.rotation_epsilon, "rotation_epsilon", 0.0, 1.0);
	ddr.registerVariable<double>("angle_to_add", &config.angle_to_add, "angle_to_add", 0.0, 10);
	ddr.registerVariable<double>("match_time_to_park", &config.match_time_to_park, "match_time_to_park", 0.0, 60.0);

	ddr.publishServicesTopics();

	teleop_cmd_vel = std::make_unique<TeleopCmdVel<DynamicReconfigVars>>(config);
	robot_orientation_driver = std::make_unique<RobotOrientationDriver>(n);

	const std::map<std::string, std::string> service_connection_header{{"tcp_nodelay", "1"}};

	BrakeSrv = n.serviceClient<std_srvs::Empty>("/frcrobot_jetson/swerve_drive_controller/brake", false, service_connection_header);
	ParkSrv = n.serviceClient<std_srvs::SetBool>("/frcrobot_jetson/swerve_drive_controller/toggle_park", false, service_connection_header);
	IMUZeroSrv = n.serviceClient<imu_zero_msgs::ImuZeroAngle>("/imu/set_imu_zero", false, service_connection_header);
	setCenterSrv = n.serviceClient<talon_swerve_drive_controller_msgs::SetXY>("/frcrobot_jetson/swerve_drive_controller/change_center_of_rotation", false, service_connection_header);	
	JoystickRobotVel = n.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1);
	SwerveOdomZeroSrv = n.serviceClient<std_srvs::Empty>("/frcrobot_jetson/swerve_drive_controller/reset_odom", false, service_connection_header);
#ifdef NEED_JOINT_STATES
	ros::Subscriber joint_states_sub = n.subscribe("/frcrobot_jetson/joint_states", 1, &jointStateCallback);
#endif
	//ros::Subscriber talon_states_sub = n.subscribe("/frcrobot_jetson/talonfxpro_states", 1, &talonFXProStateCallback);
	ros::Subscriber match_state_sub = n.subscribe("/frcrobot_rio/match_data", 1, matchStateCallback);
	ros::ServiceServer robot_orient_service = n.advertiseService("robot_orient", orientCallback);

	auto_mode_select_pub = n.advertise<behavior_actions::AutoMode>("/auto/auto_mode", 1, true);

	// TODO: Replace these with 2024 versions
	//intaking_ac = std::make_shared<actionlib::SimpleActionClient<behavior_actions::Intaking2023Action>>("/intaking/intaking_server_2023", true);
	//placing_ac = std::make_shared<actionlib::SimpleActionClient<behavior_actions::Placing2023Action>>("/placing/placing_server_2023", true);
	//pathing_ac = std::make_shared<actionlib::SimpleActionClient<behavior_actions::FourbarElevatorPath2023Action>>("/fourbar_elevator_path/fourbar_elevator_path_server_2023", true);
	//align_and_place_ac = std::make_shared<actionlib::SimpleActionClient<behavior_actions::AlignAndPlaceGrid2023Action>>("/align_and_place_grid", true);

	const ros::Duration startup_wait_time_secs(15);
	const ros::Time startup_start_time = ros::Time::now();
	ros::Duration startup_wait_time;
	startup_wait_time = std::max(startup_wait_time_secs - (ros::Time::now() - startup_start_time), ros::Duration(0.1));
	if(!BrakeSrv.waitForExistence(startup_wait_time))
	{
		ROS_ERROR("Wait (15 sec) timed out, for Brake Service in teleop_joystick_comp.cpp");
	}

	startup_wait_time = std::max(startup_wait_time_secs - (ros::Time::now() - startup_start_time), ros::Duration(0.1));
	if(!IMUZeroSrv.waitForExistence(startup_wait_time))
	{
		ROS_ERROR("Wait (1 sec) timed out, for IMU Zero Service in teleop_joystick_comp.cpp");
	}

	//Read from _num_joysticks joysticks
	// Set up this callback last, since it might use all of the various stuff
	// initialized above here. Setting it up first risks the chance that a callback
	// happens immediately and tries to use them before they have valid values

	std::vector <ros::Subscriber> subscriber_array;
	joystick_states_array.resize(num_joysticks);
	for(int j = 0; j < num_joysticks; j++)
	{
		std::stringstream s;
		s << "/frcrobot_rio/joystick_states";
		s << (j+1);
		topic_array.push_back(s.str());
		subscriber_array.push_back(n.subscribe(topic_array[j], 1, callback));
	}

	//ros::Subscriber button_box_sub = n.subscribe("/frcrobot_rio/button_box_states", 1, &buttonBoxCallback);

	ROS_WARN("joy_init");

	teleop_cmd_vel->setRobotOrient(false, 0.0);
	teleop_cmd_vel->setSlowMode(false);

	ros::spin();
	return 0;
}

void TeleopInitializer::add_custom_var(DDRVariable var) {
	custom_vars.push_back(var);
}

void TeleopInitializer::set_n_params(ros::NodeHandle n_params) {
	n_params_ = n_params;
}

void TeleopInitializer::set_n(ros::NodeHandle n) {
	n_= n;
}

void TeleopInitializer::init(void (*callback)(const ros::MessageEvent<frc_msgs::JoystickState const>&)) {
	int num_joysticks = 1;
	if(!n_params_.getParam("num_joysticks", num_joysticks))
	{
		ROS_ERROR("Could not read num_joysticks in teleop_joystick_comp");
	}
	if(!n_params_.getParam("joystick_deadzone", config.joystick_deadzone))
	{
		ROS_ERROR("Could not read joystick_deadzone in teleop_joystick_comp");
	}
	if(!n_params_.getParam("joystick_pow", config.joystick_pow))
	{
		ROS_ERROR("Could not read joystick_pow in teleop_joystick_comp");
	}
	if(!n_params_.getParam("rotation_pow", config.rotation_pow))
	{
		ROS_ERROR("Could not read rotation_pow in teleop_joystick_comp");
	}
	if(!n_params_.getParam("min_speed", config.min_speed))
	{
		ROS_ERROR("Could not read min_speed in teleop_joystick_comp");
	}
	if(!n_params_.getParam("max_speed", config.max_speed))
	{
		ROS_ERROR("Could not read max_speed in teleop_joystick_comp");
	}
	if(!n_params_.getParam("max_speed_slow", config.max_speed_slow))
	{
		ROS_ERROR("Could not read max_speed_slow in teleop_joystick_comp");
	}
	if(!n_params_.getParam("button_move_speed", config.button_move_speed))
	{
		ROS_ERROR("Could not read button_move_speed in teleop_joystick_comp");
	}
	if(!n_params_.getParam("max_rot", config.max_rot))
	{
		ROS_ERROR("Could not read max_rot in teleop_joystick_comp");
	}
	if(!n_params_.getParam("max_rot_slow", config.max_rot_slow))
	{
		ROS_ERROR("Could not read max_rot_slow in teleop_joystick_comp");
	}
	if(!n_params_.getParam("drive_rate_limit_time", config.drive_rate_limit_time))
	{
		ROS_ERROR("Could not read drive_rate_limit_time in teleop_joystick_comp");
	}
	if(!n_params_.getParam("rotate_rate_limit_time", config.rotate_rate_limit_time))
	{
		ROS_ERROR("Could not read rotate_rate_limit_time in teleop_joystick_comp");
	}
	if(!n_params_.getParam("trigger_threshold", config.trigger_threshold))
	{
		ROS_ERROR("Could not read trigger_threshold in teleop_joystick_comp");
	}
	if(!n_params_.getParam("stick_threshold", config.stick_threshold))
	{
		ROS_ERROR("Could not read stick_threshold in teleop_joystick_comp");
	}
	if(!n_params_.getParam("imu_zero_angle", config.imu_zero_angle))
	{
		ROS_ERROR("Could not read imu_zero_angle in teleop_joystick_comp");
	}
	if(!n_params_.getParam("rotation_axis_scale", config.rotation_axis_scale))
	{
		ROS_ERROR("Could not read rotation_axis_scale in teleop_joystick_comp");
	}
	if(!n_params_.getParam("rotation_epsilon", config.rotation_epsilon))
	{
		ROS_ERROR("Could not read rotation_epsilon in teleop_joystick_comp");
	}
	if(!n_params_.getParam("angle_to_add", config.angle_to_add))
	{
		ROS_ERROR("Could not read angle_to_add in teleop_joystick_comp");
	}
	if(!n_params_.getParam("match_time_to_park", config.match_time_to_park))
	{
		ROS_ERROR("Could not read match_time_to_park in teleop_joystick_comp");
	}

	ddynamic_reconfigure::DDynamicReconfigure ddr(n_params_);

	ddr.registerVariable<double>("joystick_deadzone", &config.joystick_deadzone, "Joystick deadzone, in percent", 0., 1.);
	ddr.registerVariable<double>("min_speed", &config.min_speed, "Min linear speed to get robot to overcome friction, in m/s", 0, 1);
	ddr.registerVariable<double>("max_speed", &config.max_speed, "Max linear speed, in m/s", 0, 10.);
	ddr.registerVariable<double>("max_speed_slow", &config.max_speed_slow, "Max linear speed in slow mode, in m/s", 0., 5.);
	ddr.registerVariable<double>("max_rot", &config.max_rot, "Max angular speed", 0., 13.);
	ddr.registerVariable<double>("max_rot_slow", &config.max_rot_slow, "Max angular speed in slow mode", 0., 3.);
	ddr.registerVariable<double>("button_move_speed", &config.button_move_speed, "Linear speed when move buttons are pressed, in m/s", 0, 5);
	ddr.registerVariable<double>("joystick_pow", &config.joystick_pow, "Joystick Scaling Power, linear", 0., 5.);
	ddr.registerVariable<double>("rotation_pow", &config.rotation_pow, "Joystick Scaling Power, rotation", 0., 5.);
	ddr.registerVariable<double>("drive_rate_limit_time", &config.drive_rate_limit_time, "msec to go from full back to full forward", 0., 2000.);
	ddr.registerVariable<double>("rotate_rate_limit_time", &config.rotate_rate_limit_time, "msec to go from full counterclockwise to full clockwise", 0., 2000.);
	ddr.registerVariable<double>("trigger_threshold", &config.trigger_threshold, "Amount trigger has to be pressed to trigger action", 0., 1.);
	ddr.registerVariable<double>("stick_threshold", &config.stick_threshold, "Amount stick has to be moved to trigger diag mode action", 0., 1.);
	ddr.registerVariable<double>("imu_zero_angle", &config.imu_zero_angle, "Value to pass to imu/set_zero when zeroing", -360., 360.);
	ddr.registerVariable<double>("rotation_epsilon", &config.rotation_epsilon, "rotation_epsilon", 0.0, 1.0);
	ddr.registerVariable<double>("angle_to_add", &config.angle_to_add, "angle_to_add", 0.0, 10);
	ddr.registerVariable<double>("match_time_to_park", &config.match_time_to_park, "match_time_to_park", 0.0, 60.0);

	// register custom variables
	for (DDRVariable& var : this->custom_vars) {
		ddr.registerVariable<double>(var.name, &var.value, var.description, var.min, var.max);
	}

	ddr.publishServicesTopics();

	teleop_cmd_vel = std::make_unique<TeleopCmdVel<DynamicReconfigVars>>(config);
	robot_orientation_driver = std::make_unique<RobotOrientationDriver>(n_);

	const std::map<std::string, std::string> service_connection_header{{"tcp_nodelay", "1"}};

	BrakeSrv = n_.serviceClient<std_srvs::Empty>("/frcrobot_jetson/swerve_drive_controller/brake", false, service_connection_header);
	ParkSrv = n_.serviceClient<std_srvs::SetBool>("/frcrobot_jetson/swerve_drive_controller/toggle_park", false, service_connection_header);
	IMUZeroSrv = n_.serviceClient<imu_zero_msgs::ImuZeroAngle>("/imu/set_imu_zero", false, service_connection_header);
	setCenterSrv = n_.serviceClient<talon_swerve_drive_controller_msgs::SetXY>("/frcrobot_jetson/swerve_drive_controller/change_center_of_rotation", false, service_connection_header);	
	JoystickRobotVel = n_.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1);
	SwerveOdomZeroSrv = n_.serviceClient<std_srvs::Empty>("/frcrobot_jetson/swerve_drive_controller/reset_odom", false, service_connection_header);
#ifdef NEED_JOINT_STATES
	ros::Subscriber joint_states_sub = n_.subscribe("/frcrobot_jetson/joint_states", 1, &jointStateCallback);
#endif
	//ros::Subscriber talon_states_sub = n.subscribe("/frcrobot_jetson/talonfxpro_states", 1, &talonFXProStateCallback);
	ros::Subscriber match_state_sub = n_.subscribe("/frcrobot_rio/match_data", 1, matchStateCallback);
	ros::ServiceServer robot_orient_service = n_.advertiseService("robot_orient", orientCallback);
	// Subscribe to arbitrary clients

	auto_mode_select_pub = n_.advertise<behavior_actions::AutoMode>("/auto/auto_mode", 1, true);

	const ros::Duration startup_wait_time_secs(15);
	const ros::Time startup_start_time = ros::Time::now();
	ros::Duration startup_wait_time;
	startup_wait_time = std::max(startup_wait_time_secs - (ros::Time::now() - startup_start_time), ros::Duration(0.1));
	if(!BrakeSrv.waitForExistence(startup_wait_time))
	{
		ROS_ERROR("Wait (15 sec) timed out, for Brake Service in teleop_joystick_comp.cpp");
	}

	startup_wait_time = std::max(startup_wait_time_secs - (ros::Time::now() - startup_start_time), ros::Duration(0.1));
	if(!IMUZeroSrv.waitForExistence(startup_wait_time))
	{
		ROS_ERROR("Wait (1 sec) timed out, for IMU Zero Service in teleop_joystick_comp.cpp");
	}

	//Read from _num_joysticks joysticks
	// Set up this callback last, since it might use all of the various stuff
	// initialized above here. Setting it up first risks the chance that a callback
	// happens immediately and tries to use them before they have valid values

	std::vector <ros::Subscriber> subscriber_array;
	joystick_states_array.resize(num_joysticks);
	for(int j = 0; j < num_joysticks; j++)
	{
		std::stringstream s;
		s << "/frcrobot_rio/joystick_states";
		s << (j+1);
		topic_array.push_back(s.str());
		subscriber_array.push_back(n_.subscribe(topic_array[j], 1, callback));
	}

	ROS_WARN("joy_init");

	teleop_cmd_vel->setRobotOrient(false, 0.0);
	teleop_cmd_vel->setSlowMode(false);

	ros::spin();
}