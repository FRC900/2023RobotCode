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

//frc_msgs::ButtonBoxState2023 button_box;

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

bool up_down_switch_mid = false;
bool left_right_switch_mid = false;

double last_offset;
bool last_robot_orient;

// Diagnostic mode controls
int direction_x{};
int direction_y{};
int direction_z{};

bool robot_is_disabled{false};
bool elevator_up = false;
bool no_driver_input{false};

uint8_t grid;
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
	robot_is_disabled = msg.Disabled;
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

std::shared_ptr<actionlib::SimpleActionClient<path_follower_msgs::holdPositionAction>> distance_ac;

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


/*
void place(void) {
	ROS_WARN("Called unimplemented function \"place\"");
	behavior_actions::Placing2023Goal goal;
	goal.node = node;
	goal.piece = game_piece;
	goal.override_game_piece = true;
	goal.step = moved ? goal.PLACE_RETRACT : goal.MOVE;
	placing_ac->sendGoal(goal);
	moved = !moved;
}
*/

AutoPlaceState auto_place_state = AutoPlaceState::WAITING_TO_ALIGN; 

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

