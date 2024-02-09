// Toggle between rotation using rightStickX and rightTrigger - leftTrigger
#define ROTATION_WITH_STICK

#include "ros/ros.h"
#include "frc_msgs/JoystickState.h"
//#define NEED_JOINT_STATES
#ifdef NEED_JOINT_STATES
#include "sensor_msgs/JointState.h"
#endif
#include "geometry_msgs/Twist.h"
#include <string>
#include <cmath>

#include "std_srvs/Empty.h"

#include <vector>
#include "teleop_joystick_control/RobotOrient.h"

#include "frc_msgs/ButtonBoxState2023.h"
#include "frc_msgs/MatchSpecificData.h"

#include "actionlib/client/simple_action_client.h"

#include "ddynamic_reconfigure/ddynamic_reconfigure.h"

#include "teleop_joystick_control/TeleopCmdVel2023.h"
#include "behavior_actions/AutoMode.h"

#include "path_follower_msgs/holdPositionAction.h"

#include <imu_zero_msgs/ImuZeroAngle.h>
#include <angles/angles.h>
#include "teleop_joystick_control/RobotOrientationDriver.h"
#include <teleop_joystick_control/SnapConeCube.h>
#include <behavior_actions/Intaking2023Action.h>
#include <behavior_actions/Placing2023Action.h>
#include <behavior_actions/FourbarElevatorPath2023Action.h>
#include <talon_swerve_drive_controller_msgs/SetXY.h>
#include <behavior_actions/AlignAndPlaceGrid2023Action.h>
#include <talon_state_msgs/TalonFXProState.h>
#include <std_srvs/SetBool.h>
#include "behavior_actions/DriveToObjectAction.h"


#include "teleop_joystick_control/teleop_joystick_comp_general.h"

// start at waiting to align
// when driver transition becomes true, and we are at waiting to align, then align
// if aligning and you see press callback, place
// if on placing and see a press event, go to waiting to align  
enum AutoPlaceState {
	WAITING_TO_ALIGN = 0,
	ALIGNING = 1,
	PLACING = 2,
};

AutoPlaceState auto_place_state = AutoPlaceState::WAITING_TO_ALIGN; 

bool up_down_switch_mid;
bool left_right_switch_mid;
bool robot_is_disabled;
bool elevator_up;
uint8_t grid;

// Figure out a better name for this
struct DynamicReconfigVars2023
{
	double max_speed_elevator_extended{1.5};
	double max_rot_elevator_extended{0.2}; 
	double cone_length{0.3302/2};
	double cube_length{0.2032/2};
	double elevator_threshold{0.5};
	double cone_tolerance{0.1};
	double cube_tolerance{0.1};
} config2023;

frc_msgs::ButtonBoxState2023 button_box;
ros::ServiceClient FourbarRezeroSrv;

ros::ServiceClient snapConeCubeSrv;

std::shared_ptr<actionlib::SimpleActionClient<behavior_actions::DriveToObjectAction>> auto_note_pickup_ac;

std::shared_ptr<actionlib::SimpleActionClient<behavior_actions::Intaking2023Action>> intaking_ac;
std::shared_ptr<actionlib::SimpleActionClient<behavior_actions::Placing2023Action>> placing_ac;
std::shared_ptr<actionlib::SimpleActionClient<behavior_actions::FourbarElevatorPath2023Action>> pathing_ac;
std::shared_ptr<actionlib::SimpleActionClient<behavior_actions::AlignAndPlaceGrid2023Action>> align_and_place_ac;

size_t elevator_idx = std::numeric_limits<size_t>::max();
double elevator_height{0};
double elevator_setpoint{0};

void talonFXProStateCallback(const talon_state_msgs::TalonFXProState talon_state)
{           
	// fourbar_master_idx == max of size_t at the start
	if (elevator_idx == std::numeric_limits<size_t>::max()) // could maybe just check for > 0
	{
		for (size_t i = 0; i < talon_state.name.size(); i++)
		{
			if (talon_state.name[i] == "elevator_leader")
			{
				elevator_idx = i;
				break;
			}
		}
	}
	if (elevator_idx != std::numeric_limits<size_t>::max())
	{
		elevator_height = talon_state.position[elevator_idx];
		elevator_setpoint = talon_state.control_position[elevator_idx];

		pathed = (elevator_height >= config2023.elevator_threshold);
		// if we are currently above the height or want to go above the height
		if (elevator_height > config2023.elevator_threshold || elevator_setpoint > config2023.elevator_threshold) {
			teleop_cmd_vel->setCaps(config2023.max_speed_elevator_extended, config2023.max_rot_elevator_extended);

		}
		else {
			teleop_cmd_vel->resetCaps();
		}
	}
	else {
		ROS_ERROR_STREAM_THROTTLE(0.1, "teleop_joystick_comp_2023 : Can not find talon with name = elevator_leader");
	}
}

void place() {
	behavior_actions::Placing2023Goal goal;
	goal.node = node;
	goal.piece = game_piece;
	goal.override_game_piece = true;
	goal.step = moved ? goal.PLACE_RETRACT : goal.MOVE;
	placing_ac->sendGoal(goal);
	moved = !moved;
}

// start at waiting to align
// when driver transition becomes true, and we are at waiting to align, then align
// if aligning and you see press callback, place
// if on placing and see a press event, go to waiting to align  

uint8_t intake_piece = 0;

void buttonBoxCallback(const ros::MessageEvent<frc_msgs::ButtonBoxState2023 const>& event)
{
	//ROS_INFO_STREAM("Button Box callback running!");

	button_box = *(event.getMessage());

	static ros::Time last_header_stamp = button_box.header.stamp;
	
	bool driver_input_changed = false;
	if (last_no_driver_input != no_driver_input && no_driver_input == true) {
		driver_input_changed = true; 
	}
	ROS_INFO_STREAM_THROTTLE(1, "Auto place state = " << std::to_string(auto_place_state));
	
	if(button_box.lockingSwitchPress)
	{
	}

	if(button_box.lockingSwitchButton)
	{
		ROS_INFO_STREAM_THROTTLE(1, "Use pathing = true");
		use_pathing = true;
	}
	else
	{
		ROS_INFO_STREAM_THROTTLE(1, "Use pathing = false");
		use_pathing = false;
	}
	ROS_INFO_STREAM_THROTTLE(2, "Use pathing = " << use_pathing);
	ROS_INFO_STREAM_THROTTLE(2, "Alliance color = " << alliance_color);

	if(button_box.lockingSwitchRelease)
	{
		// // Clear out pressed state when switching modes
		// // so that the press will be seen by the new mode
		// joystick1_left_trigger_pressed = false;
		// joystick1_right_trigger_pressed = false;
		// // Force a publish 0 to all the diag mode
		// // controllers here before switching out
		// // of diagnostics mode
		// zero_all_diag_commands();
		// publish_diag_cmds();
		// diagnostics_mode = false;
		// robot_orientation_driver->stopRotation();
		// ROS_WARN_STREAM("Disabling diagnostics mode!");

		teleop_cmd_vel->setRobotOrient(false, 0.0);
		ROS_WARN_STREAM("Field relative mode!");
	}

	if(button_box.redButton) {
	}
	if(button_box.redPress) {
		ROS_WARN_STREAM("Preempting all actions!");
		placing_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
		intaking_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
		pathing_ac->cancelAllGoals();
		align_and_place_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
		auto_note_pickup_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
		pathed = false;
	}
	if(button_box.redRelease) {
	}

	if(button_box.topLeftConeButton) {
	}
	if(button_box.topLeftConePress) {
		ROS_INFO_STREAM("teleop : rezero fourbar!");
		std_srvs::Empty rezero_cmd;
		FourbarRezeroSrv.call(rezero_cmd);
	}
	if(button_box.topLeftConeRelease) {
	}

	if(button_box.topMiddleConeButton) {
	}
	if(button_box.topMiddleConePress) {
		intake_piece = behavior_actions::Intaking2023Goal::BASE_TOWARDS_US_CONE;
	}
	if(button_box.topMiddleConeRelease) {
	}

	if(button_box.topRightCubeButton) {
	}
	if(button_box.topRightCubePress) {
		intake_piece = behavior_actions::Intaking2023Goal::CUBE;
	}
	if(button_box.topRightCubeRelease) {
	}

	// HAS TO BE ABOVE BUTTON
	if(button_box.gridSelectConeLeftPress || button_box.gridSelectConeRightPress) {
		game_piece = behavior_actions::Placing2023Goal::VERTICAL_CONE;
		last_no_driver_input = false;
		if (use_pathing) {
			if (auto_place_state == AutoPlaceState::PLACING) {
				auto_place_state = AutoPlaceState::WAITING_TO_ALIGN;
				ROS_INFO_STREAM("Auto place state == WAITING_TO_ALIGN"); 
			}
			else if (auto_place_state == AutoPlaceState::ALIGNING && elevator_height > config2023.elevator_threshold) {
				place();
				auto_place_state = AutoPlaceState::PLACING;
				ROS_INFO_STREAM("Auto place state == PLACING"); 
			}
			else if (elevator_height <= config2023.elevator_threshold && auto_place_state == AutoPlaceState::ALIGNING) {
				auto_place_state = AutoPlaceState::WAITING_TO_ALIGN;
				ROS_INFO_STREAM("AutoPlaceState set back to waiting to align ");
			}
		}
		if (!use_pathing) {
			ROS_INFO_STREAM("Placing a cone!");
			place();
			pathed = false;
		}
	}

	if(button_box.gridSelectConeLeftButton && driver_input_changed && auto_place_state == AutoPlaceState::WAITING_TO_ALIGN && use_pathing) {
		game_piece = behavior_actions::Placing2023Goal::VERTICAL_CONE;
		behavior_actions::AlignAndPlaceGrid2023Goal align_goal;
		align_goal.alliance = alliance_color;
		moved = true;
		pathed = true;
		align_goal.tolerance = config2023	.cone_tolerance;
		align_goal.tolerance_for_extend = 0.25;
		align_goal.auto_place = false;
		align_goal.grid_id = 1 + grid_position;
		ROS_INFO_STREAM("Sending align to goal with id " << std::to_string(align_goal.grid_id));
		align_goal.node = node;
		align_goal.piece = game_piece;
		align_goal.override_game_piece = false;
		align_goal.from_Trex = false; // maybe should be true since that is what we do in auto?
		align_and_place_ac->sendGoal(align_goal);
		auto_place_state = AutoPlaceState::ALIGNING; 
		ROS_INFO_STREAM("AutoPlaceState == ALIGNING");
	}
	if(button_box.gridSelectConeLeftRelease) {
	}

	// PRESS MUST BE BEFORE BUTTON
	if(button_box.gridSelectCubePress) {
		last_no_driver_input = false;
		if (use_pathing) {
			if (auto_place_state == AutoPlaceState::PLACING) {
				auto_place_state = AutoPlaceState::WAITING_TO_ALIGN;
				ROS_INFO_STREAM("Auto place state == WAITING_TO_ALIGN"); 
			}
			else if (elevator_height <= config2023.elevator_threshold && auto_place_state == AutoPlaceState::ALIGNING) {
				auto_place_state = AutoPlaceState::WAITING_TO_ALIGN;
				ROS_INFO_STREAM("AutoPlaceState set back to waiting to align");
			}
		}
		if (!use_pathing) {
			ROS_INFO_STREAM("Placing a cube!");
			place();
		}
	}

	if(button_box.gridSelectCubeButton && driver_input_changed && auto_place_state == AutoPlaceState::WAITING_TO_ALIGN && use_pathing) {
		game_piece = behavior_actions::Placing2023Goal::CUBE;
		behavior_actions::AlignAndPlaceGrid2023Goal align_goal;
		align_goal.alliance = alliance_color;
		moved = true;
		pathed = true;
		align_goal.tolerance = config2023.cube_tolerance;
		align_goal.tolerance_for_extend = 2.0;
		align_goal.auto_place = true;
		align_goal.grid_id = 2 + grid_position;
		ROS_INFO_STREAM("Sending align to goal with id " << std::to_string(align_goal.grid_id));
		align_goal.node = node;
		align_goal.piece = game_piece;
		align_goal.override_game_piece = false;
		align_goal.from_Trex = false; // maybe should be true since that is what we do in auto?
		align_and_place_ac->sendGoal(align_goal);
		auto_place_state = AutoPlaceState::PLACING;
		ROS_INFO_STREAM("Auto Place STATE == PLACING");
	}
	if(button_box.gridSelectCubeRelease) {
	}

	if(button_box.gridSelectConeRightButton && driver_input_changed && auto_place_state == AutoPlaceState::WAITING_TO_ALIGN && use_pathing) {
		game_piece = behavior_actions::Placing2023Goal::VERTICAL_CONE; // type doesn't matter for placing
		behavior_actions::AlignAndPlaceGrid2023Goal align_goal;
		align_goal.alliance = alliance_color;
		moved = true;
		pathed = true;
		align_goal.tolerance = config2023.cone_tolerance;
		align_goal.tolerance_for_extend = 0.25;
		align_goal.auto_place = false;
		align_goal.grid_id = 3 + grid_position;
		ROS_INFO_STREAM("Sending align to goal with id " << std::to_string(align_goal.grid_id));
		align_goal.node = node;
		align_goal.piece = game_piece;
		align_goal.override_game_piece = false;
		align_goal.from_Trex = false; // maybe should be true since that is what we do in auto?
		
		align_and_place_ac->sendGoal(align_goal);
		auto_place_state = AutoPlaceState::ALIGNING; 
		ROS_INFO_STREAM("AutoPlaceState == ALIGNING");
	}
	if(button_box.gridSelectConeRightPress) {
		// already handled above
	}
	if(button_box.gridSelectConeRightRelease) {
	}

	if(button_box.heightSelectSwitchUpButton) {
		node = behavior_actions::Placing2023Goal::HIGH;
	}
	if(button_box.heightSelectSwitchUpPress) {
		if (robot_is_disabled)
		{
			auto_mode = 0;
			// should we be worried about messages being dropped here?
			behavior_actions::AutoMode auto_mode_msg;
			auto_mode_msg.header.stamp = ros::Time::now();
			auto_mode_msg.auto_mode = autoMode(2023);
			auto_mode_select_pub.publish(auto_mode_msg);
		}
		node = behavior_actions::Placing2023Goal::HIGH;
	}
	if(button_box.heightSelectSwitchUpRelease) {
	}

	if(!(button_box.heightSelectSwitchUpButton || button_box.heightSelectSwitchDownButton)) {
		if (!up_down_switch_mid) {
			node = behavior_actions::Placing2023Goal::MID;
			if (robot_is_disabled)
			{
				auto_mode = 1;
				// should we be worried about messages being dropped here?
				behavior_actions::AutoMode auto_mode_msg;
				auto_mode_msg.header.stamp = ros::Time::now();
				auto_mode_msg.auto_mode = autoMode(2023);
				auto_mode_select_pub.publish(auto_mode_msg);
			}
		}

		up_down_switch_mid = true;
	} else {
		up_down_switch_mid = false;
	}

	if(button_box.heightSelectSwitchDownButton) {
		node = behavior_actions::Placing2023Goal::HYBRID;
	}
	if(button_box.heightSelectSwitchDownPress) {
		node = behavior_actions::Placing2023Goal::HYBRID;
		if (robot_is_disabled)
		{
			auto_mode = 2;
			// should we be worried about messages being dropped here?
			behavior_actions::AutoMode auto_mode_msg;
			auto_mode_msg.header.stamp = ros::Time::now();
			auto_mode_msg.auto_mode = autoMode(2023);
			auto_mode_select_pub.publish(auto_mode_msg);
		}
	}
	if(button_box.heightSelectSwitchDownRelease) {
	}

	if(button_box.heightSelectSwitchLeftButton) {
	}
	if(button_box.heightSelectSwitchLeftPress) {
		if (robot_is_disabled)
		{
			auto_starting_pos = 1;
			// should we be worried about messages being dropped here?
			behavior_actions::AutoMode auto_mode_msg;
			auto_mode_msg.header.stamp = ros::Time::now();
			auto_mode_msg.auto_mode = autoMode(2023);
			auto_mode_select_pub.publish(auto_mode_msg);
		}
		grid_position = 0;
	}
	if(button_box.heightSelectSwitchLeftRelease) {
	}

	if(!(button_box.heightSelectSwitchLeftButton || button_box.heightSelectSwitchRightButton)) {
		if (!left_right_switch_mid) {
			node = behavior_actions::Placing2023Goal::MID;
			if (robot_is_disabled)
			{
				auto_starting_pos = 2;
				// should we be worried about messages being dropped here?
				behavior_actions::AutoMode auto_mode_msg;
				auto_mode_msg.header.stamp = ros::Time::now();
				auto_mode_msg.auto_mode = autoMode(2023);
				auto_mode_select_pub.publish(auto_mode_msg);
			}
		}
		grid_position = 3;
		left_right_switch_mid = true;
	} else {
		left_right_switch_mid = false;
	}

	if(button_box.heightSelectSwitchRightButton) {
	}
	if(button_box.heightSelectSwitchRightPress) {
		if (robot_is_disabled)
		{
			auto_starting_pos = 3;
			// should we be worried about messages being dropped here?
			behavior_actions::AutoMode auto_mode_msg;
			auto_mode_msg.header.stamp = ros::Time::now();
			auto_mode_msg.auto_mode = autoMode(2023);
			auto_mode_select_pub.publish(auto_mode_msg);
		}
		grid_position = 6;
	}
	if(button_box.heightSelectSwitchRightRelease) {
	}

	if(button_box.centralYellowButton) {
	}
	if(button_box.centralYellowPress) {
		robot_orientation_driver->setTargetOrientation(0.0, true);
	}
	if(button_box.centralYellowRelease) {
	}

	if(button_box.bottomLeftYellowButton) {
	}
	if(button_box.bottomLeftYellowPress) {
		robot_orientation_driver->setTargetOrientation(M_PI, true);
	}
	if(button_box.bottomLeftYellowRelease) {
	}

	if(button_box.bottomRightWhiteButton) {
	}
	if(button_box.bottomRightWhitePress) {
		ROS_INFO_STREAM("teleop_joystick_comp_2023 : zeroing IMU to 180");
		imu_zero_msgs::ImuZeroAngle imu_cmd;
		imu_cmd.request.angle = 180.0;
		IMUZeroSrv.call(imu_cmd);
		ROS_INFO_STREAM("teleop_joystick_comp_2023 : zeroing swerve odom");
		std_srvs::Empty odom_cmd;
		SwerveOdomZeroSrv.call(odom_cmd);
	}
	if(button_box.bottomRightWhiteRelease) {
	}

	if(button_box.rightGreenPress)
	{
		moveDirection(0, 1, 0);
	}
	if(button_box.rightGreenButton)
	{
		sendDirection();
	}
	if(button_box.rightGreenRelease)
	{
		moveDirection(0, -1, 0);
	}


	if(button_box.leftGreenPress)
	{
		moveDirection(0, -1, 0);
	}
	if(button_box.leftGreenButton)
	{
		sendDirection();
	}
	if(button_box.leftGreenRelease)
	{
		moveDirection(0, 1, 0);
	}


	if(button_box.topGreenPress)
	{
		moveDirection(1, 0, 0);
	}
	if(button_box.topGreenButton)
	{
		sendDirection();
	}
	if(button_box.topGreenRelease)
	{
		moveDirection(-1, 0, 0);
	}

	if(button_box.bottomGreenPress)
	{
		moveDirection(-1, 0, 0);
	}
	if(button_box.bottomGreenButton)
	{
		sendDirection();
	}
	if(button_box.bottomGreenRelease)
	{
		moveDirection(1, 0, 0);
	}
}

void evaluateCommands(const ros::MessageEvent<frc_msgs::JoystickState const>& event)
{
	//So the code can use specific joysticks
	int joystick_id = -1;

	const ros::M_string &header = event.getConnectionHeader();
	const std::string topic = header.at("topic");

	//Identifies the incoming message as the correct joystick based on the topic the message was recieved from
	for(size_t i = 0; (i < topic_array.size()); i++)
	{
		if(topic == topic_array[i])
		{
			joystick_states_array[i] = *(event.getMessage());
			joystick_id = i;
			break;
		}
	}

	if(joystick_id == -1)
	{
		ROS_ERROR("Joystick message topic not identified. Teleop callback failed.");
		return;
	}

	//Only do this for the first joystick
	if(joystick_id == 0)
	{
		static ros::Time last_header_stamp = evalateDriverCommands();

		if(!diagnostics_mode)
		{
			//Joystick1: buttonA
			//Joystick1: buttonA
			if(joystick_states_array[0].buttonAPress)
			{
				ROS_INFO_STREAM("teleop_joystick_comp_2023 : snapping to nearest cone and enabling robot relative driving mode!");
				if (teleop_joystick_control::SnapConeCube srv; snapConeCubeSrv.call(srv))
				{
					if (srv.response.nearest_cone_angle > -900) {
						ROS_INFO_STREAM("Using angle of " << srv.response.nearest_cone_angle);
						robot_orientation_driver->setTargetOrientation(srv.response.nearest_cone_angle, true /*from teleop*/);
						teleop_cmd_vel->setRobotOrient(true, 0);
					}
				}
			}
			if(joystick_states_array[0].buttonAButton)
			{
				
			}
			if(joystick_states_array[0].buttonARelease)
			{
				teleop_cmd_vel->setRobotOrient(false, 0);
			}

			//Joystick1: buttonB
			if(joystick_states_array[0].buttonBPress)
			{
				ROS_INFO_STREAM("teleop_joystick_comp_2023 : snapping to nearest cube and enabling robot relative driving mode!");
				if (teleop_joystick_control::SnapConeCube srv; snapConeCubeSrv.call(srv))
				{
					if (srv.response.nearest_cube_angle > -900) {
						ROS_INFO_STREAM_THROTTLE(1, "Using angle of " << srv.response.nearest_cube_angle);
						robot_orientation_driver->setTargetOrientation(srv.response.nearest_cube_angle, true /*from teleop*/);
						teleop_cmd_vel->setRobotOrient(true, 0);
					}
				}
			}
			if(joystick_states_array[0].buttonBButton)
			{	
			}
			if(joystick_states_array[0].buttonBRelease)
			{
				teleop_cmd_vel->setRobotOrient(false, 0);
			}

			//Joystick1: buttonX
			if(joystick_states_array[0].buttonXPress)
			{

			}
			if(joystick_states_array[0].buttonXButton)
			{
			}
			if(joystick_states_array[0].buttonXRelease)
			{
			}

			//Joystick1: buttonY
			if(joystick_states_array[0].buttonYPress)
			{

			}
			if(joystick_states_array[0].buttonYButton)
			{
			}
			if(joystick_states_array[0].buttonYRelease)
			{

			}

			//Joystick1: bumperLeft
			if(joystick_states_array[0].bumperLeftPress)
			{
				teleop_cmd_vel->setSlowMode(true);
			}
			if(joystick_states_array[0].bumperLeftButton)
			{
			}
			if(joystick_states_array[0].bumperLeftRelease)
			{
				teleop_cmd_vel->setSlowMode(false);
			}

			//Joystick1: bumperRight
			if(joystick_states_array[0].bumperRightPress)
			{
				behavior_actions::Intaking2023Goal goal;
				goal.outtake = goal.OUTTAKE_CONE;
				intaking_ac->sendGoal(goal);
			}
			if(joystick_states_array[0].bumperRightButton)
			{
			}
			if(joystick_states_array[0].bumperRightRelease)
			{
				intaking_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
			}


			// Should be the dpad right here

			//Joystick1: directionLeft
			if(joystick_states_array[0].directionLeftPress)
			{
				
			}
			if(joystick_states_array[0].directionLeftButton)
			{

			}
			else
			{
			}
			if(joystick_states_array[0].directionLeftRelease)
			{

			}

			//Joystick1: directionRight
			if(joystick_states_array[0].directionRightPress)
			{
			}
			if(joystick_states_array[0].directionRightButton)
			{
			}
			if(joystick_states_array[0].directionRightRelease)
			{
			}

			//Joystick1: directionUp
			if(joystick_states_array[0].directionUpPress)
			{
			}
			if(joystick_states_array[0].directionUpButton)
			{
			}
			if(joystick_states_array[0].directionUpRelease)
			{
			}

			//Joystick1: directionDown
			if(joystick_states_array[0].directionDownPress)
			{
			}
			if(joystick_states_array[0].directionDownButton)
			{
			}
			if(joystick_states_array[0].directionDownRelease)
			{
			}

			// end dpad


			//Joystick1: stickLeft
			if(joystick_states_array[0].stickLeftPress)
			{
			}
			if(joystick_states_array[0].stickLeftButton)
			{
			}
			else
			{
			}
			if(joystick_states_array[0].stickLeftRelease)
			{
			}

#ifdef ROTATION_WITH_STICK
			if(joystick_states_array[0].leftTrigger > config.trigger_threshold)
			{
				ROS_INFO_STREAM("LEFT TRIGGER");
				if(!joystick1_left_trigger_pressed)
				{
					behavior_actions::DriveToObjectGoal goal;
					goal.id = "note";
					goal.distance_away = 0.6;
					goal.tolerance = 0.1; // todo make configurable
					auto_note_pickup_ac->sendGoal(goal);
					ROS_WARN_STREAM("Auto picking up notes");
				}

				joystick1_left_trigger_pressed = true;
			}
			else
			{
				if(joystick1_left_trigger_pressed)
				{
					auto_note_pickup_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
				}

				joystick1_left_trigger_pressed = false;
			}

			//Joystick1: rightTrigger
			if(joystick_states_array[0].rightTrigger > config.trigger_threshold)
			{
				if(!joystick1_right_trigger_pressed)
				{
					behavior_actions::Intaking2023Goal goal;
					goal.piece = goal.VERTICAL_CONE;
					intaking_ac->sendGoal(goal);
				}

				joystick1_right_trigger_pressed = true;
			}
			else
			{
				if(joystick1_right_trigger_pressed)
				{
					intaking_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
				}

				joystick1_right_trigger_pressed = false;
			}
#endif
		}
		else
		{
			// Drive in diagnostic mode unconditionally
	#if 0
			//Joystick1 Diagnostics: leftStickY
			if(abs(joystick_states_array[0].leftStickY) > config.stick_threshold)
			{
			}

			//Joystick1 Diagnostics: leftStickX
			if(abs(joystick_states_array[0].leftStickX) > config.stick_threshold)
			{
			}

			//Joystick1 Diagnostics: rightStickY
			if(abs(joystick_states_array[0].rightStickY) > config.stick_threshold)
			{
			}

			//Joystick1 Diagnostics: rightStickX
			if(abs(joystick_states_array[0].rightStickX) > config.stick_threshold)
			{
			}
#endif

			//Joystick1 Diagnostics: stickLeft
			if(joystick_states_array[0].stickLeftPress)
			{
			}
			if(joystick_states_array[0].stickLeftButton)
			{
			}
			if(joystick_states_array[0].stickLeftRelease)
			{
			}

			//Joystick1 Diagnostics: stickRight
			if(joystick_states_array[0].stickRightPress)
			{
			}
			if(joystick_states_array[0].stickRightButton)
			{
			}
			if(joystick_states_array[0].stickRightRelease)
			{
			}

			//Joystick1 Diagnostics: buttonA
			if(joystick_states_array[0].buttonAPress)
			{
			}
			if(joystick_states_array[0].buttonAButton)
			{
			}
			if(joystick_states_array[0].buttonARelease)
			{
			}

			//Joystick1 Diagnostics: buttonB
			if(joystick_states_array[0].buttonBPress)
			{
			}
			if(joystick_states_array[0].buttonBButton)
			{
			}
			if(joystick_states_array[0].buttonBRelease)
			{
			}

			//Joystick1 Diagnostics: buttonX
			if(joystick_states_array[0].buttonXPress)
			{
			}
			if(joystick_states_array[0].buttonXButton)
			{
			}
			if(joystick_states_array[0].buttonXRelease)
			{
			}

			//Joystick1 Diagnostics: buttonY
			if(joystick_states_array[0].buttonYPress)
			{
			}
			if(joystick_states_array[0].buttonYButton)
			{
			}
			if(joystick_states_array[0].buttonYRelease)
			{
			}

			//Joystick1: buttonBack
			if(joystick_states_array[0].buttonBackPress)
			{
			}
			if(joystick_states_array[0].buttonBackButton)
			{
			}
			if(joystick_states_array[0].buttonBackRelease)
			{
			}

			//Joystick1: buttonStart
			if(joystick_states_array[0].buttonStartPress)
			{
			}
			if(joystick_states_array[0].buttonStartButton)
			{
			}
			if(joystick_states_array[0].buttonStartRelease)
			{
			}

			//Joystick1 Diagnostics: bumperLeft
			if(joystick_states_array[0].bumperLeftPress)
			{
			}
			if(joystick_states_array[0].bumperLeftButton)
			{
			}
			if(joystick_states_array[0].bumperLeftRelease)
			{
			}

			//Joystick1 Diagnostics: bumperRight
			if(joystick_states_array[0].bumperRightPress)
			{
			}
			if(joystick_states_array[0].bumperRightButton)
			{
			}
			if(joystick_states_array[0].bumperRightRelease)
			{
			}

			//Joystick1 Diagnostics: leftTrigger
			if(joystick_states_array[0].leftTrigger > config.trigger_threshold)
			{
				if(!joystick1_left_trigger_pressed)
				{

					//zero_all_diag_commands();
				}

				joystick1_left_trigger_pressed = true;
			}
			else
			{
				if(joystick1_left_trigger_pressed)
				{
				}

				joystick1_left_trigger_pressed = false;
			}
			//Joystick1 Diagnostics: rightTrigger
			if(joystick_states_array[0].rightTrigger > config.trigger_threshold)
			{
				if(!joystick1_right_trigger_pressed)
				{
					//zero_all_diag_commands();
				}

				joystick1_right_trigger_pressed = true;
			}
			else
			{
				if(joystick1_right_trigger_pressed)
				{
				}

				joystick1_right_trigger_pressed = false;
			}

			//Joystick1 Diagnostics: directionLeft
			if(joystick_states_array[0].directionLeftPress)
			{
			}
			if(joystick_states_array[0].directionLeftButton)
			{
			}
			if(joystick_states_array[0].directionLeftRelease)
			{
			}

			//Joystick1 Diagnostics: directionRight
			if(joystick_states_array[0].directionRightPress)
			{
			}
			if(joystick_states_array[0].directionRightButton)
			{
			}
			if(joystick_states_array[0].directionRightRelease)
			{
			}

			//Joystick1 Diagnostics: directionUp
			if(joystick_states_array[0].directionUpPress)
			{
			}
			if(joystick_states_array[0].directionUpButton)
			{
			}
			if(joystick_states_array[0].directionUpRelease)
			{
			}

			//Joystick1 Diagnostics: directionDown
			if(joystick_states_array[0].directionDownPress)
			{
			}
			if(joystick_states_array[0].directionDownButton)
			{
			}
			if(joystick_states_array[0].directionDownRelease)
			{
			}
		}

		last_header_stamp = joystick_states_array[0].header.stamp;
	}
	else if(joystick_id == 1)
	{
		// TODO Add empty button mappings here.
	}
	if (diagnostics_mode)
	{
		publish_diag_cmds();
	}
}

#ifdef NEED_JOINT_STATES
void jointStateCallback(const sensor_msgs::JointState &joint_state)
{
	// TODO - remove this if not used
}
#endif

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Joystick_controller");
	ros::NodeHandle n;
	ros::NodeHandle n_params(n, "teleop_params");
	ros::NodeHandle n_diagnostics_params(n, "teleop_diagnostics_params");
	ros::NodeHandle n_swerve_params(n, "/frcrobot_jetson/swerve_drive_controller");

	if(!n_params.getParam("max_speed_elevator_extended", config2023.max_speed_elevator_extended))
	{
		ROS_ERROR("Could not read max_speed_elevator_extended in teleop_joystick_comp");
	}
	if(!n_params.getParam("max_rot_elevator_extended", config2023.max_rot_elevator_extended))
	{
		ROS_ERROR("Could not read max_rot_elevator_extended in teleop_joystick_comp");
	}	
	if(!n_params.getParam("cone_tolerance", config2023.cone_tolerance))
	{
		ROS_ERROR("Could not read cone_tolerance in teleop_joystick_comp");
	}

	if(!n_params.getParam("cube_tolerance", config2023.cube_tolerance))
	{
		ROS_ERROR("Could not read cube_tolerance in teleop_joystick_comp");
	}

	TeleopInitializer initializer;
	initializer.set_n_params(n_params);
	
	initializer.add_custom_var( DDRVariable {"max_speed_elevator_extended", &config2023.max_speed_elevator_extended, "Max linear speed in elevator extended mode, in m/s", 0., 2} );
	initializer.add_custom_var( DDRVariable {"max_rot_elevator_extended", &config2023.max_rot_elevator_extended, "Max angular speed in elevator extended mode", 0., 1.} );
	initializer.add_custom_var( DDRVariable {"cone_tolerance", &config2023.cone_tolerance, "cone_tolerance", 0.0, 0.5} );
	initializer.add_custom_var( DDRVariable {"cube_tolerance", &config2023.cube_tolerance, "cube_tolerance", 0.0, 0.5} );

	const std::map<std::string, std::string> service_connection_header{{"tcp_nodelay", "1"}};

// Get back n and s_c_h
	FourbarRezeroSrv = n.serviceClient<std_srvs::Empty>("/frcrobot_jetson/four_bar_controller_2023/rezero_service", false, service_connection_header);
	ros::Subscriber talon_states_sub = n.subscribe("/frcrobot_jetson/talonfxpro_states", 1, &talonFXProStateCallback);

	

	intaking_ac = std::make_shared<actionlib::SimpleActionClient<behavior_actions::Intaking2023Action>>("/intaking/intaking_server_2023", true);
	placing_ac = std::make_shared<actionlib::SimpleActionClient<behavior_actions::Placing2023Action>>("/placing/placing_server_2023", true);
	pathing_ac = std::make_shared<actionlib::SimpleActionClient<behavior_actions::FourbarElevatorPath2023Action>>("/fourbar_elevator_path/fourbar_elevator_path_server_2023", true);
	align_and_place_ac = std::make_shared<actionlib::SimpleActionClient<behavior_actions::AlignAndPlaceGrid2023Action>>("/align_and_place_grid", true);

	ros::Subscriber button_box_sub = n.subscribe("/frcrobot_rio/button_box_states", 1, &buttonBoxCallback);
	return 0;
}