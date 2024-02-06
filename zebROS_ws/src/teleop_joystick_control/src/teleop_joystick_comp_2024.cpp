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

//std::unique_ptr<TeleopCmdVel<DynamicReconfigVars>> teleop_cmd_vel;

//frc_msgs::ButtonBoxState2023 button_box;

// TODO: Add 2024 versions
//std::shared_ptr<actionlib::SimpleActionClient<behavior_actions::Intaking2023Action>> intaking_ac;
//std::shared_ptr<actionlib::SimpleActionClient<behavior_actions::Placing2023Action>> placing_ac;
//std::shared_ptr<actionlib::SimpleActionClient<behavior_actions::FourbarElevatorPath2023Action>> pathing_ac;
//std::shared_ptr<actionlib::SimpleActionClient<behavior_actions::AlignAndPlaceGrid2023Action>> align_and_place_ac;

void talonFXProStateCallback(const talon_state_msgs::TalonFXProState talon_state)
{    
	// Year specific
	/*     
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

		pathed = (elevator_height >= config.elevator_threshold);
		// if we are currently above the height or want to go above the height
		if (elevator_height > config.elevator_threshold || elevator_setpoint > config.elevator_threshold) {
			//teleop_cmd_vel->setSuperSlowMode(true);
		}
		else {
			//teleop_cmd_vel->setSuperSlowMode(false);
		}
	}
	else {
		ROS_ERROR_STREAM_THROTTLE(0.1, "teleop_joystick_comp_2023 : Can not find talon with name = elevator_leader");
	}
	*/  
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
	if(joystick_id == 0) {

		static ros::Time last_header_stamp = evalateDriverCommands();

		if(!diagnostics_mode)
		{
			//Joystick1: buttonA
			//Joystick1: buttonA
			if(joystick_states_array[0].buttonAPress)
			{

			}
			if(joystick_states_array[0].buttonAButton)
			{
				
			}
			if(joystick_states_array[0].buttonARelease)
			{
				
			}

			//Joystick1: buttonB
			if(joystick_states_array[0].buttonBPress)
			{

			}
			if(joystick_states_array[0].buttonBButton)
			{	
			}
			if(joystick_states_array[0].buttonBRelease)
			{
			
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
				
			}
			if(joystick_states_array[0].bumperLeftButton)
			{
			}
			if(joystick_states_array[0].bumperLeftRelease)
			{
				
			}

			//Joystick1: bumperRight
			if(joystick_states_array[0].bumperRightPress)
			{
				
			}
			if(joystick_states_array[0].bumperRightButton)
			{
			}
			if(joystick_states_array[0].bumperRightRelease)
			{
				
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
				if(!joystick1_left_trigger_pressed)
				{
					
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

			//Joystick1: rightTrigger
			if(joystick_states_array[0].rightTrigger > config.trigger_threshold)
			{
				if(!joystick1_right_trigger_pressed)
				{
					
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

	// FIXME: Add TeleopCmdVel2024
	//teleop_cmd_vel = std::make_unique<TeleopCmdVel<DynamicReconfigVars>>(config);
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
	ros::Subscriber talon_states_sub = n.subscribe("/frcrobot_jetson/talonfxpro_states", 1, &talonFXProStateCallback);
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
		subscriber_array.push_back(n.subscribe(topic_array[j], 1, &evaluateCommands));
	}

	//ros::Subscriber button_box_sub = n.subscribe("/frcrobot_rio/button_box_states", 1, &buttonBoxCallback);

	ROS_WARN("joy_init");

	//teleop_cmd_vel->setRobotOrient(false, 0.0);
	//teleop_cmd_vel->setSlowMode(false);

	ros::spin();
	return 0;
}
