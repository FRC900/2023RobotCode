#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <behavior_actions/Intaking2023Action.h>
#include <behavior_actions/Intake2023Action.h>
#include <behavior_actions/Elevater2023Action.h>
#include <behavior_actions/GamePieceState2023.h>
#include <std_msgs/Float64.h>

class IntakingServer2023
{
protected:

	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<behavior_actions::Intaking2023Action> as_;
	std::string action_name_;
	// create messages that are used to published result
	behavior_actions::Intaking2023Result result_;

	behavior_actions::GamePieceState2023 game_piece_state_;

	double server_timeout_;
	double cube_time_; // time to keep intaking after we see a cube
	double cone_time_; // time to keep intaking after we see a cone

	// shouldn't need to worry about the intake timeout since the intaker server handles that

	ros::Subscriber game_piece_sub_;

	actionlib::SimpleActionClient<behavior_actions::Intake2023Action> intake_ac_;
	actionlib::SimpleActionClient<behavior_actions::Elevater2023Action> elevater_ac_;

public:
	void gamePieceStateCallback(const behavior_actions::GamePieceState2023 &msg) {
		game_piece_state_ = msg;
	}

	IntakingServer2023(std::string name) :
		as_(nh_, name, boost::bind(&IntakingServer2023::executeCB, this, _1), false),
		action_name_(name),
		game_piece_sub_(nh_.subscribe("/game_piece_state", 1, &IntakingServer2023::gamePieceStateCallback, this)),
		intake_ac_("/intake/intake_server_2023", true),
		elevater_ac_("/elevater/elevater_server_2023", true)
	{
		if (!nh_.getParam("cube_time", cube_time_))
		{
			ROS_ERROR_STREAM("2023_intaking_server : could not find cube_time");
			return;
		}
		if (!nh_.getParam("cone_time", cone_time_))
		{
			ROS_ERROR_STREAM("2023_intaking_server : could not find cone_time");
			return;
		}
		if (!nh_.getParam("server_timeout", server_timeout_)) {
			ROS_WARN_STREAM("2023_intaking_server : could not find server_timeout, defaulting to 10 seconds");
			server_timeout_ = 10;
		}
		const std::map<std::string, std::string> service_connection_header{{"tcp_nodelay", "1"}};
		as_.start();
	}

	~IntakingServer2023(void)
	{
	}

	void executeCB(const behavior_actions::Intaking2023GoalConstPtr &goal)
	{
		bool success = true;

		if (!intake_ac_.waitForServer(ros::Duration(server_timeout_))) {
			ROS_ERROR_STREAM("2023_intaking_server : timed out connecting to intake server, aborting");
			result_.timed_out = true;
			as_.setAborted(result_);
			return;
		}

		behavior_actions::Intake2023Goal intakeGoal;
		goal.outtake = false;
		goal.go_fast = false;

		intake_ac_.sendGoal(intakeGoal);
		
		ros::Rate r(10);
		while (game_piece_state_.game_piece == behavior_actions::GamePieceState2023::NONE && !intake_ac_.getState().isDone()) {
			ros::spinOnce();
			ROS_INFO_STREAM_THROTTLE(0.1, "2023_intaking_server : waiting for a game piece...");
			r.sleep();
		}

		if (intake_ac_.getState() != intake_ac_.getState().SUCCEEDED) {
			ROS_ERROR_STREAM("2023_intaking_server : intake server failed, aborting!");
			as_.setAborted(result_);
			return;
		}

		if(success)
		{
			ROS_INFO_STREAM("2023_intaking_server : Succeeded");
			// set the action state to succeeded
			as_.setSucceeded(result_);
		}
	}


};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "intaking_server_2023");

	IntakingServer2023 intaking("intaking_server_2023");
	ros::spin();

	return 0;
}
