#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <behavior_actions/FourbarElevatorPath2023Action.h>
#include <std_msgs/Float64.h>
#include <talon_state_msgs/TalonState.h>

struct ElevatorFourbarPoint {
	// in that order
	double elevator;
	double fourbar;
};

using Path = std::vector<ElevatorFourbarPoint>;

class FourbarElevatorPathServer2023
{
protected:

	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<behavior_actions::FourbarElevatorPath2023Action> as_;
	std::string action_name_;
	// create messages that are used to published result
	behavior_actions::FourbarElevatorPath2023Result result_;

	ros::ServiceClient elevator_srv_;
	ros::ServiceClient fourbar_srv_;

	size_t fourbar_idx_;
	size_t elevator_idx_;
	double fourbar_position_;
	double elevator_position_;

	ros::Subscriber talon_states_sub_;

	std::map<std::string, Path> path_map_;
	
public:

	FourbarElevatorPathServer2023(std::string name) :
		as_(nh_, name, boost::bind(&FourbarElevatorPathServer2023::executeCB, this, _1), false),
		action_name_(name)
	{
		fourbar_idx_ = std::numeric_limits<size_t>::max();
		elevator_idx_ = std::numeric_limits<size_t>::max();
		talon_states_sub_ = nh_.subscribe("/frcrobot_jetson/talon_states", 1, &FourbarElevatorPathServer2023::talonStateCallback, this);
		XmlRpc::XmlRpcValue pathList;
    	nh_.getParam("paths", pathList);

		for (XmlRpc::XmlRpcValue::iterator path=pathList.begin(); path!=pathList.end(); ++path) {
			// path->first = name
			// path->second = [[e,f],[e,f]...]
			ROS_INFO_STREAM("path name = " << path->first << ". points:");
			Path p;
			for (size_t i = 0; i < path->second.size(); i++) {
				auto pt = ElevatorFourbarPoint();
				pt.elevator = static_cast<double>(path->second[i][0]);
				pt.fourbar = static_cast<double>(path->second[i][1]);
				p.push_back(pt);
				ROS_INFO_STREAM("(" << pt.elevator << ", " << pt.fourbar << ")");
			}
			path_map_[path->first] = p;
		}
		as_.start();
	}

	~FourbarElevatorPathServer2023(void)
	{
	}

	void talonStateCallback(const talon_state_msgs::TalonState &talon_state)
	{
		if (fourbar_idx_ == std::numeric_limits<size_t>::max()) // could maybe just check for > 0
		{
			for (size_t i = 0; i < talon_state.name.size(); i++)
			{
				if (talon_state.name[i] == "four_bar")
				{
					fourbar_idx_ = i;
					break;
				}
			}
		}
		if (!(fourbar_idx_ == std::numeric_limits<size_t>::max())) 
		{
			fourbar_position_ = talon_state.position[fourbar_idx_];
		}
		else {
			ROS_ERROR_STREAM("2023_fourbar_elevator_path_server : cannot find talon with name = four_bar");
		}

		if (elevator_idx_ == std::numeric_limits<size_t>::max()) // could maybe just check for > 0
		{
			for (size_t i = 0; i < talon_state.name.size(); i++)
			{
				if (talon_state.name[i] == "elevator_leader")
				{
					elevator_idx_ = i;
					break;
				}
			}
		}
		if (!(elevator_idx_ == std::numeric_limits<size_t>::max())) 
		{
			elevator_position_ = talon_state.position[elevator_idx_];
		}
		else {
			ROS_ERROR_STREAM("2023_fourbar_elevator_path_server : cannot find talon with name = elevator_leader");
		}
	}

	template<class C, class S>
	bool waitForResultAndCheckForPreempt(const ros::Duration & timeout, actionlib::SimpleActionClient<C> & ac, actionlib::SimpleActionServer<S> & as, bool preempt_at_timeout = false)
	{
		bool negative_timeout = false;
		if (timeout < ros::Duration(0, 0)) {
			ROS_WARN("waitForResultAndCheckForPreempt : Negative timeout, waiting forever");
			negative_timeout = true;
		}

		ros::Time timeout_time = ros::Time::now() + timeout;

		ros::Rate r(10);

		while (ros::ok() && !as.isPreemptRequested()) {
			ros::spinOnce();
			// Determine how long we should wait
			ros::Duration time_left = timeout_time - ros::Time::now();

			// Check if we're past the timeout time
			if (timeout > ros::Duration(0, 0) && time_left <= ros::Duration(0, 0) && !negative_timeout) {
				if (preempt_at_timeout) {
					ac.cancelGoalsAtAndBeforeTime(ros::Time::now());
				}
				break;
			}

			if (ac.getState().isDone()) {
				break;
			}
			r.sleep();
		}

		return ac.getState().isDone();
	}

	void executeCB(const behavior_actions::FourbarElevatorPath2023GoalConstPtr &goal)
	{

		as_.setSucceeded(result_);
	}


};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "fourbar_elevator_path_server_2023");

	FourbarElevatorPathServer2023 pather("fourbar_elevator_path_server_2023");
	ros::spin();

	return 0;
}
