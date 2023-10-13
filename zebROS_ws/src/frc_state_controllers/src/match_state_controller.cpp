#include <controller_interface/controller.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <frc_interfaces/match_data_interface.h>
#include "frc_msgs/MatchSpecificData.h"
#include "periodic_interval_counter/periodic_interval_counter.h"

namespace match_state_controller
{
class MatchStateController: public controller_interface::Controller<hardware_interface::MatchStateInterface>
{
private:
		hardware_interface::MatchStateHandle match_state_;
		std::unique_ptr<realtime_tools::RealtimePublisher<frc_msgs::MatchSpecificData> > realtime_pub_;
		std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
		double publish_rate_{20};
public:
    bool init(hardware_interface::MatchStateInterface *hw,
              ros::NodeHandle                         &root_nh,
              ros::NodeHandle                         &controller_nh)

	{
		ROS_INFO_STREAM_NAMED("match_state_controller", "init is running");

		//get match name, check if there is more or less than 1 match
		std::vector<std::string> match_names = hw->getNames();
		if (match_names.size() > 1) {
			ROS_ERROR_STREAM("Cannot initialize multiple match state joints");
			return false; }
		else if (match_names.size() < 1) {
			ROS_ERROR_STREAM("Cannot initialize zero match state joints");
			return false; }

		const std::string match_name = match_names[0];

		//get publish rate from config file
		if (!controller_nh.getParam("publish_rate", publish_rate_))
		{
			ROS_WARN("Could not read publish_rate in match state controller");
		}
		else if (publish_rate_ <= 0.0)
		{
			ROS_ERROR_STREAM("Invalid publish_rate in match state controller (" << publish_rate_ << ")");
			return false;
		}
		interval_counter_ = std::make_unique<PeriodicIntervalCounter>(publish_rate_);

		//set up publisher
		realtime_pub_ = std::make_unique<realtime_tools::RealtimePublisher<frc_msgs::MatchSpecificData>>(root_nh, "match_data", 2);

		auto &m = realtime_pub_->msg_;

		m.matchTimeRemaining = 0.0;
		m.gameSpecificData.clear();
		m.eventName = "";
		m.allianceColor = 0.0;
		m.matchType = 0.0;
		m.driverStationLocation = 0.0;
		m.matchNumber = 0.0;
		m.replayNumber = 0.0;
		m.Enabled = false;
		m.Disabled = false;
		m.Autonomous = false;
		m.DSAttached = false;
		m.FMSAttached = false;
		m.OperatorControl = false;
		m.Test = false;
		m.EStopped = false;
		m.BatteryVoltage = 0.0;
		m.getMatchTimeStatus = "";
		m.getAllianceStationStatus = "";
		m.getVinVoltageStatus = "";

		match_state_ = hw->getHandle(match_name);

		return true;
	}

    void starting(const ros::Time &time)
    {
		interval_counter_->reset();
    }

    void update(const ros::Time &time, const ros::Duration &period)
	{
		if (interval_counter_->update(period))
		{
			if (realtime_pub_->trylock())
			{
				auto &m = realtime_pub_->msg_;

				m.header.stamp = time;

				const auto &ms = match_state_;

				//read from the object and stuff it in a msg
				m.matchTimeRemaining = ms->getMatchTimeRemaining();
				m.gameSpecificData = ms->getGameSpecificData();
				m.eventName = ms->getEventName();
				m.allianceColor = ms->getAllianceColor();
				m.matchType = ms->getMatchType();
				m.driverStationLocation = ms->getDriverStationLocation();
				m.matchNumber = ms->getMatchNumber();
				m.replayNumber = ms->getReplayNumber();
				m.Enabled = ms->isEnabled();
				m.Disabled = ms->isDisabled();
				m.Autonomous = ms->isAutonomous();
				m.DSAttached = ms->isDSAttached();
				m.FMSAttached = ms->isFMSAttached();
				m.OperatorControl = ms->isOperatorControl();
				m.Test = ms->isTest();
				m.EStopped = ms->isEStopped();
				m.BatteryVoltage = ms->getBatteryVoltage();

				m.getMatchTimeStatus = ms->getGetMatchTimeStatus();
				m.getAllianceStationStatus = ms->getGetAllianceStationStatus();
				m.getVinVoltageStatus = ms->getGetVinVoltageStatus();
				realtime_pub_->unlockAndPublish();
			}
			else
			{
				interval_counter_->force_publish();
			}
		}
	}

    void stopping(const ros::Time & )
    {}
}; // class
} // namespace

namespace state_listener_controller
{
class MatchStateListenerController :
	public controller_interface::Controller<hardware_interface::RemoteMatchStateInterface>
{
private:
	ros::Subscriber sub_command_;
	hardware_interface::MatchStateWritableHandle handle_;

	// Real-time buffer holds the last command value read from the "command" topic.
	realtime_tools::RealtimeBuffer<hardware_interface::MatchHWState> command_buffer_;

    // Save data from the published message
	void commandCB(const frc_msgs::MatchSpecificDataConstPtr &msg)
	{
		hardware_interface::MatchHWState data;
		data.setMatchTimeRemaining(msg->matchTimeRemaining);

		data.setGameSpecificData(msg->gameSpecificData);
		data.setEventName(msg->eventName);

		data.setAllianceColor(msg->allianceColor);
		data.setMatchType(msg->matchType);
		data.setDriverStationLocation(msg->driverStationLocation);
		data.setMatchNumber(msg->matchNumber);
		data.setReplayNumber(msg->replayNumber);

		data.setEnabled(msg->Enabled);
		data.setDisabled(msg->Disabled);
		data.setAutonomous(msg->Autonomous);
		data.setFMSAttached(msg->FMSAttached);
		data.setDSAttached(msg->DSAttached);
		data.setOperatorControl(msg->OperatorControl);
		data.setTest(msg->Test);

		data.setBatteryVoltage(msg->BatteryVoltage);

		data.setGetMatchTimeStatus(msg->getMatchTimeStatus);
		data.setGetAllianceStationStatus(msg->getAllianceStationStatus);
		data.setGetVinVoltageStatus(msg->getVinVoltageStatus);
		command_buffer_.writeFromNonRT(data);
	}

public:
	virtual ~MatchStateListenerController()
	{
		sub_command_.shutdown();
	}

	bool init(hardware_interface::RemoteMatchStateInterface *hw, ros::NodeHandle &n)
	{
		// Read list of hw, make a list, grab handles for them, plus allocate storage space
		auto joint_names = hw->getNames();
		if (joint_names.size() == 0)
		{
			ROS_ERROR("Match State Listener Controller : no remote match joints defined. Don't run this on a roboRio");
		}
		ROS_INFO_STREAM("Match State Listener Controller got joint " << joint_names[0]);
		handle_ = hw->getHandle(joint_names[0]);

		std::string topic;

		// get topic to subscribe to
		if (!n.getParam("topic", topic))
		{
			ROS_ERROR("Parameter 'topic' not set");
			return false;
		}

		sub_command_ = n.subscribe<frc_msgs::MatchSpecificData>(topic, 1, &MatchStateListenerController::commandCB, this);
		return true;
	}

	void starting(const ros::Time & /*time*/)
	{
	}
	void stopping(const ros::Time & /*time*/)
	{
	}

	void update(const ros::Time & /*time*/, const ros::Duration & /*period*/)
	{
		// Quick way to do a shallow copy of the entire HW state
		*(handle_.operator->()) = *command_buffer_.readFromRT();
	}


}; // class
} // namespace state_listener_controller

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(match_state_controller::MatchStateController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(state_listener_controller::MatchStateListenerController, controller_interface::ControllerBase)