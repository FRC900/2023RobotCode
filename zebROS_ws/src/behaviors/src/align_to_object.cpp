// This node autonomously targets and drives to a configurable distance away from an object detected using vision.
// It continuously checks where the object is to update its target.
#include "ros/ros.h"
#include "field_obj/Detection.h"
#include "field_obj/Object.h"
#include "geometry_msgs/Point.h"
#include <actionlib/server/simple_action_server.h>
#include "behavior_actions/DriveToObjectAction.h"
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <path_follower_msgs/holdPositionAction.h>
#include <sensor_msgs/Imu.h>
#include <frc_msgs/MatchSpecificData.h>
#include <behavior_actions/GamePieceState2023.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include <std_srvs/Empty.h>

class AlignActionAxisConfig
{
	public:
		AlignActionAxisConfig(const std::string &name,
							  const std::string &enable_pub_topic,
							  const std::string &command_pub_topic,
							  const std::string &state_pub_topic,
							  const std::string &error_sub_topic,
							  const std::string &timeout_param,
							  const std::string &error_threshold_param)
			: name_(name)
			, enable_pub_topic_(enable_pub_topic)
			, command_pub_topic_(command_pub_topic)
			, state_pub_topic_(state_pub_topic)
			, error_sub_topic_(error_sub_topic)
			, timeout_param_(timeout_param)
			, error_threshold_param_(error_threshold_param)
		{
		}
		std::string name_;
		std::string enable_pub_topic_;
		std::string command_pub_topic_;
		std::string state_pub_topic_;
		std::string error_sub_topic_;
		std::string timeout_param_;
		std::string error_threshold_param_;
};

class AlignActionAxisState
{
	public:
		AlignActionAxisState(ros::NodeHandle &nh,
							 const std::string &enable_pub_topic,
							 const std::string &command_pub_topic,
							 const std::string &state_pub_topic)
			: enable_pub_(nh.advertise<std_msgs::Bool>(enable_pub_topic, 1, true))
			, command_pub_(nh.advertise<std_msgs::Float64>(command_pub_topic, 1, true))
			, state_pub_(nh.advertise<std_msgs::Float64>(state_pub_topic, 1, true))
		{
			// Set defaults for PID node topics to prevent
			// spam of "Waiting for first setpoint message."
			std_msgs::Bool bool_msg;
			bool_msg.data = false;
			enable_pub_.publish(bool_msg);

			std_msgs::Float64 float64_msg;
			float64_msg.data = 0.0;
			command_pub_.publish(float64_msg);
			state_pub_.publish(float64_msg);
		}
		void setEnable(bool enable_state)
		{
			std_msgs::Bool enable_msg;
			enable_msg.data = enable_state;
			enable_pub_.publish(enable_msg);
		}
		void setCommand(double command)
		{
			std_msgs::Float64 command_msg;
			command_msg.data = command;
			command_pub_.publish(command_msg);
		}
		void setState(double state)
		{
			std_msgs::Float64 state_msg;
			state_msg.data = state;
			state_pub_.publish(state_msg);
		}
	private:
		ros::Publisher enable_pub_;
		ros::Publisher command_pub_;
		ros::Publisher state_pub_;
};


geometry_msgs::Point operator-(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs) {
    geometry_msgs::Point p;
    p.x = lhs.x - rhs.x;
    p.y = lhs.y - rhs.y;
    p.z = lhs.z - rhs.z;
    return p;
}

geometry_msgs::Point operator+(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs) {
    geometry_msgs::Point p;
    p.x = lhs.x + rhs.x;
    p.y = lhs.y + rhs.y;
    p.z = lhs.z + rhs.z;
    return p;
}

double getYaw(const geometry_msgs::Quaternion &o) {
    tf2::Quaternion q;
    tf2::fromMsg(o, q);
    tf2::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);
    return y;
}

double distance(const geometry_msgs::Point &pt) {
    return hypot(pt.x, pt.y, pt.z);
}

bool operator<(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs) {
    return distance(lhs) < distance(rhs);
}

bool operator>(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs) {
    return distance(lhs) > distance(rhs);
}

class AlignToObjectActionServer
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<behavior_actions::DriveToObjectAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  behavior_actions::DriveToObjectFeedback feedback_;
  behavior_actions::DriveToObjectResult result_;
  field_obj::Detection latest_;
  ros::Subscriber sub_;
  ros::Subscriber imu_sub_;
  actionlib::SimpleActionClient<path_follower_msgs::holdPositionAction> ac_hold_position_;
  double xOffset_;
  double holdPosTimeout_;
  double latest_yaw_{0};
  double orient_effort_{0};

  double x_error_{0};
  double angle_error_{0};

  double valid_frames_config_{4};
  double missed_frames_before_exit_{20};
  double velocity_ramp_time_{0.5}; // time to transition from current command velocity to full PID control

  std::map<std::string, AlignActionAxisState> axis_states_;

  ros::Publisher cmd_vel_pub_;
  ros::Subscriber x_effort_sub_;
  double x_eff_;
  ros::Subscriber y_effort_sub_;
  double y_eff_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Publisher orientation_command_pub_;
  ros::Subscriber control_effort_sub_;

  ros::Subscriber cmd_vel_sub_;
  geometry_msgs::TwistStamped latest_cmd_vel_;

public:

  AlignToObjectActionServer(std::string name) :
    as_(nh_, name, boost::bind(&AlignToObjectActionServer::executeCB, this, _1), false),
    action_name_(name),
    sub_(nh_.subscribe<field_obj::Detection>("/tf_object_detection/object_detection_world", 1, &AlignToObjectActionServer::callback, this)),
    ac_hold_position_("/hold_position/hold_position_server", true),
    tf_listener_(tf_buffer_),
    orientation_command_pub_(nh_.advertise<std_msgs::Float64>("/teleop/orientation_command", 1)),
    control_effort_sub_(nh_.subscribe<std_msgs::Float64>("/teleop/orient_strafing/control_effort", 1, &AlignToObjectActionServer::controlEffortCB, this)),
    cmd_vel_sub_(nh_.subscribe<geometry_msgs::TwistStamped>("/frcrobot_jetson/swerve_drive_controller/cmd_vel_out", 1, &AlignToObjectActionServer::cmdVelCb, this))
  {
    const std::map<std::string, std::string> service_connection_header{ {"tcp_nodelay", "1"} };

    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/imu/zeroed_imu", 1, &AlignToObjectActionServer::imuCb, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/align/cmd_vel", 1, false);
    x_effort_sub_ = nh_.subscribe<std_msgs::Float64>("x_position_pid/x_command", 1, [&](const std_msgs::Float64ConstPtr &msg) {x_eff_ = msg->data;});

    AlignActionAxisConfig x_axis("x", "x_position_pid/pid_enable", "x_position_pid/x_cmd_pub", "x_position_pid/x_state_pub", "x_position_pid/pid_debug", "x_timeout_param", "x_error_threshold_param");
    if (!addAxis(x_axis))
    {
      ROS_ERROR_STREAM("Error adding x_axis to align_to_object.");
      return ;
    }

    nh_.getParam("min_valid_frames", valid_frames_config_);
    nh_.getParam("velocity_ramp_time", velocity_ramp_time_);
    nh_.getParam("missed_frames_allowed", missed_frames_before_exit_);

    // geometry_msgs::Twist t1;
    // t1.linear.x = 1.0;
    // geometry_msgs::Twist t2;
    // t2.linear.x = -0.1;
    // t2.linear.y = 1.0;

    // std::vector<std::pair<geometry_msgs::Twist, double>> pairs = {{t1, 0.8}, {t2, 0.2}};

    // ROS_INFO_STREAM(weightedAverageCmdVel(pairs));

    as_.start();
  }

  ~AlignToObjectActionServer(void)
  {
  }

  void cmdVelCb(const geometry_msgs::TwistStampedConstPtr &msg) {
    latest_cmd_vel_ = *msg;
  }

  void controlEffortCB(const std_msgs::Float64ConstPtr& msg) {
    orient_effort_ = msg->data;
  }

  void imuCb(const sensor_msgs::ImuConstPtr &msg) {
    latest_yaw_ = getYaw(msg->orientation);
  }

  void callback(const field_obj::DetectionConstPtr& msg) {
    latest_ = *msg;
  }

  bool addAxis(const AlignActionAxisConfig &axis_config)
  {
    axis_states_.emplace(std::make_pair(axis_config.name_,
                      AlignActionAxisState(nh_,
                          axis_config.enable_pub_topic_,
                          axis_config.command_pub_topic_,
                          axis_config.state_pub_topic_)));
    return true;
  }

  std::optional<field_obj::Object> findClosestObject(const field_obj::Detection &detection, const std::string &object_id) {
    double minDistance = std::numeric_limits<double>::max();
    std::optional<field_obj::Object> closestObject = std::nullopt; 
    if (detection.objects.size() == 0) {
      return std::nullopt;
    }
    for (field_obj::Object obj : detection.objects) {
      double d = distance(obj.location);
      if (d < minDistance && obj.id == object_id) {
        minDistance = d;
        closestObject = obj;
      }
    }
    return closestObject;
  }

  geometry_msgs::Twist weightedAverageCmdVel(const std::vector<std::pair<geometry_msgs::Twist, double>> &entries) {
    // takes a vector of <cmd_vel, weight> pairs and computes their weighted average
    // weights must all add to one
    std::vector<geometry_msgs::Twist> weightedTwists;
    std::transform(entries.begin(), entries.end(), std::back_inserter(weightedTwists), [](std::pair<geometry_msgs::Twist, double> entry){
      geometry_msgs::Twist t = entry.first;
      t.linear.x *= entry.second;
      t.linear.y *= entry.second;
      t.angular.z *= entry.second;
      return t;
    });
    geometry_msgs::Twist finalTwist;
    std::for_each(weightedTwists.begin(), weightedTwists.end(), [&](geometry_msgs::Twist t){
      finalTwist.linear.x += t.linear.x;
      finalTwist.linear.y += t.linear.y;
      finalTwist.angular.z += t.angular.z;
    });
    return finalTwist;
  }

  void executeCB(const behavior_actions::DriveToObjectGoalConstPtr &goal)
  {
    // just to make sure to go through the loop once
    x_error_ = 100;
    angle_error_ = 100;
    ros::Rate r = ros::Rate(250);
    ROS_INFO_STREAM("Execute callback");
    auto x_axis_it = axis_states_.find("x");
    auto &x_axis = x_axis_it->second;

    x_axis.setEnable(true);
    std::optional<field_obj::Object> closestObject_ = findClosestObject(latest_, goal->id);
    field_obj::Object closestObject;
    if (closestObject_ == std::nullopt) {
      closestObject = field_obj::Object();
      closestObject.location.x = 0;
      closestObject.location.y = 0;
      closestObject.location.z = 0;
      ROS_INFO_STREAM("No initial object found!");
    } else {
      closestObject = closestObject_.value();
    }

    double field_relative_object_angle = latest_yaw_ + atan2(closestObject.location.y, closestObject.location.x);

    geometry_msgs::Point objectLocation;

    int missed_frames = 0;
    std_msgs::Float64 msg;
    msg.data = field_relative_object_angle;
    orientation_command_pub_.publish(msg);
    uint8_t valid_frames = 0;

    geometry_msgs::Twist initial_cmd_vel = latest_cmd_vel_.twist;
    ROS_INFO_STREAM("Initial cmd vel is " << initial_cmd_vel);
    ros::Time start = ros::Time::now();

    while (abs(x_error_) > goal->tolerance || valid_frames < valid_frames_config_) {
        ros::spinOnce(); // grab latest callback data
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_ERROR_STREAM("align_to_object : Preempted");
            as_.setPreempted();
            x_axis.setEnable(false);
            return;
        }

        closestObject_ = findClosestObject(latest_, goal->id);
        if (closestObject_ == std::nullopt) {
            ROS_ERROR_STREAM("align_to_object : Could not find object for this frame! :(");
            missed_frames++;
            if (missed_frames >= missed_frames_before_exit_) {
              ROS_ERROR_STREAM("Missed more than " << missed_frames_before_exit_ << " frames! Aborting");
              x_axis.setEnable(false);
              as_.setPreempted();
              return;
            }
            r.sleep();
            continue;
        }
        else {
          closestObject = closestObject_.value();
          x_axis.setEnable(true);
          missed_frames = 0;
        }
        field_relative_object_angle = latest_yaw_ + atan2(closestObject.location.y, closestObject.location.x);

        msg.data = field_relative_object_angle;
        orientation_command_pub_.publish(msg);

        geometry_msgs::PointStamped point;
        point.point = closestObject.location;
        point.header = latest_.header;
        geometry_msgs::PointStamped point_out;
        // no need for x to be field relative so get the x value before the transform to map
        tf_buffer_.transform(point, point_out, "base_link", ros::Duration(0.1));
        objectLocation.x = point_out.point.x;
        objectLocation.y = point_out.point.y;

        x_axis.setState(-objectLocation.x);

        x_axis.setCommand(-goal->distance_away);

        x_error_ = fabs(goal->distance_away - objectLocation.x);
        angle_error_ = fabs(atan2(closestObject.location.y, closestObject.location.x));

        // note: need to account for spinning, because that changes the direction we're pointing --> changes what command we need to send.
        // could try to do the math, but i'm not sure how we'd calculate that.
        // also, we'd probably need to control linear acceleration (since linear velocity has to dynamically change :/)
        // seems like the correct way to do it is just based on how far we've turned since the last vision update:

        geometry_msgs::Twist pid_twist;
        // t.linear.x = x_eff_ - (ros::Time::now() - latest_.header.stamp).toSec() * std::cos(orient_effort_);
        // t.linear.y = y_eff_ - (ros::Time::now() - latest_.header.stamp).toSec() * std::sin(orient_effort_);
        pid_twist.linear.x = x_eff_ * cos(field_relative_object_angle - latest_yaw_);
				pid_twist.linear.y = x_eff_ * sin(field_relative_object_angle - latest_yaw_);
        pid_twist.angular.z = orient_effort_;

        double timeDelta = (ros::Time::now() - start).toSec();
        double initialVelocityInfluence = timeDelta > velocity_ramp_time_ ? 0.0 : ((velocity_ramp_time_ - timeDelta) / velocity_ramp_time_);

        std::vector<std::pair<geometry_msgs::Twist, double>> pairs;
        pairs.push_back({pid_twist, 1.0 - initialVelocityInfluence});
        pairs.push_back({initial_cmd_vel, initialVelocityInfluence});
        geometry_msgs::Twist t = weightedAverageCmdVel(pairs);

        cmd_vel_pub_.publish(t);
        feedback_.x_error = x_error_;
        feedback_.angle_error = angle_error_;
        if (abs(x_error_) <= goal->tolerance && angle_error_ <= goal->tolerance) {
          valid_frames++;
        }
        as_.publishFeedback(feedback_);
        r.sleep();
    }
    x_axis.setEnable(false);
    result_.success = true;
    as_.setSucceeded(result_);
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "align_to_object");

  AlignToObjectActionServer alignToObject("align_to_object");
  ros::spin();

  return 0;
}