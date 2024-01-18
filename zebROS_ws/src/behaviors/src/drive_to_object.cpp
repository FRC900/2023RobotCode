
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
  double y_error_{0};
  double angle_error_{0};

  double valid_frames_config_{4};
  double missed_frames_before_exit_{20};

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

public:

  AlignToObjectActionServer(std::string name) :
    as_(nh_, name, boost::bind(&AlignToObjectActionServer::executeCB, this, _1), false),
    action_name_(name),
    sub_(nh_.subscribe<field_obj::Detection>("/tf_object_detection/object_detection_world", 1, &AlignToObjectActionServer::callback, this)),
    ac_hold_position_("/hold_position/hold_position_server", true),
    tf_listener_(tf_buffer_),
    orientation_command_pub_(nh_.advertise<std_msgs::Float64>("/teleop/orientation_command", 1)),
    control_effort_sub_(nh_.subscribe<std_msgs::Float64>("/teleop/orient_strafing/control_effort", 1, &AlignToObjectActionServer::controlEffortCB, this))
  {
    const std::map<std::string, std::string> service_connection_header{ {"tcp_nodelay", "1"} };

    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/imu/zeroed_imu", 1, &AlignToObjectActionServer::imuCb, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/align/cmd_vel", 1, false);
    x_effort_sub_ = nh_.subscribe<std_msgs::Float64>("x_position_pid/x_command", 1, [&](const std_msgs::Float64ConstPtr &msg) {x_eff_ = msg->data;});
    y_effort_sub_ = nh_.subscribe<std_msgs::Float64>("y_position_pid/y_command", 1, [&](const std_msgs::Float64ConstPtr &msg) {y_eff_ = msg->data;});

    AlignActionAxisConfig x_axis("x", "x_position_pid/pid_enable", "x_position_pid/x_cmd_pub", "x_position_pid/x_state_pub", "x_position_pid/pid_debug", "x_timeout_param", "x_error_threshold_param");
	  AlignActionAxisConfig y_axis("y", "y_position_pid/pid_enable", "y_position_pid/y_cmd_pub", "y_position_pid/y_state_pub", "y_position_pid/pid_debug", "y_timeout_param", "y_error_threshold_param");
    if (!addAxis(x_axis))
    {
      ROS_ERROR_STREAM("Error adding x_axis to drive_to_object.");
      return ;
    }
    if (!addAxis(y_axis))
    {
      ROS_ERROR_STREAM("Error adding y_axis to drive_to_object.");
      return;
    }

    nh_.getParam("min_valid_frames", valid_frames_config_);
    nh_.getParam("missed_frames_allowed", missed_frames_before_exit_);

    as_.start();
  }

  ~AlignToObjectActionServer(void)
  {
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

  void executeCB(const behavior_actions::DriveToObjectGoalConstPtr &goal)
  {
    // just to make sure to go through the loop once
    x_error_ = 100;
    y_error_ = 100;
    angle_error_ = 100;
    ros::Rate r = ros::Rate(30);
    ROS_INFO_STREAM("Execute callback");
    auto x_axis_it = axis_states_.find("x");
    auto &x_axis = x_axis_it->second;
    auto y_axis_it = axis_states_.find("y");
    auto &y_axis = y_axis_it->second;

    x_axis.setEnable(true);
    y_axis.setEnable(true);
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

    geometry_msgs::Point offset; // should be goal->distance_away m from object in direction of robot
    // and this is when we get there, so that just means distance_away in the x direction
    offset.x = goal->distance_away;
    offset.y = 0;

    geometry_msgs::Point objectLocation = offset; // just to start

    int missed_frames = 0;
    std_msgs::Float64 msg;
    msg.data = latest_yaw_ + atan2(closestObject.location.y, closestObject.location.x);
    orientation_command_pub_.publish(msg);
    uint8_t valid_frames = 0;

    while (hypot(x_error_, y_error_) > goal->tolerance || valid_frames < valid_frames_config_) {
        ros::spinOnce(); // grab latest callback data
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_ERROR_STREAM("drive_to_object : Preempted");
            as_.setPreempted();
            x_axis.setEnable(false);
            y_axis.setEnable(false);
            return;
        }

        closestObject_ = findClosestObject(latest_, goal->id);
        if (closestObject_ == std::nullopt) {
            ROS_ERROR_STREAM("drive_to_object : Could not find object for this frame! :(");
            missed_frames++;
            if (missed_frames >= missed_frames_before_exit_) {
              ROS_ERROR_STREAM("Missed more than " << missed_frames_before_exit_ << " frames! Aborting");
              x_axis.setEnable(false);
              y_axis.setEnable(false);
              as_.setPreempted();
              return;
            }
            r.sleep();
            continue;
        }
        else {
          closestObject = closestObject_.value();
          x_axis.setEnable(true);
          y_axis.setEnable(true);
          missed_frames = 0;
        }

        msg.data = latest_yaw_ + atan2(closestObject.location.y, closestObject.location.x);
        orientation_command_pub_.publish(msg);

        geometry_msgs::PointStamped point;
        point.point = closestObject.location;
        point.header = latest_.header;
        geometry_msgs::PointStamped point_out;
        ROS_INFO_STREAM("Latest header before base_link = " << latest_.header);
        // no need for x to be field relative so get the x value before the transform to map
        tf_buffer_.transform(point, point_out, "base_link", ros::Duration(0.1));
        objectLocation.x = point_out.point.x;
        objectLocation.y = point_out.point.y;

        ROS_INFO_STREAM("Offset y = " << offset.y);

        x_axis.setState(-objectLocation.x);
        y_axis.setState(-objectLocation.y);

        x_axis.setCommand(-offset.x);
        y_axis.setCommand(-offset.y);

        ROS_INFO_STREAM("Y axis state = " << objectLocation.y << " command = " << offset.y << ", effort = " << y_eff_); 

        x_error_ = fabs(offset.x - objectLocation.x);
        y_error_ = fabs(offset.y - objectLocation.y);
        angle_error_ = fabs(atan2(closestObject.location.y, closestObject.location.x));

        ROS_INFO_STREAM("x tag = " << objectLocation.x << ", commanded x = " << offset.x << ", effort = " << x_eff_);
        geometry_msgs::Twist t;
        t.linear.x = x_eff_;
        t.linear.y = y_eff_;
        t.angular.z = orient_effort_;
        cmd_vel_pub_.publish(t);
        feedback_.x_error = x_error_;
        feedback_.y_error = y_error_;
        feedback_.angle_error = angle_error_;
        if (hypot(x_error_, y_error_) <= goal->tolerance && angle_error_ <= goal->tolerance) {
          valid_frames++;
        }
        as_.publishFeedback(feedback_);
        r.sleep();
        ROS_INFO_STREAM("offset = " << offset.x << " " << offset.y << " " << offset.z);
        ROS_INFO_STREAM("error = " << x_error_ << ", " << y_error_ << " = " << hypot(x_error_, y_error_));
    }
    x_axis.setEnable(false);
    y_axis.setEnable(false);
    result_.success = true;
    as_.setSucceeded(result_);
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "align_to_grid_closed_loop");

  AlignToObjectActionServer alignToGrid("align_to_grid_closed_loop");
  ros::spin();

  return 0;
}