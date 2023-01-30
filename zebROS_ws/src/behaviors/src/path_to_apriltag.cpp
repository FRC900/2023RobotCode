#include "ros/ros.h"
#include "behavior_actions/GamePiecePickup.h"
#include <nav_msgs/Path.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <path_follower_msgs/PathAction.h>
#include "base_trajectory_msgs/GenerateSpline.h"
#include <field_obj/Detection.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <optional>
#include "behavior_actions/PathToAprilTagAction.h"
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <actionlib/server/simple_action_server.h>

/*
"object_id: '1'
max_objects: 1
primary_frame_id: 'base_link'
secondary_object_id: ''
secondary_max_objects: 0
secondary_max_distance: 0.0
secondary_frame_id: ''
min_radius: 0.0
endpoint:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}"
*/

trajectory_msgs::JointTrajectoryPoint generateTrajectoryPoint(double x, double y, double rotation)
{
	trajectory_msgs::JointTrajectoryPoint point;
	point.positions.resize(3);
	point.positions[2] = rotation;
	point.positions[0] = x;
	point.positions[1] = y;
	return point;
}

double getYaw(const geometry_msgs::Quaternion &o) {
    tf2::Quaternion q;
    tf2::fromMsg(o, q);
    tf2::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);
    return y;
}

struct Point2D {
    double x;
    double y;
};

double angleBetween(Point2D one, Point2D two) {
    return atan2(two.y - one.y, two.x - one.x);
}

class PathToAprilTagAction
{
protected:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_params_;
    actionlib::SimpleActionServer<behavior_actions::PathToAprilTagAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    behavior_actions::PathToAprilTagFeedback feedback_;
    behavior_actions::PathToAprilTagResult result_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    double latestImuZ_;
    double timeout_;
    field_obj::Detection latest_;
    actionlib::SimpleActionClient<path_follower_msgs::PathAction> ac_;
    ros::ServiceClient client_;
    ros::Subscriber sub_;
    ros::Subscriber subImu_;

public:

    PathToAprilTagAction(std::string name) :
        nh_params_(nh_, "path_to_apriltag"),
        as_(nh_, name, boost::bind(&PathToAprilTagAction::executeCB, this, _1), false),
        action_name_(name),
        tf_listener_(tf_buffer_),
        ac_("/path_follower/path_follower_server", true),
        client_(nh_.serviceClient<base_trajectory_msgs::GenerateSpline>("/path_follower/base_trajectory/spline_gen")),
        sub_(nh_.subscribe<field_obj::Detection>("/tf_object_detection/tag_detection_world", 1, boost::bind(&PathToAprilTagAction::callback, this, _1))),
        subImu_(nh_.subscribe<sensor_msgs::Imu>("/imu/zeroed_imu", 1, boost::bind(&PathToAprilTagAction::imuCallback, this, _1))),
        timeout_(nh_params_.param<double>("timeout", timeout_, 0.2)) // roughly equal to apriltag detector fps
    {
        as_.start();
    }

    ~PathToAprilTagAction(void)
    {
    }

    std::optional<tf2::Transform> getTransformToTag(uint32_t id, double rotation) {
        if (latest_.objects.size() == 0 || ros::Time::now() - latest_.header.stamp > ros::Duration(timeout_)) {
            return std::nullopt;
        }
        else {
            for (auto object : latest_.objects) {
                if (std::stoi(object.id) == id) {
                    geometry_msgs::PointStamped point;
                    point.point = object.location;
                    point.header = latest_.header;
                    geometry_msgs::PointStamped point_out;
                    tf_buffer_.transform(point, point_out, "base_link");
                    tf2::Transform tf;
                    tf.setOrigin(tf2::Vector3(point_out.point.x, point_out.point.y, point_out.point.z));
                    tf2::Quaternion q;
                    q.setRPY(0, 0, (rotation - latestImuZ_));
                    tf.setRotation(q);
                    return tf;
                }
            }
            return std::nullopt;
        }
    }

    std::optional<trajectory_msgs::JointTrajectoryPoint> applyOffset(uint32_t id, const geometry_msgs::Pose &offset, double tagRotation) {
        std::optional<tf2::Transform> transform_ = getTransformToTag(id, tagRotation);

        if (!transform_.has_value()) {
            ROS_INFO_STREAM("transform no value");
            return std::nullopt;
        }

        tf2::Transform transform = transform_.value();

        geometry_msgs::Transform gmt = tf2::toMsg(transform); // now tag -> base_link
        geometry_msgs::TransformStamped gmts;
        gmts.header.frame_id = "tag";
        gmts.child_frame_id = "base_link";
        gmts.transform = gmt;

        // get offset, transform by inverse transform
        geometry_msgs::Pose p;
        tf2::doTransform(offset, p, gmts);
        trajectory_msgs::JointTrajectoryPoint pt = generateTrajectoryPoint(p.position.x, p.position.y, getYaw(p.orientation));
        return std::optional<trajectory_msgs::JointTrajectoryPoint>{pt};
    }


    void callback(const field_obj::DetectionConstPtr& msg) {
        // need to use transforms...
        // oh wait no we don't! we can tell spline srv what frame id it is relevant to
        latest_ = *msg;
    }

    void imuCallback(const sensor_msgs::ImuConstPtr& msg) {
        latestImuZ_ = getYaw(msg->orientation);
    }

    void executeCB(const behavior_actions::PathToAprilTagGoalConstPtr &goal)
    {
        uint32_t id = goal->id;
        double tagRot = goal->tagRotation;
        auto pt_ = applyOffset(id, goal->offset, tagRot);

        if (!pt_.has_value()) {
            as_.setAborted(result_);
            return;
        }
        auto pt = pt_.value();

        base_trajectory_msgs::GenerateSpline spline_gen_srv;

        spline_gen_srv.request.header = latest_.header; // timestamp
        spline_gen_srv.request.header.frame_id = "base_link"; // correct frame id

        // if there is an offset, get atan2(y, x), add angle between robot and apriltag

        spline_gen_srv.request.points.resize(2);
        spline_gen_srv.request.point_frame_id.resize(2);
        spline_gen_srv.request.points[0] = generateTrajectoryPoint(0, 0, 0);
        //ROS_INFO_STREAM(position.x << "," << position.y << " " << zRot << " " << latestImuZ << " " << zRot - latestImuZ);
        // angle now relative to apriltag
        spline_gen_srv.request.points[1] = pt;
        spline_gen_srv.request.point_frame_id[0] = "base_link";
        spline_gen_srv.request.point_frame_id[1] = goal->frame_id;
        base_trajectory_msgs::PathOffsetLimit path_offset_limit;
        spline_gen_srv.request.path_offset_limit.push_back(path_offset_limit);

        if (client_.call(spline_gen_srv))
        {
            ROS_INFO_STREAM("Called");
        }
        else
        {
            ROS_ERROR("Failed to call service");
            return;
        }

        nav_msgs::Path path = spline_gen_srv.response.path;

        ROS_INFO("Waiting for action server to start.");
        // wait for the action server to start
        ac_.waitForServer(); //will wait for infinite time

        ROS_INFO("Action server started, sending goal.");
        // send a goal to the action
        path_follower_msgs::PathGoal pathGoal;
        pathGoal.path = path;
        pathGoal.waypointsIdx = spline_gen_srv.response.waypointsIdx;
        pathGoal.waypoints = spline_gen_srv.response.waypoints;
        ac_.sendGoal(pathGoal);

        ros::Rate r(30);
        while (!ac_.getState().isDone()) {
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ac_.cancelGoalsAtAndBeforeTime(ros::Time::now());
                as_.setPreempted();
                return;
            }
            r.sleep();
        }

        //wait for the action to return
        as_.setSucceeded(result_);
    }


};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_to_apriltag");

    PathToAprilTagAction path_to_apriltag("path_to_apriltag");
    ros::spin();

    return 0;
}