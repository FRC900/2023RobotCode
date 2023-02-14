#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <field_obj/Detection.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/Imu.h>
#include <boost/circular_buffer.hpp>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <angles/angles.h>

class SnapToConeCube2023
{
    protected:

        ros::NodeHandle nh_;
        ros::Subscriber object_detection_sub_;
        ros::Subscriber cmd_vel_sub_;
        ros::Publisher nearest_cone_pub_;
        ros::Publisher nearest_cube_pub_;
        double nearest_cargo_angle_;
        double nearest_opponent_cargo_angle_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        geometry_msgs::TwistStamped cmd_vel_out_;
        ros::Subscriber imu_sub_;
        boost::circular_buffer<std::pair<ros::Time, double>> imu_cb_;
        std::mutex buffer_mutex_;
        ddynamic_reconfigure::DDynamicReconfigure ddr_;
        double zed_time_offset_;


    public:

        SnapToConeCube2023() : tf_listener_(tf_buffer_), imu_cb_(200), zed_time_offset_(0.01)
        {

        }

        void init()
        {
            nearest_cone_pub_ = nh_.advertise<std_msgs::Float64>("nearest_cone_angle", 1);
            nearest_cube_pub_ = nh_.advertise<std_msgs::Float64>("nearest_cube_angle", 1);
            object_detection_sub_ = nh_.subscribe("/tf_object_detection/object_detection_world_filtered", 1, &SnapToConeCube2023::objectDetectionCallback, this);
            cmd_vel_sub_ = nh_.subscribe("/frcrobot_jetson/swerve_drive_controller/cmd_vel_out", 1, &SnapToConeCube2023::cmdVelSub, this);
            imu_sub_ = nh_.subscribe("/imu/zeroed_imu", 1, &SnapToConeCube2023::imuCallback, this);
            ROS_INFO_STREAM("snap_to_nearest_conecube_2023 : initialized");
            ddr_.registerVariable<double>("zed_time_offset", &zed_time_offset_, "", 0, 1);
            ddr_.publishServicesTopics();
        }

        void imuCallback(const sensor_msgs::Imu &imuState)
        {
            const tf2::Quaternion imuQuat(imuState.orientation.x, imuState.orientation.y, imuState.orientation.z, imuState.orientation.w);
            double roll;
            double pitch;
            double yaw;
            tf2::Matrix3x3(imuQuat).getRPY(roll, pitch, yaw);

            if (std::isfinite(yaw)) // ignore NaN results
            {
                std::unique_lock<std::mutex> lock_that_is_unique(buffer_mutex_);
                imu_cb_.push_back({imuState.header.stamp, -yaw});
            }
        }

        void cmdVelSub(const geometry_msgs::TwistStamped &msg) {
            cmd_vel_out_ = msg;
        }

        void objectDetectionCallback(const field_obj::Detection &msg)
        {
            double imu_angle = -1;
            ros::spinOnce();
            if (imu_cb_.size() != 0) {
                std::unique_lock<std::mutex> lock_that_is_unique(buffer_mutex_);
                for (boost::circular_buffer<std::pair<ros::Time, double>>::reverse_iterator it = imu_cb_.rbegin(); it != imu_cb_.rend(); it++) {
                    auto angle = *it;
                    if (angle.first <= msg.header.stamp - ros::Duration(zed_time_offset_)) {
                        imu_angle = angle.second;
                        break;
                    }
                }
            }
            if (imu_angle == -1) {
                ROS_ERROR_STREAM("IMU data too new");
                return;
            }
            field_obj::Object closest_cone;
            field_obj::Object closest_cube;
            double shortest_cone_distance = std::numeric_limits<double>::max();
            double shortest_cube_distance = std::numeric_limits<double>::max();
            for (auto obj : msg.objects)
            {
                if (obj.id == "cone")
                {
                    double dist = hypot(obj.location.x, obj.location.y);
                    if (dist < shortest_cone_distance)
                    {
                        closest_cone = obj;
                        shortest_cone_distance = dist;
                    }
                }
                else if (obj.id == "cube")
                {
                    double dist = hypot(obj.location.x, obj.location.y);
                    if (dist < shortest_cube_distance)
                    {
                        closest_cube = obj;
                        shortest_cube_distance = dist;
                    }
                }
            }
            if (shortest_cone_distance != std::numeric_limits<double>::max()) {
                std_msgs::Float64 msg1;
                try
                {
                    geometry_msgs::PoseStamped p1s;
                    p1s.header = msg.header;
                    p1s.pose.position = closest_cone.location;
                    tf2::Quaternion q1;
                    q1.setRPY(0, 0, angles::from_degrees(closest_cone.angle));
                    geometry_msgs::Quaternion q1m = tf2::toMsg(q1);
                    p1s.pose.orientation = q1m;
                    p1s = tf_buffer_.transform(p1s, "base_link", ros::Duration(0.05));
                    msg1.data = angles::shortest_angular_distance(imu_angle, atan2(p1s.pose.position.y, p1s.pose.position.x));
                }
                catch (...)
                {
                    ROS_WARN_STREAM_THROTTLE(0.1, "snap_to_nearest_conecube_2023 : transform to base_link failed, using untransformed angle");
                    msg1.data = angles::from_degrees(closest_cone.angle);
                }
                nearest_cone_pub_.publish(msg1);
            }

            if (shortest_cube_distance != std::numeric_limits<double>::max()) {
                std_msgs::Float64 msg2;
                try
                {
                    geometry_msgs::PoseStamped p2s;
                    p2s.header = msg.header;
                    p2s.pose.position = closest_cube.location;
                    tf2::Quaternion q2;
                    q2.setRPY(0, 0, angles::from_degrees(closest_cube.angle));
                    geometry_msgs::Quaternion q2m = tf2::toMsg(q2);
                    p2s.pose.orientation = q2m;
                    p2s = tf_buffer_.transform(p2s, "base_link", ros::Duration(0.05));
                    msg2.data = angles::shortest_angular_distance(imu_angle, atan2(p2s.pose.position.y, p2s.pose.position.x));
                }
                catch (...)
                {
                    ROS_WARN_STREAM_THROTTLE(0.1, "snap_to_nearest_conecube_2023 : transform to base_link failed, using untransformed angle");
                    msg2.data = angles::from_degrees(closest_cube.angle);
                }
                nearest_cube_pub_.publish(msg2);
            }
        }


};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "snap_to_nearest_conecube_2023");

    SnapToConeCube2023 conecube;
    conecube.init();
    ros::spin();

    return 0;
}
