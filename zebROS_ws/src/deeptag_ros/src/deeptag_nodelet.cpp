#include <optional>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "apriltag_msgs/ApriltagArrayStamped.h"

namespace deeptag_ros_nodelet
{
    class DeeptagRosNodelet : public nodelet::Nodelet
    {
    public:
        DeeptagRosNodelet(void) = default;

        ~DeeptagRosNodelet() override = default;

    protected:
        void onInit() override
        {
            nh_ = getMTPrivateNodeHandle();

            image_transport::ImageTransport it(nh_);
            camera_image_sub_ = std::make_unique<image_transport::Subscriber>(it.subscribe("image_rect", 1, &DeeptagRosNodelet::callback, this));
            camera_info_sub_ = nh_.subscribe("camera_info", sub_rate, &GoalDetect::camera_info_callback, this);
            pub_apriltag_detections = nh_.advertise<field_obj::Detection>("goal_detect_msg", pub_rate);
            pub_debug_image_ = it.advertise("debug_image", 2);
        }

    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_;
        ros::Subscriber camera_image_sub_;
        ros::Subscriber camera_info_sub_;
        image_transport::Publisher pub_debug_image_;
        ros::Publisher pub_apriltag_detections;_

        std::optional<sensor_msgs::CameraInfo> camera_info_{std::nullopt};
    }
}