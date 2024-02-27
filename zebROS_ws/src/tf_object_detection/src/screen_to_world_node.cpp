#include <ros/ros.h>
#include <nodelet/loader.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "field_obj/Detection.h"
#include "field_obj/TFDetection.h"
#include "field_obj_tracker/convert_coords.h"

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "depth_algorithms.h"

int main (int argc, char **argv) {
	ros::init(argc, argv, "screen_to_world_node"); // FIXME I don't know what '@(node)' should be replaced with
	nodelet::Loader nodelet;
	nodelet::M_string remap(ros::names::getRemappings());
	nodelet::V_string nargv;
	std::string nodelet_name = ros::this_node::getName();
	nodelet.load(nodelet_name, "tf_object_detection/ScreenToWorld", remap, nargv); // FIXME Is this the right Nodelet name?
	return 0;
}
