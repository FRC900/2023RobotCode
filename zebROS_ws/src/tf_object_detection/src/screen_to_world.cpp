#include <ros/ros.h>
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
#include "nodelet/nodelet.h"

class ScreenToWorld : public nodelet::Nodelet {
public:
private:
	virtual void onInit() {
		ros::NodeHandle nh = getNodeHandle();
  		ros::NodeHandle private_nh = getPrivateNodeHandle();
		ros::NodeHandle nh_param("~");
		image_transport::ImageTransport it(nh);

		std::string algorithm_str;
		double timeout;

		nh_param.param<std::string>("depth_algorithm", algorithm_str, "CONTOURS");
		nh_param.param<double>("timeout", timeout, 0.05);

		ROS_INFO_STREAM("[screen_to_world] algorithm=" << algorithm_str << ", timeout=" << timeout);

		if (algorithm_str == "CONTOURS")
		{
			algorithm = CONTOURS;
		}
		else if (algorithm_str == "CONTOURS_NON_ADAPTIVE")
		{
			algorithm = CONTOURS_NON_ADAPTIVE;
		}
		else if (algorithm_str == "K_MEANS")
		{
			algorithm = K_MEANS;
		}
		else if (algorithm_str == "TWO_D_ONLY")
		{
			algorithm = TWO_D_ONLY;
		}
		else
		{
			ROS_ERROR_STREAM("Unknown depth algorithm type : " << algorithm_str);
			return -1;
		}

		// All algorithms require depth info, with the exception of TWO_D_ONLY
		// For the depth algorithms, create a ROS synchronizer to collect
		// image + depth into a single callback
		std::unique_ptr<image_transport::SubscriberFilter> depth_sub;
		std::unique_ptr<message_filters::Subscriber<field_obj::TFDetection>> obsub;
		typedef message_filters::sync_policies::ApproximateTime<field_obj::TFDetection, sensor_msgs::Image> ObjDepthSyncPolicy;
		std::unique_ptr<message_filters::Synchronizer<ObjDepthSyncPolicy>> obj_depth_sync;

		// For the 2-D only mode, just pass in a dummy image ptr - this should never
		// be used so hopefully just a default initialized sensor img will work as a placeholder
		ros::Subscriber sub;
		boost::shared_ptr<sensor_msgs::Image> dummy_image_ptr(new sensor_msgs::Image());

		cv::Mat img(cv::Size(720, 1280), CV_32FC1, cv::Scalar(0));

		std_msgs::Header header;		 // empty header
		header.seq = 0;					 // user defined counter
		header.stamp = ros::Time::now(); // time
		cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_32FC1, img);
		img_bridge.toImageMsg(*dummy_image_ptr); // from cv_bridge to sensor_msgs::Image

		if (algorithm != TWO_D_ONLY)
		{
			// Create a filter subscriber to camera depth info
			depth_sub = std::make_unique<image_transport::SubscriberFilter>(it, "/zed_objdetect/depth/depth_registered", 1);

			// And another filer subscriber to the TF object detection
			obsub = std::make_unique<message_filters::Subscriber<field_obj::TFDetection>>(nh, "obj_detection_msg", 1);

			// Create a synchronizer which combines the two - it calls the callback function when it matches
			// up a message from each subscriber which have the (approximately) same timestamps
			// TODO - try this with an exact synchronizer?
			obj_depth_sync = std::make_unique<message_filters::Synchronizer<ObjDepthSyncPolicy>>(ObjDepthSyncPolicy(10), *obsub, *depth_sub);

			obj_depth_sync->setMaxIntervalDuration(ros::Duration(timeout));
			obj_depth_sync->registerCallback(boost::bind(callback, _1, _2));
		}
		else
		{
			sub = nh.subscribe<field_obj::TFDetection>("obj_detection_msg", 1, boost::bind(callback, _1, dummy_image_ptr));
		}

		// Set up a simple subscriber to capture camera info
		ros::Subscriber camera_info_sub_ = nh.subscribe("/zed_objdetect/left/camera_info", 1, camera_info_callback);

		// And a publisher to published converted 3d coords
		pub = private_nh.advertise<field_obj::Detection>("object_detection_world", 1);
	}
}

