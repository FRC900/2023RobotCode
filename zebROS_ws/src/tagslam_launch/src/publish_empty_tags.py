# write a node to publish empty tag detections
import rospy
from apriltag_msgs.msg import ApriltagArrayStamped

if __name__ == '__main__':
    rospy.init_node('publish_empty_tags')
    pub = rospy.Publisher('/detector/tags', ApriltagArrayStamped, queue_size=10)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        msg = ApriltagArrayStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'zed_objdetect_left_camera_optical_frame'
        pub.publish(msg)
        rate.sleep()