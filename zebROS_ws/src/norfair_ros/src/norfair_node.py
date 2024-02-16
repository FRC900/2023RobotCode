#! /usr/bin/env python3
import numpy as np
import rospy
from norfair import Detection, Tracker
from norfair_ros.msg import Detection as DetectionMsg
from norfair_ros.msg import Detections as DetectionsMsg
from norfair_ros.msg import Point


class NorfairNode:
    def publisher(self, tracked_objects: list):
        """
        Tracked objects to ROS message.

        Parameters
        ----------
        tracked_objects : list
            List of tracked objects.
        """
        detection_msg = DetectionsMsg()
        detection_msg.detections = []

        for tracked_object in tracked_objects:
            detection_msg.detections.append(
                DetectionMsg(
                    id=tracked_object.id,
                    label=tracked_object.last_detection.label,
                    scores=[score for score in tracked_object.last_detection.scores],
                    points=[Point(point=point) for point in tracked_object.last_detection.points],
                )
            )
        detection_msg.header.stamp = rospy.get_rostime()
        
        self.pub.publish(detection_msg)

    def pipeline(self, bbox: DetectionsMsg):
        """
        Generate Norfair detections and pass them to the tracker.

        Parameters
        ----------
        bbox : DetectionsMsg
            DetectionsMsg message from converter.
        """
        detections = []
        for detection in bbox.detections:
            detections.append(
                Detection(
                    points=np.array([point.point for point in detection.points]),
                    scores=np.array(detection.scores),
                    label=detection.label,
                )
            )
        tracked_objects = self.tracker.update(detections)
        
        self.publisher(tracked_objects)

    def main(self):
        """
        Norfair initialization and subscriber and publisher definition.
        """
        rospy.init_node("norfair_ros")

        # Load parameters
        publishers = rospy.get_param("norfair_publishers")
        subscribers = rospy.get_param("norfair_subscribers")
        norfair_setup = rospy.get_param("norfair_setup")
        converter = subscribers["converter"]
        norfair_detections = publishers["detections"]

        # Norfair tracker initialization
        self.tracker = Tracker(
            distance_function=norfair_setup["distance_function"],
            distance_threshold=norfair_setup["distance_threshold"],
            hit_counter_max=norfair_setup["hit_counter_max"],
            initialization_delay=norfair_setup["initialization_delay"],
            pointwise_hit_counter_max=norfair_setup["pointwise_hit_counter_max"],
            detection_threshold=norfair_setup["detection_threshold"],
            past_detections_length=norfair_setup["past_detections_length"],
        )
        self.tracked_objects = []

        # ROS subscriber and publisher definition
        self.pub = rospy.Publisher(
            norfair_detections["topic"], DetectionsMsg, queue_size=norfair_detections["queue_size"]
        )
        rospy.Subscriber(converter["topic"], DetectionsMsg, self.pipeline)

        rospy.spin()


if __name__ == "__main__":
    try:
        NorfairNode().main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Norfair node terminated.")
