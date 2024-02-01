#! /usr/bin/env python3

# import rospy
# import actionlib
# import actionlib_tutorials.msg
# import math
# import tf2_ros
# import geometry_msgs.msg

# class AlignToSpeaker(object):
#     # create messages that are used to publish feedback/result
#     _feedback = actionlib_tutorials.msg.FibonacciFeedback()
#     _result = actionlib_tutorials.msg.FibonacciResult()


#     tfBuffer = tf2_ros.Buffer()
#     listener = tf2_ros.TransformListener(tfBuffer)

#     rospy.wait_for_service('spawn')

#     rate = rospy.Rate(10.0)
#     while not rospy.is_shutdown():
#         try:
#             trans = tfBuffer.lookup_transform(turtle_name, 'turtle1', rospy.Time())
#         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#             rate.sleep()
#             continue


    

#     def __init__(self, name):
#         self._action_name = name
#         self._as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorials.msg.AlignToSpeaker, execute_cb=self.execute_cb, auto_start = False)
#         self._as.start()
      
#     def execute_cb(self, goal):
#         # helper variables
#         r = rospy.Rate(1)
#         success = True
        
#         # append the seeds for the fibonacci sequence
#         self._feedback.sequence = []
#         self._feedback.sequence.append(0)
#         self._feedback.sequence.append(1)
        
#         # publish info to the console for the user
#         rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))
        
#         # start executing the action
#         for i in range(1, goal.order):
#             # check that preempt has not been requested by the client
#             if self._as.is_preempt_requested():
#                 rospy.loginfo('%s: Preempted' % self._action_name)
#                 self._as.set_preempted()
#                 success = False
#                 break
#             self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
#             # publish the feedback
#             self._as.publish_feedback(self._feedback)
#             # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
#             r.sleep()
          
#         if success:
#             self._result.sequence = self._feedback.sequence
#             rospy.loginfo('%s: Succeeded' % self._action_name)
#             self._as.set_succeeded(self._result)
        
# if __name__ == '__main__':
#     rospy.init_node('fibonacci')
#     server = AlignToSpeaker(rospy.get_name())
#     rospy.spin()

import rospy

import math
import tf2_ros
import geometry_msgs.msg


if __name__ == '__main__':
    rospy.init_node('testing')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    


    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('base_link', 'speaker', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(e) 
            rate.sleep()
            continue

        msg = geometry_msgs.msg.Twist()

        print(math.atan2(trans.transform.translation.y, trans.transform.translation.x))
        msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
        rate.sleep()