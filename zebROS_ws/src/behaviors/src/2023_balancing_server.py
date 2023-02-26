#! /usr/bin/env python3

# balancing server, controls the entire balancing process and keeps track of the state and eventually leads to robot being balanced
# assumes that the robot is already aligned with the charging station
# ALSO assumes that negative is up 

# imports for actionlib server and messages
import rospy
import actionlib
import behavior_actions.msg
import sensor_msgs.msg
import std_msgs.msg
import math
import geometry_msgs.msg
from enum import Enum
from tf.transformations import euler_from_quaternion # may look like tf1 but is actually tf2
import std_srvs.srv

class States(Enum):
    # we only actually care about two states, the one with no wheels and all the rest as one state
    NO_WEELS_ON = 0 # starting state, nothing touching charging station
    ONE_WHEEL_ON_RAMP = 1 # when first wheel gets on to the ramp
    ONE_WHEEL_ON_CENTER = 2 # when that same wheel gets to the middle, should be able to call PID node from here
    TWO_WHEELS_ON_CENTER = 3 # when we have both wheels on the center
    # no states after this because if we fall off the other side we have done something very wrong and need to restart

def handle_param_load(name):
    try:
        res = rospy.get_param(name)
    except rospy.ROSException as e:
        rospy.logerr(f"Unable to load param with name={name}")
        return False
    return res

class AutoBalancing:
    # create messages that are used to publish feedback/result
    _feedback = behavior_actions.msg.Balancing2023Feedback()
    _result = behavior_actions.msg.Balancing2023Result()

    def __init__(self, name):
        self.debug = False
        self._action_name = name

        self._as = actionlib.SimpleActionServer(self._action_name, behavior_actions.msg.Balancing2023Action,
                                                execute_cb=self.balancing_callback, auto_start=False)
        self._as.start()

        res = handle_param_load("imu_sub_topic")
        imu_sub_topic = res if res else "/imu/zeroed_imu"
        rospy.loginfo(f"Using imu topic {imu_sub_topic}")
        
        self.balancer_client = actionlib.SimpleActionClient("balancer_server", behavior_actions.msg.Balancer2023Action)
        self.balancer_client.wait_for_server(rospy.Duration(5)) # 5 sec timeout
        #rospy.wait_for_service('/frcrobot_jetson/swerve_drive_controller/brake', rospy.Duration(5))
        self.brake_srv = rospy.ServiceProxy('/frcrobot_jetson/swerve_drive_controller/brake', std_srvs.srv.Empty)
        self.cmd_vel_topic = "/balance_position/balancer_server/swerve_drive_controller/cmd_vel"
        self.pub_cmd_vel = rospy.Publisher(self.cmd_vel_topic, geometry_msgs.msg.Twist, queue_size=1)
        self.derivative = -999
        self.angle_to_add = math.radians(20)
        self.PID_enabled = False
        self.current_pitch = -1000
        #self.old_pitch = -1000
        #self.old_pitch_time = -1000
        self.current_pitch_time = rospy.Time.now()
        self.state = States.NO_WEELS_ON
        self.sub_imu = rospy.Subscriber(imu_sub_topic, sensor_msgs.msg.Imu, self.imu_callback)
        self.derivative_pub = rospy.Publisher("derivative", std_msgs.msg.Float64, queue_size=0)
        self.down_measurements = 0
        self.threshold = math.radians(0.5) # how much a measurement must change by to be counted
        
        rospy.loginfo("Finished initalizing balancing server")

    def imu_callback(self, imu_msg):
        rospy.logdebug("Imu callback")
        q = imu_msg.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w]) 
        #roll = euler[0]
        pitch = euler[1]
        #yaw = euler[2]
        # self.old_pitch = self.current_pitch
        # self.old_pitch_time = self.current_pitch_time
        self.current_pitch = pitch
        self.current_pitch_time = imu_msg.header.stamp
        # (y2-y1)/(x2-x1)
        # print(f"Time delta {(self.current_pitch_time - self.old_pitch_time).to_sec()}")

        # self.derivative = (self.current_pitch - self.old_pitch) / (self.current_pitch_time - self.old_pitch_time).to_sec()
        # self.derivative_pub.publish(self.derivative)
        if self.debug:
            rospy.loginfo_throttle(1, f"Balancing server - Pitch in degrees {round(pitch*(180/math.pi), 4)}")

    def preempt(self):
        rospy.logwarn(f"Called preempt for balancING server")
        #self.balancer_client.cancel_all_goals()
        
        self.balancer_client.cancel_goal()
        self.state = States.NO_WEELS_ON
        #self.balancer_client.cancel_goals_at_and_before_time(rospy.Time.now())

    def balancing_callback(self, goal):
        rospy.loginfo(f"Auto Balancing Actionlib called with goal {goal}")
        # TODO, make sure we are aligned with 0 degrees (or 180 depending on how it works out)d
        
        # TODO, remove this when new method workds
        #goal_to_send = behavior_actions.msg.Balancer2023Goal()
        #goal_to_send.angle_offset = self.angle_to_add

        #self.balancer_client.send_goal(goal_to_send)
        
        # angle when we are on the ramp where balancer can take over = 0.259rad ~ 15 degrees
        # going to try and run pid to get to that and then run balancer? 
        r = rospy.Rate(100)
        start_time = rospy.Time.now()
        while True:                
            # I think subscribers should update without spinOnce... it doesn't exist in python
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested() or rospy.is_shutdown():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self.preempt() # stop pid
                self._as.set_preempted()
                return
            
            if self.state == States.NO_WEELS_ON:
                if rospy.Time.now() - start_time >= rospy.Duration(2):
                    rospy.logwarn("2 seconds of driving has not lead to a large enough angle increase, stopping balance!")
                    self.preempt()
                    self._as.set_preempted()
                    return
                rospy.loginfo_throttle(2, "Sending cmd of 1")
                cmd_vel_msg = geometry_msgs.msg.Twist()
                cmd_vel_msg.angular.x = 0 # todo, tune me
                cmd_vel_msg.angular.y = 0
                cmd_vel_msg.angular.z = 0
                cmd_vel_msg.linear.x = 1.5
                cmd_vel_msg.linear.y = 0
                cmd_vel_msg.linear.z = 0
                self.pub_cmd_vel.publish(cmd_vel_msg)

            if abs(self.current_pitch) >= math.radians(13) and self.state == States.NO_WEELS_ON:
                start_time = rospy.Time.now()
                rospy.logwarn("Recalled balancer with 0 offset=======================")
                while (rospy.Time.now() - start_time <= rospy.Duration(0.75)):
                    if self._as.is_preempt_requested():
                        rospy.loginfo('%s: Preempted' % self._action_name)
                        self.preempt() # stop pid
                        self._as.set_preempted()
                        break
                    cmd_vel_msg = geometry_msgs.msg.Twist()
                    cmd_vel_msg.angular.x = 0 # todo, tune me
                    cmd_vel_msg.angular.y = 0
                    cmd_vel_msg.angular.z = 0
                    cmd_vel_msg.linear.x = 1.5
                    cmd_vel_msg.linear.y = 0
                    cmd_vel_msg.linear.z = 0
                    self.pub_cmd_vel.publish(cmd_vel_msg)
                    r.sleep()
                r.sleep()
                self.state = States.ONE_WHEEL_ON_RAMP

            if self.state == States.ONE_WHEEL_ON_RAMP:
                rospy.loginfo("Running PID!")
                goal_to_send = behavior_actions.msg.Balancer2023Goal()
                goal_to_send.angle_offset = 0
                self.balancer_client.send_goal(goal_to_send)
                self.state = States.TWO_WHEELS_ON_CENTER
            
            # ramp is going down, we need to hard stop
            # normal extension for charging station is 11 degrees when fully pushed down
            ''' 
            if self.state == States.TWO_WHEELS_ON_CENTER and abs(self.current_pitch) <= math.radians(5):
                rospy.loginfo("Brake mode called!, Angle less than 7 degrees")
                try:
                    self.brake_srv()
                except Exception as e:
                    rospy.logerr("Could not call brake service in balancing!")
                    rospy.logerr("Stack trace\n" + str(e))
                
                start_time = rospy.Time.now()
                success = False
                r = rospy.Rate(100)
                while (rospy.Time.now() - start_time <= rospy.Duration(1)):
                    if self._as.is_preempt_requested():
                        rospy.loginfo('%s: Preempted' % self._action_name)
                        self.preempt() # stop pid
                        self._as.set_preempted()
                        return
                    r.sleep()

                if not success:
                    rospy.loginfo("Preempted while waiting for balance!")
                    return
                # run pid to finish off balancing, should already be balanced but if we are off a little this will fix it
                goal_to_send = behavior_actions.msg.Balancer2023Goal()
                goal_to_send.angle_offset = 0
                self.balancer_client.send_goal(goal_to_send)
                self.state = States.TWO_WHEELS_ON_CENTER
                rospy.loginfo("Finished balancing!")
            '''
            
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            r.sleep()

if __name__ == '__main__':
    rospy.logwarn("Initalizing balancing server")
    rospy.init_node('balancer')
    name = rospy.get_name()
    balancer_server = AutoBalancing(name)
    rospy.spin()

