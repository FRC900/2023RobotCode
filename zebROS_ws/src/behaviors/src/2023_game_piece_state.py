import rospy
import sensor_msgs.msg
import frc_msgs.msg
import std_msgs.msg
from message_filters import ApproximateTimeSynchronizer, Subscriber as SyncSubscriber
import behavior_actions.msg
import math
from enum import Enum


class GamePieces(Enum):

    CUBE=0
    VERTICAL_CONE=1
    BASE_TOWARDS_US_CONE=2 
    BASE_AWAY_US_CONE=3
    NONE=4 # for sure no game piece
    UNKNOWN=5 # we have a game piece, but not sure what it is right now


class GamePieceState:

    def __init__(self) -> None:
        self.state = GamePieces.NONE # not actually true, we will start with one, something in auto will update this
        self.offset_from_center = math.inf

        # CHECK TOPICS
        self.button_box_sub = rospy.Subscriber("/frcrobot_jetson/button_box_state_controller", frc_msgs.msg.ButtonBoxState, queue_size=1)
        # for real robot
        self.left_intake_sub = SyncSubscriber("/frcrobot_jetson/intake_terabee_left_x", sensor_msgs.msg.Range)
        self.right_intake_sub = SyncSubscriber("/frcrobot_jetson/intake_terabee_right_x", sensor_msgs.msg.Range)
        # for sim
        self.left_intake_sub = SyncSubscriber("/terabee/terabee_x", sensor_msgs.msg.Range)
        self.right_intake_sub = SyncSubscriber("/terabee/terabee_y", sensor_msgs.msg.Range)
        ats = ApproximateTimeSynchronizer([self.left_intake_sub, self.right_intake_sub], queue_size=1, slop=0.1)
        ats.registerCallback(self.terabeeCB)

        self.game_piece_pub = rospy.Publisher("game_piece_state", behavior_actions.msg.GamePieceState2023, queue_size=1)
        self.game_piece_offset_pub = rospy.Publisher("game_piece_center_offset", std_msgs.msg.Float64, queue_size=1) 

    def publish_states(self):
        fmsg = std_msgs.msg.Float64()
        fmsg.data = self.offset_from_center
        self.game_piece_offset_pub.publish(fmsg)

        gpmsg = behavior_actions.msg.GamePieceState2023()
        self.game_piece_pub()

    def terabeeCB(self, left: sensor_msgs.msg.Range, right: sensor_msgs.msg.Range):
        rospy.loginfo("Terabee callback in 2023 game piece state")
        for msg in (left, right):
            if msg.range == 0 or not math.isfinite(msg.range):
                self.offset_from_center = math.inf
                self.state = GamePieces.NONE
                self.publish_states()
                self.game_piece_offset_pub.publish(self.offset_from_center)
                return
            
        # right x is positive, left is negative
        self.offset_from_center = left.range - right.range
        self.game_piece_offset_pub.publish(self.offset_from_center)

    def buttonBoxCB(self, button_box):
        
    
if __name__ == "__main__":
    print("started")
    rospy.init_node("game_piece_state_2023")
    rospy.loginfo("Starting 2023 game piece state server!")
    game_piece_state = GamePieceState()
    r = rospy.Rate(10)
    msg = std_msgs.msg.Float64()
    msg.data = math.inf 
    while True:
        game_piece_state.game_piece_offset_pub.publish(msg)
        r.sleep()
    rospy.spin()
    rospy.loginfo("Successfully started: 2023 game piece state server!")
