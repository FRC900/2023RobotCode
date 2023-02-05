import rospy
import sensor_msgs.msg
from message_filters import ApproximateTimeSynchronizer, Subscriber as SyncSubscriber
import behavior_actions.msg
import math

def terabeeCB(left: sensor_msgs.msg.Range, right: sensor_msgs.msg.Range):
    if left.range == 0 or math.isinf(left.range): 
    #print(left)
    #print(right)
    pass

rospy.init_node("game_piece_state_2023")
left_intake_sub = SyncSubscriber("/frcrobot_jetson/intake_terabee_left_x", sensor_msgs.msg.Range)
right_intake_sub = SyncSubscriber("/frcrobot_jetson/intake_terabee_right_x", sensor_msgs.msg.Range)
left_intake_sub = SyncSubscriber("/terabee/terabee_x", sensor_msgs.msg.Range)
right_intake_sub = SyncSubscriber("/terabee/terabee_y", sensor_msgs.msg.Range)
game_piece_pub = rospy.Publisher("game_piece_state", behavior_actions.msg.GamePieceState2023)

ats = ApproximateTimeSynchronizer([left_intake_sub, right_intake_sub], queue_size=5, slop=0.1)
ats.registerCallback(terabeeCB)
rospy.spin()