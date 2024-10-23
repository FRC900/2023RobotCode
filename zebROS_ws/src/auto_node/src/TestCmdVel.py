from auto_base import AutoBase
from series_action import SeriesAction
from wait_action import WaitAction
from drive_trajectory_iterator import DriveTrajectoryActionIterator
from parallel_action import ParallelAction
from intake_action import IntakeAction
from dynamic_shoot_action import DynamicShootAction
from geometry_msgs.msg import Twist
from cmd_vel_action import CmdVelAction

class TestCmdVel(AutoBase):
    def __init__(self) -> None:
        super().__init__(display_name="TestCmdVel", # must match choreo path name (if expected traj count is != 0)
                         expected_trajectory_count=0) # how many segments of the path there are (split at waypoints)

    def get_action(self) -> SeriesAction:
        twist_msg: Twist = Twist()
        twist_msg.linear.x = -2
        twist_msg.linear.y = -2
        reverse_twist_msg: Twist = Twist()
        reverse_twist_msg.linear.x = 2
        reverse_twist_msg.linear.y = 2      
        return SeriesAction([
            CmdVelAction(twist_msg, 2),
            WaitAction(2),
            CmdVelAction(reverse_twist_msg, 2)
        ])
