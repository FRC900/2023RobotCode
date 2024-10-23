
# should be __init__.py in autos directory
from enum import Enum
from auto_base import AutoBase
# import all autos here
from Test4Note import Test4Note
from TestCmdVel import TestCmdVel
from TestCmdVelCircle import TestCmdVelCircle

class AutonomousNames(str, Enum):
    Test4Note = "2025_4_Note"
    TestCmdVel = "2025_cmd_vel_test"
    TestCmdVelCircle = "2024_cmd_vel_circle_test"

    def __str__(self) -> str:
        return str.__str__(self)


def init_auto_selection_map() -> dict[AutonomousNames, AutoBase]:
    """
    Returns an autonomous selection map, mapping auto names to auto programs.
    """
    return {
        AutonomousNames.TestCmdVel: TestCmdVel(),
        AutonomousNames.Test4Note: Test4Note(),
        AutonomousNames.TestCmdVelCircle: TestCmdVelCircle()
    }
