#!/usr/bin/env python3
import sys
import rospy
from beginner_tutorials.srv import AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse
def add_two_ints_client(a, b):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(a, b)
        return resp1.sum
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def usage():
    return f"{sys.argv[0]} [a b]"

if __name__ == "__main__":
    if len(sys.argv) == 3:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    rospy.init_node('add_two_ints_client', anonymous=True)
    rospy.loginfo(f"Requesting {a}+{b}")
    result = add_two_ints_client(a, b)
    print(f"{a} + {b} = {result}")