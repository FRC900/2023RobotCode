#!/usr/bin/env python3
import rospy
from beginner_tutorials.srv import add2ints, add2intsRequest, add2intsResponse

rospy.init_node("add_2_ints_client", anonymous=True)

rospy.wait_for_service("/add_2_ints")

try:
    add_two_ints_client = rospy.ServiceProxy("/add_2_ints", add2ints)
    request = add2intsRequest()
    request.a = int(input("What is the first number we want to add? "))
    request.b = int(input("What is the second number we want to add? "))

    response = add_two_ints_client.call(request)
    
    print(f"The sum is {response.sum}")

except rospy.ServiceException as e:
    print(f"Service call failed: {e}")