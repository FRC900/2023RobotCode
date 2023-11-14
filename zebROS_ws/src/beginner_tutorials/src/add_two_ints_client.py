#!/usr/bin/env python3
import rospy

from beginner_tutorials.srv import CalcTwoInts, CalcTwoIntsResponse, CalcTwoIntsRequest
rospy.init_node("add_two_ints_server", anonymous=True)

rospy.wait_for_service("/add_two_ints")

try:
    add_two_ints_client = rospy.ServiceProxy('/add_two_ints', CalcTwoInts)
    request = CalcTwoIntsRequest()
    request.a = int(input("What is the first number you want to add? "))
    request.b = int(input("What is the second number you want to add? "))
    
    response = add_two_ints_client.call(request)
    print(f"the sum is {response.calculate}")
    
except rospy.ServiceException as e:
    print(f"Service call failed: {e}")









