#!/usr/bin/env python3
import rospy

from beginner_tutorials.srv import CalcTwoInts, CalcTwoIntsResponse

def handle_add_two_ints(req):
    print(f"Returning [{req.a} + {req.b} = {req.a + req.b}]")
    response = CalcTwoIntsResponse()
    response.calculate = req.a + req.b
    return response




rospy.init_node("add_two_ints_server", anonymous=True)
s = rospy.Service("add_two_ints", CalcTwoInts, handle_add_two_ints)
print("Ready to add two ints.")
rospy.spin()