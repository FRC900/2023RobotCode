#!/usr/bin/env python3
import rospy

from beginner_tutorials.srv import AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse

def handle_add_two_ints(req: AddTwoIntsRequest):
    print(f"Returning [{req.a} + {req.b} = {req.a + req.b}]")
    response = AddTwoIntsResponse()
    response.sum = req.a + req.b
    return response

rospy.init_node("add_two_ints_server", anonymous=True)
s = rospy.Service("/add_two_ints", AddTwoInts, handle_add_two_ints)
print("Ready to add two ints.")
rospy.spin()