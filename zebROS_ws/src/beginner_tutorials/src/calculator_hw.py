#!/usr/bin/env python3
import rospy

from beginner_tutorials.msg import CalcTwoInts, CalcTwoIntsResponse


def handle_add_two_ints(req):
    if req.operation == ADD:
        print(f"Returning [{req.a} + {req.b} = {req.a + req.b}]")
        response = CalcTwoIntsResponse()
        response.sum = req.a + req.b
        return response
    if req.operation == SUB:
        print(f"Returning [{req.a} + {req.b} = {req.a - req.b}]")
        response = CalcTwoIntsResponse()
        response.div = req.a - req.b
        return response
    if req.operation == MUL:
        print(f"Returning [{req.a} + {req.b} = {req.a * req.b}]")
        response = CalcTwoIntsResponse()
        response.div = req.a * req.b
        return response
    if req.operation == DIV:
        print(f"Returning [{req.a} + {req.b} = {req.a / req.b}]")
        response = CalcTwoIntsResponse()
        response.div = req.a / req.b
        return response




rospy.init_node("add_two_ints_server", anonymous=True)
s = rospy.Service("add_two_ints", CalcTwoInts, handle_add_two_ints)
print("Ready to add two ints.")
rospy.spin()