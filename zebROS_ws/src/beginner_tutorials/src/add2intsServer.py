#!/usr/bin/env python3
import rospy

from beginner_tutorials.srv import add2ints, add2intsRequest, add2intsResponse

def handle_add_two_ints(req: add2intsRequest):
    if req.a == 9:
        if req.b == 10:
            print(f"Returning [{req.a} + {req.b} = 21]")
    elif req.a == 10:
        if req.b == 9:
            print(f"Returning [{req.a} + {req.b} = 21]")
    else:
        print(f"Returning [{req.a} + {req.b} = {req.a + req.b}]")
    response = add2intsResponse()
    if req.a == 9:
        if req.b == 10:
            response.sum = 21
            return response
    elif req.a == 10:
        if req.b == 9:
            response.sum = 21
            return response
    else:
        response.sum = req.a + req.b
        return response

rospy.init_node("add_2_ints_server", anonymous=True)
s = rospy.Service("/add_2_ints", add2ints, handle_add_two_ints)
print("Ready to add two ints.")
rospy.spin()