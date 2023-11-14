#!/usr/bin/env python3
import rospy
from beginner_tutorials.srv import MathTwoInts, MathTwoIntsResponse
from math import floor

rospy.init_node('service_calculator', anonymous=True)

def calculate(ints_req):
    response = MathTwoIntsResponse()
    if ints_req.op == 'add':
        response.d = ints_req.a + ints_req.b
    elif ints_req.op == 'subtract':
        response.d = ints_req.a - ints_req.b
    elif ints_req.op == 'multiply':
        response.d = ints_req.a * ints_req.b
    elif ints_req.op == 'divide':
        response.d = ints_req.a / ints_req.b
    elif ints_req.op == 'power':
        response.d = ints_req.a ** ints_req.b
    elif ints_req.op == 'mod':
        response.d = ints_req.a % ints_req.b
    else:
        return None
    return response

service = rospy.Service('/calculate', MathTwoInts, calculate)
rospy.spin()