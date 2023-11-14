#!/usr/bin/env python3
import rospy

from beginner_tutorials.srv import calcInts, calcIntsRequest, calcIntsResponse

def calculateInts(req: calcIntsRequest):
    if (req.operation == "+") or ("add" in req.operation):
        print(f"The response will be [{req.a} + {req.b}]")
        response = calcIntsResponse()
        response.calculation = req.a + req.b
        return response
    elif (req.operation == "-") or ("sub" in req.operation):
        print(f"The response will be [{req.a} - {req.b}]")
        response = calcIntsResponse()
        response.calculation = req.a - req.b
        return response
    elif (req.operation == "*") or ("mul" in req.operation):
        print(f"The response will be [{req.a} * {req.b}]")
        response = calcIntsResponse()
        response.calculation = req.a * req.b
        return response
    elif (req.operation == "/") or ("div" in req.operation):
        print(f"The response will be [{req.a}/{req.b}]")
        response = calcIntsResponse()
        response.calculation = req.a / req.b
        return response
    elif (req.operation == "^") or ("pow" in req.operation):
        print(f"The response will be [{req.a}^{req.b}]")
        response = calcIntsResponse()
        response.calculation = req.a ** req.b
        return response
    elif (req.operation == "%") or ("rem" in req.operation):
        print(f"The response will be the remainder when you divide {req.a} and {req.b}")
        response = calcIntsResponse()
        response.calculation = req.a % req.b
        return response
    else:
        print("Don't be an idiot and fix your input. >:(")
    
rospy.init_node("calculate_server", anonymous=True)
s = rospy.Service("/calcInts", calcInts, calculateInts)
print("Ready to add calculate your numbers.")
rospy.spin()