#!/usr/bin/env python3
import rospy
from beginner_tutorials.srv import MathTwoInts, MathTwoIntsResponse

a = float(input('Num 1: '))
op = input('Operation (add/subtract/multiply/divide): ')
b = float(input('Num 2: '))

try:
    calculator = rospy.ServiceProxy('/calculate', MathTwoInts)
    print(f'Result: {calculator(a, b, op)}')
except rospy.ServiceException as e:
    print(f'Service call failed: {e}')