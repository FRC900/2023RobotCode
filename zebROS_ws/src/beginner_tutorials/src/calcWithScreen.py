#!/usr/bin/env python3

#import pygame (screen), ros (calculator itself)
import pygame
import rospy
from beginner_tutorials.srv import calcInts, calcIntsRequest, calcIntsResponse

pygame.init()


red = pygame.Color(255, 0, 0)
screen = pygame.display.set_mode((1280, 720))
screen.fill(red)
clock = pygame.time.Clock()
running = True

while running:
    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        print(event)
        if event.type == pygame.QUIT:
            running = False
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
    
    screen.fill(red)
    pygame.display.flip()
    pygame.Rect(10, 10, 100, 100)
    clock.tick(60)  # limits FPS to 60



pygame.quit()

