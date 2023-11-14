#!/usr/bin/env python3
import rospy
from beginner_tutorials.srv import AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse

rospy.init_node("add_two_ints_client", anonymous=True)

rospy.wait_for_service("/add_two_ints")

try:
    add_two_ints_client = rospy.ServiceProxy("/add_two_ints", AddTwoInts)
    request = AddTwoIntsRequest()
    request.a = int(input("What is the first number we want to add? "))
    request.b = int(input("What is the second number we want to add? "))

    response = add_two_ints_client.call(request)

    print(f"The sum is {response.sum}")

except rospy.ServiceException as e:
    print(f"Service call failed: {e}")
except rospy.ROSSerializationException as e:
     print(f"Serialization failed: {e}")
except ValueError as e:
    print(f"According to Shaurya, you're an idiot, try again: {e}")
except BaseException as e:
    print(f"Something else failed: {e}")