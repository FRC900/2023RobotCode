#!/usr/bin/env python3
import rospy
from beginner_tutorials.msg import ZebraData 

def callback(zebra_msg):
    print(f"the zebra has  {zebra_msg.num_stripes} stripes")
    if zebra_msg.has_horn:
        print("zebbby is a uncicorn")
    else:
        print("zeeby no zebracorn")

        print("the zebby is somewhere")

        print(f"Zeebys name isnt zeeby its {zebra_msg.name}")


sub = rospy.Subscriber("input", ZebraData, callback, queue_size=1)

rospy.init_node("derwin", anonymous=True)
rospy.spin()