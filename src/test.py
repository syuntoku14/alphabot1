#!/usr/bin/env python

import AlphaBot1
import pigpio
import rospy
from std_msgs.msg import Float64MultiArray

pi = pigpio.pi()
alphabot1 = AlphaBot1.AlphaBot1(pi)

rospy.init_node('odometry_publisher')
pub = rospy.Publisher('odometry', Float64MultiArray, queue_size=1)

rate = rospy.Rate(1000)
while not rospy.is_shutdown():
    estimated_state = [alphabot1.x, alphabot1.y, alphabot1.theta]
    pub.publish(Float64MultiArray(data=estimated_state))
    alphabot1.execute()
    rate.sleep()

alphabot1.wheel.set_wheel_speeds(0, 0)
pi.stop()
