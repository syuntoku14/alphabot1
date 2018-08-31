#!/usr/bin/env python

import AlphaBot1
import pigpio
import rospy
from std_msgs.msg import Int32MultiArray

pi = pigpio.pi()
wheel = AlphaBot1.WheelSetter(pi)
sensor = AlphaBot1.SensorGetter(pi)

wheel.set_wheel_speeds(0.6, 0.6)
rospy.init_node('encoder_publisher')
pub = rospy.Publisher('encoder', Int32MultiArray, queue_size=1)

rate = rospy.Rate(1000)
while not rospy.is_shutdown():
    left, right = sensor.ret_encoder_vals()
    pub.publish(Int32MultiArray(data=[left, right]))
    rate.sleep()
wheel.set_wheel_speeds(0, 0)
pi.stop()
