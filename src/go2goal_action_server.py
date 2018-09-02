#!/usr/bin/env python

import AlphaBot1
import pigpio
import rospy
import tf
import actionlib
from alphabot1.msg import Go2GoalAction, Go2GoalGoal, Go2GoalResult

pi = pigpio.pi()
alphaBot1 = AlphaBot1.AlphaBot1(pi)
alphaBot1.set_go_to_goal(1.0, 2.0)
rospy.init_node('alphabot_tf_broadcaster')

rate = rospy.Rate(1000)
while not rospy.is_shutdown():
    if alphaBot1.is_at_goal():
        break
    br = tf.TransformBroadcaster()
    br.sendTransform((alphaBot1.x, alphaBot1.y, 0), 
            tf.transformations.quaternion_from_euler(0, 0, alphaBot1.theta),
            rospy.Time.now(), "alphabot", "world")
    alphaBot1.execute()
    rate.sleep()

alphaBot1.wheel.set_wheel_speeds(0, 0)
pi.stop()
