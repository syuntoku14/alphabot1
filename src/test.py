#!/usr/bin/env python

import AlphaBot1
import pigpio
import rospy
import tf

pi = pigpio.pi()
alphabot1 = AlphaBot1.AlphaBot1(pi)
alphabot1.set_go_to_goal(1.0, 2.0)
rospy.init_node('alphabot_tf_broadcaster')

rate = rospy.Rate(1000)
while not rospy.is_shutdown():
    if alphabot1.is_at_goal():
        break
    br = tf.TransformBroadcaster()
    br.sendTransform((alphabot1.x, alphabot1.y, 0), 
            tf.transformations.quaternion_from_euler(0, 0, alphabot1.theta),
            rospy.Time.now(), "alphabot", "world")
    alphabot1.execute()
    rate.sleep()

alphabot1.wheel.set_wheel_speeds(0, 0)
pi.stop()
