#!/usr/bin/env python
"""
# goal
float64 goal_x
float64 goal_y
---
# result
float64 result_x
float64 result_y
float64 result_theta
---
# feedback
float64 estimated_x
float64 estimated_y
float64 estimated_theta
"""

import AlphaBot1
import pigpio
import rospy
import tf
import actionlib
from alphabot1.msg import Go2GoalAction, Go2GoalGoal, Go2GoalResult

pi = pigpio.pi()
alphabot1 = AlphaBot1.AlphaBot1(pi)
rospy.init_node('alphabot1_go2goal_server')
rate = rospy.Rate(1000)

def go2goal(goal):
    global alphabot1, rate
    alphabot1.set_go_to_goal(goal.goal_x, goal.goal_y)
    while not alphabot1.is_at_goal():
        br = tf.TransformBroadcaster()
        br.sendTransform((alphabot1.x, alphabot1.y, 0), 
                tf.transformations.quaternion_from_euler(0, 0, alphabot1.theta),
                rospy.Time.now(), "alphabot", "world")
        alphabot1.execute()
        rate.sleep()
    result = Go2GoalResult(result_x=alphabot1.x, result_y=alphabot1.y, result_theta=alphabot1.theta)
    server.set_succeeded(result, "Goal!")
    alphabot1.wheel.set_wheel_speeds(0, 0)

server = actionlib.SimpleActionServer('go2goal', Go2GoalAction, go2goal, False)
server.start()
rospy.spin()
