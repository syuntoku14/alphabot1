#!/usr/bin/env python

import rospy
import actionlib
from alphabot1.msg import Go2GoalAction, Go2GoalGoal, Go2GoalResult

rospy.init_node('alphabot1_go2goal_client')
client = actionlib.SimpleActionClient('go2goal', Go2GoalAction)
client.wait_for_server()
goal = Go2GoalGoal(goal_x=2.0, goal_y=1.0)
client.send_goal(goal)
client.wait_for_result()
print('goal at')
print('x:{}, y:{}'.format(client.get_result().result_x, client.get_result().result_y))
