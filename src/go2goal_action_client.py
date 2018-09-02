#!/usr/bin/env python

import rospy
import actionlib
import sys
from alphabot1.msg import Go2GoalAction, Go2GoalGoal, Go2GoalResult

argv = sys.argv
try:
    goal_x = float(argv[1])
    goal_y = float(argv[2])
except:
    print("goal's type isn't float")
    quit()

rospy.init_node('alphabot1_go2goal_client')
client = actionlib.SimpleActionClient('go2goal', Go2GoalAction)
client.wait_for_server()
goal = Go2GoalGoal(goal_x=goal_x, goal_y=goal_y)
client.send_goal(goal)
client.wait_for_result()
print('goal at')
print('x:{}, y:{}'.format(client.get_result().result_x, client.get_result().result_y))
