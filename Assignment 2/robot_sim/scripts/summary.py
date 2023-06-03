"""
.. module:: summary

:platform: Unix
:synopsis: Python module for monitoring the goals status.

.. moduleauthor:: Nafiseh Monavari (S5174160)
"""
#! /usr/bin/env python3

import sys
import rospy
from robot_sim.srv import GoalSummary
from robot_sim.msg import PlanningActionFeedback

goals_reached = 0
goals_cancelled = 0

def updateGoalSummary(msg):
    """
    Description:
        It counts the number of reached or cancelled goals.
        
    Args: 
        msg: A message of type 'PlanningActionFeedback'.

    Returns:
        None.

    """
    global goals_reached
    global goals_cancelled

    state = msg.feedback.stat
    if state == "Target reached!":
        goals_reached = goals_reached + 1

    if state == "Target cancelled!":
        goals_cancelled = goals_cancelled + 1  
       
def sendGoalSummary(res):
    """
    Description:
        Sends response to the 'goalSummary' request.
        
    Args: 
        msg: A message of type 'PlanningActionFeedback'

    Returns:
        None.

    """
    global goals_reached
    global goals_cancelled

    return [goals_reached,goals_cancelled]

if __name__ == "__main__":
    try:
        rospy.init_node('goal_summary')
        goalSummarySubscriber = rospy.Subscriber("reaching_goal/feedback", PlanningActionFeedback, updateGoalSummary)
        goalSummaryService = rospy.Service('goalSummary', GoalSummary, sendGoalSummary)

        if goalSummaryService:
            print("Server is ready :D")
        rospy.spin()

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)