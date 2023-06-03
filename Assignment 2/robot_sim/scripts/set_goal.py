"""
.. module:: set_goal

:platform: Unix
:synopsis: Python module to show an interface to the user

.. moduleauthor:: Nafiseh Monavari (S5174160)
"""
#! /usr/bin/env python3

import sys
import os
import rospy
import actionlib
import actionlib.msg
from geometry_msgs.msg import PoseStamped
from robot_sim.msg import PlanningAction, PlanningGoal

client = None

clearScreen = lambda: os.system('clear')

def showWelcome():
    """
    Description:    
        It shows a welcome message to the user.
        
    Args: 
        None.

    Returns:
        None.
        
    """
    clearScreen()
    print("----------------------------------------------------------------------------")
    print("*                                                                          *")
    print("*                          Client Control Panel                            *")
    print("*                                                                          *")
    print("----------------------------------------------------------------------------")

def error():
    print("Invalid input! Please enter the operation number from the menu...")

def showMenu():
    """
    Description:
        Shows an operation-select menu to the user.
        
    Args:
        None.

    Returns: 
        None.
 
    """
    global client

    print("\n----------------------------------------------------------------------------")
    print("*                            Operation Menu                                *")
    print("----------------------------------------------------------------------------")
    print("1-Enter a new goal position (x,y).")
    print("2-Cancel the current goal.")
    print("3-Exit.\n")

    userSelect = input("Please select the operation from the menu:")
    if(userSelect == "1"): 
        print("Please enter the goal location x,y:")
        x = input("x:")
        y = input("y:")
        try:
            x = float(x)
            y = float(y)

            
            print("\nRequest for changing position to (x:%s,y:%s)"%(x, y))
            sendGoal(client, x, y) 
        except ValueError:
            print("Invalid Input!")       
    elif(userSelect == "2"):
        cancelGoal(client)
    elif(userSelect == "3"):
        print("Exiting the user interface...Done")
        sys.exit(1)
    else:
        error()    

def createClient():
    """
    Description:
        Creates an action-client to connect to the '/reaching_goal' server.
        

    Message Type:
        PlanningAction
            geometry_msgs/PoseStamped target_pose

            ---

            ---

            geometry_msgs/Pose actual_pose
            string stat

    Args:
        None.

    Returns: 
        None.
 
    """
    try:
        client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    except rospy.ROSInterruptException:
        print("Could not create the client", file=sys.stderr) 
    return client       

def sendGoal(client, x, y):
    target_pose = PoseStamped() 
    
    try:
        client.wait_for_server()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y        
        goal = PlanningGoal(target_pose)

        client.send_goal(goal)
    except rospy.ServiceException as e: 
        print("Service call failed: %s"%e)

def cancelGoal(client):
    client.cancel_goal()
 
if __name__ == "__main__":
    showWelcome()

    try:
        rospy.init_node('client')
        client = createClient()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
    while not rospy.is_shutdown():
        showMenu()


            

        