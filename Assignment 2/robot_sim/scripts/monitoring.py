"""
.. module:: monitoring

:platform: Unix
:synopsis: Python module for monitoring the data.

.. moduleauthor:: Nafiseh Monavari (S5174160)
"""
#! /usr/bin/env python3

import sys
import rospy
import math
from geometry_msgs.msg import Point
from robot_sim.msg import OdoSensor, KinematicData

kinematicData = KinematicData()
kinematicData.distance = 0
kinematicData.vel_x_avrg = 0
kinematicData.vel_y_avrg = 0

desiredPosition = Point()

velXData = []
velYData = []

sumVx = 0
sumVy = 0

period = 10

def movingAverage(vx, vy):
    """
    Description:
        It computes the moving average of the velocity data.
        
    Args: 
        vx: Current velocity in the x direction
        vy: Current veloity in the y direction

    Returns:
        None.

    """
    global sumVx, sumVy, velXData, velYData, period

    sumVx += vx
    sumVy += vy

    velXData.append(vx)
    velYData.append(vy)

    if (len(velXData) > period):
        sumVx -= velXData.pop(0)
        sumVy -= velYData.pop(0)
        return [sumVx / period, sumVy/period] 

    return [0, 0]

def updateKinematicInfo(odoMsg):
    """
    Description:
        It compute the 'distance to the target' and 'average speed' of the robot.
        
    Args:
        odoMsg: A message of type 'OdoSensor'

    Returns:
        None.

    """
    global processData

    desiredPosition.x = rospy.get_param('des_pos_x')
    desiredPosition.y = rospy.get_param('des_pos_y')

    kinematicData.distance = math.sqrt(pow(desiredPosition.y - odoMsg.y, 2) + pow(desiredPosition.x - odoMsg.x, 2))

    averageVelocities = movingAverage(odoMsg.vel_x, odoMsg.vel_y)

    kinematicData.vel_x_avrg = averageVelocities[0]
    kinematicData.vel_y_avrg = averageVelocities[1]

if __name__ == "__main__":
    printRate = rospy.get_param('print_rate')

    try:
        rospy.init_node('kinematic_monitor')
        kinematicSubscriber = rospy.Subscriber("odometer", OdoSensor, updateKinematicInfo)
        kinematicPublisher = rospy.Publisher('kinematic_monitor', KinematicData, queue_size=10)          
        rate = rospy.Rate(printRate)

        while not rospy.is_shutdown():
            if kinematicData:
                kinematicPublisher.publish(kinematicData)
                print("Distance to Taget: %.3f, Average Vx, Vy: %.3f, %.3f"%(kinematicData.distance, kinematicData.vel_x_avrg, kinematicData.vel_y_avrg ))
                rate.sleep()                
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
       
  
    



            

        