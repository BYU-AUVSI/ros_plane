#!/usr/bin/env python
import rospy
import time
import numpy as np
import os
from rosplane_msgs.msg import Controller_Commands


class ObstacleDeviation():

    def __init__(self):
        self.command_sub_ = rospy.Subscriber('controller_commands', Controller_Commands, self.cmdCallback, queue_size=5)
        self.command_pub_ = rospy.Publisher('controller_commands_dev', Controller_Commands, queue_size=1)
        # print("Initizaliding the obastacle thing")
        while not rospy.is_shutdown():
            rospy.spin()

    def cmdCallback(self, msg):
        #Only doing 2D plane for now. Will mess with altitude deviations later.
        currentChi = msg.chi_c
        deviatedCommand = Controller_Commands()
        deviatedCommand = msg
        # deviatedCommand.chi_c = .2
        self.command_pub_.publish(deviatedCommand)

if __name__ == '__main__':
    rospy.init_node('ObstacleDeviator')
    obsDev = ObstacleDeviation()
    # print("HERE")
