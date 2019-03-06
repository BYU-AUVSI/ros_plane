#!/usr/bin/env python
import rospy
import time
import numpy as np
import os
from rosplane_msgs.msg import Controller_Commands, State
# from pf_avoidance import PotentialField as PF
# import autograd.numpy as ag


class ObstacleDeviation():

    def __init__(self):
        self.command_sub_ = rospy.Subscriber('controller_commands_dev', Controller_Commands, self.cmdCallback, queue_size=5)
        self.command_pub_ = rospy.Publisher('controller_commands', Controller_Commands, queue_size=1)
        self.state_sub_ = rospy.Subscriber('state', State, self.stateCallback, queue_size=1)
        # vehicle_state_sub_ = nh_.subscribe<rosplane_msgs::State>("state", 1, &path_follower_base::vehicle_state_callback, this);
        # PF.addObstacle(1.0, 2, 3, 0, 0)
        # PF.addObstacle(0.0, 0, 0, 1, 1)
        #
        # state.pn = 0.0
        # state.pe = 0.0;
        # state.h = 0.0;
        # state.chi = 0.0;
        # state.Va = 0.0;
        # print("Initizaliding the obastacle thing")
        while not rospy.is_shutdown():
            rospy.spin()

    def cmdCallback(self, msg):
        #Only doing 2D plane for now. Will mess with altitude deviations later.
        currentChi = msg.chi_c
        #Calculate deviation
        # x = np.array([[state.pn], [state.pe], [state.h]]) # THESE NEED TO BE FLOATS!!!

        #Make new command
        deviatedCommand = Controller_Commands()
        deviatedCommand = msg
        # deviatedCommand.chi_c = .2
        self.command_pub_.publish(deviatedCommand)

    def stateCallback(self, msg):
        state.pn = msg.position[0]
        state.pe = msg.position[1]
        state.h = -msg.position[2]
        state.chi = msg.chi
        state.Va = msg.Va

if __name__ == '__main__':
    rospy.init_node('ObstacleDeviator')
    obsDev = ObstacleDeviation()
    # print("HERE")



# from pf_avoidance import PotentialField as PF
# import autograd.numpy as np
#
# ##############################################
# ################## TESTING ###################
# ##############################################
#
# x = np.array([[1.0], [2.0], [1.0]]) # THESE NEED TO BE FLOATS!!!
# s = np.array([[1.0], [1.0], [0.0]])
#
# PF.addObstacle(1.0, 2, 3, 0, 0)
# PF.addObstacle(0.0, 0, 0, 1, 1)
#
# print(PF.Potential(x))
# print(PF.Gradient(x))
# print(PF.Hessian(x))
# print(PF.directionalDerivative(x, s))
# print(PF.secondDirectionalDerivative(x, s))
