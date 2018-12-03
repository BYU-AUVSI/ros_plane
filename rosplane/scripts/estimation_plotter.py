#!/usr/bin/python
# EDIT THIS FILE +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import rospy
import time, tf
import numpy as np
import pyqtgraph as pg
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)

class Plotter:
    """
    Class for plotting methods.
    """
    def __init__(self):
        # get parameters from server
        self.t_win = rospy.get_param('~time_window', 5.0)
        self.time0 = 0
        self.init_time = True

        # setup subsribers
        rospy.Subscriber('aruco/estimate', PoseStamped, self.tVecCallback)
        rospy.Subscriber('state', Odometry, self.truthCallback)

        # initialize Qt gui application and window
        self.app = pg.QtGui.QApplication([])
        self.w = pg.GraphicsWindow(title='States vs Time')
        self.w.resize(1000,800)

        # initialize plots in one window
        self.p_pn = self.w.addPlot()
        self.p_pn.setYRange(-10,10)
        self.p_pn.addLegend(size=(1,1), offset=(1,1))
        self.p_pe = self.w.addPlot()
        self.p_pe.setYRange(-10,10)
        self.p_pd = self.w.addPlot()
        self.p_pd.setYRange(-10,10)

        # label the plots
        self.p_pn.setLabel('left', 'x')
        self.p_pe.setLabel('left', 'y')
        self.p_pd.setLabel('left', 'z')

        # create curves to update later
        self.c_pn_t = self.p_pn.plot(name='copter')
        self.c_pe_t = self.p_pe.plot()
        self.c_pd_t = self.p_pd.plot()

        self.c_pn_e = self.p_pn.plot(name='camera')
        self.c_pe_e = self.p_pe.plot()
        self.c_pd_e = self.p_pd.plot()

        # initialize state variables
        self.time_t = 0
        self.pn_t = 0
        self.pe_t = 0
        self.pd_t = 0

        self.time_e = 0
        self.pn_e = 0
        self.pe_e = 0
        self.pd_e = 0

        # truth/estimate storage lists
        self.estimates = []
        self.truths = []

        # plot list
        self.p_list = [self.p_pn, self.p_pe, self.p_pd]

        # curve lists
        self.c_list_t = [self.c_pn_t, self.c_pe_t, self.c_pd_t]
        self.c_list_e = [self.c_pn_e, self.c_pe_e, self.c_pd_e]

    # method for updating each states
    def update(self):
        # pack stored data into lists
        self.truths.append([self.time_t, self.pn_t, self.pe_t, self.pd_t])
        self.estimates.append([self.time_e, self.pn_e, self.pe_e, self.pd_e])

        # discard data outside desired plot time window
        for i in range(0,1000):
            if self.truths[0][0] < self.truths[-1][0] - self.t_win:
                self.truths.pop(0)
            if self.estimates[0][0] < self.estimates[-1][0] - self.t_win:
                self.estimates.pop(0)

        # set the window widths
        for i in range(0,len(self.p_list)):
        	self.p_list[i].setLimits(xMin=self.estimates[-1][0] - self.t_win, xMax=self.estimates[-1][0])

        # stack the data lists
        truths_array = np.vstack(self.truths)
        time_t_array = truths_array[:,0]

        estimates_array = np.vstack(self.estimates)
        time_e_array = estimates_array[:,0]

        # set the truth states
        for i in range(0,len(self.c_list_t)):
	        self.c_list_t[i].setData(time_t_array, truths_array[:,i+1], pen=(255,0,0))

        # set the estimated states
        for i in range(0,len(self.c_list_e)):
	        self.c_list_e[i].setData(time_e_array, estimates_array[:,i+1], pen=(0,255,0))

        # update the plotted data
        self.app.processEvents()


    def truthCallback(self, msg):
        # unpack positions
        self.pn_t = msg.pose.pose.position.x
        self.pe_t = msg.pose.pose.position.y
        self.pd_t = msg.pose.pose.position.z

        # unpack time
        if self.init_time == True:
        	self.time0 = msg.header.stamp.to_sec()
        	self.init_time = False
        self.time_t = msg.header.stamp.to_sec() - self.time0

    def tVecCallback(self, msg):
        # unpack positions
        self.pn_e = msg.pose.position.x
        self.pe_e = msg.pose.position.y
        self.pd_e = msg.pose.position.z

        # unpack time
        if self.init_time == True:
        	self.time0 = msg.header.stamp.to_sec()
        	self.init_time = False
        self.time_e = msg.header.stamp.to_sec() - self.time0

################################################################################
################################################################################
################################################################################


def main():
    # initialize node
    rospy.init_node('data_plotter', anonymous=True)

    # initialize plotter class
    plotter = Plotter()

    # listen for messages and plot
    while not rospy.is_shutdown():
        try:
            # plot the local positions of each vehicle
            plotter.update()
            # let it rest a bit
            time.sleep(0.001)
        except rospy.ROSInterruptException:
            print "exiting...."
            return

if __name__ == '__main__':
    main()
