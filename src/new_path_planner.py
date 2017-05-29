#!/usr/bin/env python
# Python implementation of "path_planner.cpp"

import rospy
from ros_plane.msg import Waypoint
import math

num_waypoints = 5

def publishwaypoints():

	# Init ROS Node
	rospy.init_node('ros_plane_path_planner', anonymous=True)

	# Init Publisher
	waypointPublisher = rospy.Publisher('waypoint_path',Waypoint, queue_size=10)

	# Sleep, (this fixed bug of first waypoint not publishing)
	d = rospy.Duration(.5)
	rospy.sleep(d)

	# Set waypoints
	Va = 15.0#8.5 # 11.0
	wps =  [
				0.0, 0.0, -60, 0.0, Va,
				150.0, 5.7077, -60, 0.0, Va,
				-20, -100.71729, -60, -math.pi, Va,
				70.5817, 30.4049, -60, 1.0, Va,
				170.8173, -100.1882, -60, -1.5, Va
                #.562, -18.7037, -60, math.pi, Va
				]
                # -10, -10, -30, -45, Va,
                # -10, -125, -30, -135*math.pi/180, Va,
                # -125, -10, -30, 45*math.pi/180, Va,
                # -125, -125, -30, 135*math.pi/180, Va]

    # Loop through each waypoint
	for i in range(0,num_waypoints):

		# Make waypoint a FW_Waypoint msg
		new_waypoint = Waypoint()

		new_waypoint.w[0] = wps[i*5 + 0]
		new_waypoint.w[1] = wps[i*5 + 1]
		new_waypoint.w[2] = wps[i*5 + 2]
		new_waypoint.chi_d = wps[i*5 + 3]

		new_waypoint.chi_valid = True # True
		new_waypoint.set_current = False
		new_waypoint.Va_d = wps[i*5 + 4]

		# Publish the Waypoint
		waypointPublisher.publish(new_waypoint)

		# Sleep
		d = rospy.Duration(0.5)
		rospy.sleep(d)


if __name__ == '__main__':

	# Just run the publisher once
	try:
		publishwaypoints()
	except rospy.ROSInterruptException:
		pass
