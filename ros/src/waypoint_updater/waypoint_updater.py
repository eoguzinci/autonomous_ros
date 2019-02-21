#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight, TrafficLightArray
from std_msgs.msg import Int32
from scipy.spatial import KDTree
import numpy as np

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.
As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.
Once you have created dbw_node, you will update this node to use the status of traffic lights too.
Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.6


class WaypointUpdater(object):
    def __init__(self):

        rospy.init_node('waypoint_updater')

        # Current position of the vehicle, provided by the simulator or localization. [geometry_msgs/PoseStamped]
        self.current_pose_sub = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

        # Waypoints as provided by a static .csv file [styx_msgs/Lane]
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.traffic_waypoint_sub = rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # simulator also provides the exact location of traffic lights and their current status in `/vehicle/traffic_lights` message.
        #self.traffic_lights_sub = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.gt_traffic_cb)

        # This is a subset of /base_waypoints. [styx_msgs/Lane]
        # The first waypoint is the one in /base_waypoints which is closest to the car.
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp_idx = -1

        self.traffic_lights  = []
        # rospy.spin()
        self.loop() # This gives us control on the publishing frequency

    def loop(self):
        rate = rospy.Rate(30) # publishing freq = 30Hz
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                # # Get closest waypoint
                # closest_waypoint_idx = self.get_closest_waypoint_idx()
                # self.publish_waypoints(closest_waypoint_idx)
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x,y],1)[1]

        # Check if the closest point is ahead or behind the vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        # Equation for hyperplane through closest coord
        cl_vec = np.array(closest_coord)
        prev_vec = np.array(prev_coord)
        pos_vec = np.array([x, y])

        val = np.dot(cl_vec - prev_vec, pos_vec - cl_vec)

        # If closest waypoint is behind the car, we take the next one
        if val > 0:
            closest_idx = (closest_idx+1) % len(self.waypoints_2d)
        return closest_idx

    def publish_waypoints(self):
        # lane.header =  self.base_waypoints.header
        # lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx+LOOKAHEAD_WPS]
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        lane = Lane()

        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
	N = len(self.base_waypoints.waypoints)
	recycle = False
	recycle_len = 0
	if farthest_idx > N-1:
		farthest_idx = N-1
		recycle = True
		recycle_len = closest_idx+LOOKAHEAD_WPS-N
	
        base_waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]
	if recycle == True:
		recycle_points = self.base_waypoints.waypoints[0:recycle_len]
		for i in recycle_points:
			base_waypoints.append(i)

        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

        return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose


            # Stop at two waypoints back from the traffic light
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
            dist = self.distance(waypoints,i,stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.:
                vel = 0

            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)

        return temp

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
        
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data
        #rospy.loginfo("traffic_cb: {0}".format(self.stopline_wp_idx))

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def gt_traffic_cb(self, msg):
        self.traffic_lights = msg.lights
        # rospy.logwarn("gt_traffic_cb: {0}".format(self.traffic_lights))

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
