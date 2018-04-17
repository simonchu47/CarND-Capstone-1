#!/usr/bin/env python

import rospy
from scipy.spatial import KDTree
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32
from std_msgs.msg import String
import numpy as np
'''
This node will publish waypoints from the car's current position to some `x` distance ahead.
'''

LOOKAHEAD_WPS = 100  # Number of waypoints we will publish
NEAR_ZERO = 0.00001
STOP_BEFORE_TL = 2.5
HZ_RATE = 30  # Rospy HZ Rate to determine publishing frequency


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscriptions
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        # Publish
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.debug_waypoint_updater_pub = rospy.Publisher('debug_waypoint_updater', String, queue_size=1)

        # Member variables
        self.lane = Lane()
        self.first_pose = True
        self.current_pose = None
        self.predict_pose = None
        self.next_wp = None
        self.last_pose_stamp = rospy.Time(0)
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        self.traffic_light_wp = None
        self.traffic_light_stop = False
        self.traffic_light_det = False
        self.delta_v_per_m = 0.0
        self.final_waypoints = []
        
        # Marked for "list index out of range" issue
        #self.original_velocities = []
        
        self.waypoints_2d = []
        self.waypoint_tree = None
        
        self.waypoint_tree_unlock = True

        # Loop that keeps publishing at specified HZ rate
        rate = rospy.Rate(HZ_RATE)
        while not rospy.is_shutdown():
            if self.current_pose and self.lane.waypoints:
                if self.waypoint_tree_unlock:
                    self.publish_final_waypoints()
            
            rate.sleep()

    def waypoints_cb(self, msg):
        self.waypoint_tree_unlock = False
        self.lane.waypoints = msg.waypoints
        
        # The following is to test wether the node is single threaded or not
        """
        k = 0
        for i in range(65536):
            for j in range(4096):
                k += 1
        """
        # Marked for "list index out of range" issue
        #for i in range(len(msg.waypoints)):
        #    self.original_velocities.append(self.get_waypoint_velocity(self.lane.waypoints[i]))
        
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in
                                 msg.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
        
        self.waypoint_tree_unlock = True

    def traffic_cb(self, msg):
        self.traffic_light_wp = msg.data

    def obstacle_cb(self, msg):
        # Not yet implemented
        pass

    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def current_velocity_cb(self, velocity):
        self.current_linear_x = velocity.twist.linear.x
        self.current_angular_z = velocity.twist.angular.z

    def publish_final_waypoints(self):
        self.next_wp = self.get_closest_waypoint_id(self.current_pose)

        self.final_waypoints = []
        next_wp_id = self.next_wp
        for i in range(LOOKAHEAD_WPS):
            p = self.lane.waypoints[next_wp_id]
            self.final_waypoints.append(p)
            
            # Marked for "list index out of range" issue
            #self.set_waypoint_velocity(self.final_waypoints, i, self.original_velocities[i])
            self.set_waypoint_velocity(self.final_waypoints, i, self.get_waypoint_velocity(p))
            
            next_wp_id += 1
            if next_wp_id == len(self.lane.waypoints):
                next_wp_id = 0

        # Check if there is a red light up ahead
        
        if self.traffic_light_wp is None or self.traffic_light_wp == -1:
            self.traffic_light_stop = False
            self.traffic_light_det = False
        else :
            # Get the distance to the traffic light
            dist_to_tl = self.distance_fwrd(self.lane.waypoints, self.next_wp, self.traffic_light_wp)

            # Get desired stopping distance
            # for smooth stop use 1 m/s^2 deaccelaration
            dist_to_stop = (self.current_linear_x ** 2) / 2.0  # s = (v^2 - V^2)/2*a if a is constant

            # Assert stoping cmd if the stoping distance is less than distance to traffic light           
            if dist_to_stop >= (dist_to_tl - STOP_BEFORE_TL):
                self.traffic_light_stop = True
            else:
                self.traffic_light_stop = False


        # Start to brake
        if self.traffic_light_stop:
            next_wp_id = self.next_wp

            # Only run this once per Traffic light
            if not self.traffic_light_det:
                # Calculate deacceration per meter
                if dist_to_stop - STOP_BEFORE_TL >= 0:
                    self.delta_v_per_m = self.current_linear_x / (dist_to_stop - STOP_BEFORE_TL)
                else:
                    self.delta_v_per_m = self.current_linear_x / STOP_BEFORE_TL
                self.traffic_light_det = True

            new_v = self.current_linear_x
            for i in range(LOOKAHEAD_WPS):
                # Get distance to next wp from current
                dist_to_nxt_wp = self.distance_fwrd(self.lane.waypoints, next_wp_id, next_wp_id + 1)
                new_v -= self.delta_v_per_m * dist_to_nxt_wp

                if new_v < 0.0:
                    new_v = 0.0
                # Set new speed for wp
                self.set_waypoint_velocity(self.final_waypoints, i, new_v)

                next_wp_id += 1
                if next_wp_id == len(self.lane.waypoints):
                    next_wp_id = 0

        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.get_rostime()

        lane.waypoints = self.final_waypoints
        self.final_waypoints_pub.publish(lane)

    @staticmethod
    def get_waypoint_velocity(waypoint):
        return waypoint.twist.twist.linear.x

    @staticmethod
    def distance_between(a, b):
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)

    @staticmethod
    def set_waypoint_velocity(waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    @staticmethod
    def distance_fwrd(waypoints, wp1, wp2):
        dist = 0
        num_wp = 0
        if wp2 >= wp1:
            num_wp = wp2 - wp1
        else:
            num_wp = len(waypoints) - wp1 + wp2

        next_wp_id = wp1 + 1
        if next_wp_id == len(waypoints):
            next_wp_id = 0
                
        prev_wp_id = wp1
        for i in range(num_wp):
            dist += WaypointUpdater.distance_between(waypoints[prev_wp_id].pose.pose.position, waypoints[next_wp_id].pose.pose.position)
            prev_wp_id = next_wp_id
            next_wp_id += 1
            if next_wp_id == len(waypoints):
                next_wp_id = 0

        return dist

    def get_closest_waypoint_id(self, pose):
        x = pose.position.x
        y = pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])
        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
