#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import Lane, Waypoint

import math
from geometry_msgs.msg import TwistStamped

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

LOOKAHEAD_WPS = 10 # Number of waypoints we will publish. You can change this number
PREDICT_TIME = 1.0
NEAR_ZERO = 0.00001

class WaypointUpdater(object):

    def __init__(self):
        rospy.init_node('waypoint_updater')
        
        self.current_pose = Pose()
        self.last_pose = Pose()
        self.last_pose.position.x = 0.0
        self.last_pose.position.y = 0.0
        self.current_heading = 0.0

        #rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
	
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        #self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane)

        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        # TODO: Add other member variables you need below
        self.lane = Lane()
        self.first_pose = True
        self.last_pose_stamp = rospy.Time(0)
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        #pass

        #self.current_pose = msg.pose
        cur_x = self.current_pose.position.x
        cur_y = self.current_pose.position.y
        if cur_x != msg.pose.position.x and cur_y != msg.pose.position.y:
            self.current_pose = msg.pose
            last_x = self.last_pose.position.x
            last_y = self.last_pose.position.y
            self.current_heading = math.atan2(cur_y - last_y, cur_x - last_x)
            self.last_pose.position.x = cur_x
            self.last_pose.position.y = cur_y
        
        delay_d = self.current_linear_x * PREDICT_TIME
        phi = math.atan2(self.current_pose.position.y, self.current_pose.position.x) + self.current_pose.orientation.z + self.current_angular_z*PREDICT_TIME
        delta_x = delay_d*math.sin(phi)
        delta_y = delay_d*math.cos(phi)
        self.predict_pose = Waypoint().pose.pose
        self.predict_pose.position.x = self.current_pose.position.x + delta_x
        self.predict_pose.position.y = self.current_pose.position.y + delta_y

        rospy.loginfo('Current pose --- x is %f, y is %f', self.current_pose.position.x, self.current_pose.position.y)
        rospy.loginfo('Predict pose --- x is %f, y is %f', self.predict_pose.position.x, self.predict_pose.position.y)
        self.next_wp = self.find_next_wp(self.lane.waypoints, self.predict_pose)
        #self.next_wp = self.find_next_wp(self.lane.waypoints, self.current_pose)

        map_x = self.lane.waypoints[self.next_wp].pose.pose.position.x
        map_y = self.lane.waypoints[self.next_wp].pose.pose.position.y

        rospy.loginfo('Next waypoint[%d] x is %f, y is %f', self.next_wp, map_x, map_y)

        self.final_waypoints = []
        next_wp_id = self.next_wp
        for i in range(LOOKAHEAD_WPS):
            p = self.lane.waypoints[next_wp_id]
            self.final_waypoints.append(p)
            next_wp_id += 1
            if next_wp_id == len(self.lane.waypoints):
                next_wp_id = 0
        lane = Lane()
        lane.header.frame_id = '/world'
        #lane.header.stamp = rospy.Time(0)
        lane.header.stamp = rospy.get_rostime()

        lane.waypoints = self.final_waypoints
        self.final_waypoints_pub.publish(lane)

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        #pass
        #rospy.loginfo('Base waypoints Received - size is %d', len(waypoints.waypoints))
        self.lane.waypoints = waypoints.waypoints
        rospy.loginfo('Base waypoints Received - size is %d', len(self.lane.waypoints))


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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

    def find_next_wp(self, waypoints, pose):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        min_dist = dl(waypoints[0].pose.pose.position, pose.position)
        closest_wp_id = 0
        for i in range(1, len(waypoints)):
            dist = dl(waypoints[i].pose.pose.position, pose.position)
            if dist < min_dist:
                min_dist = dist
                closest_wp_id = i
        cur_x = pose.position.x
        cur_y = pose.position.y
        theta = math.atan2(cur_y, cur_x)
        map_x = waypoints[closest_wp_id].pose.pose.position.x
        map_y = waypoints[closest_wp_id].pose.pose.position.y

        heading = math.atan2(map_y - cur_y, map_x - cur_x)
        angle = math.fabs(self.current_heading - heading)
        #angle = math.fabs(theta - heading)
        angle = min(2*math.pi - angle, angle)
        if angle > math.pi/4:
            next_wp_id = closest_wp_id + 1
            if next_wp_id == len(waypoints):
                next_wp_id = 0
        else:
            next_wp_id = closest_wp_id

        return next_wp_id

    def get_duration(self, stamp1, stamp2):
        if stamp1.secs == stamp2.secs:
            duration = (stamp2.nsecs - stamp1.nsecs)/1000000000.0
        else:
            duration = stamp2.secs - stamp1.secs
            duration += (1000000000 - stamp1.nsecs + stamp2.nsecs)/1000000000.0

        return duration

    def current_velocity_cb(self, velocity):
        self.current_linear_x = velocity.twist.linear.x
        self.current_angular_z = velocity.twist.angular.z

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
