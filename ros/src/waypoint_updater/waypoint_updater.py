#!/usr/bin/env python
from __future__ import division
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32
import copy

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

LOOKAHEAD_WPS = 20 # Number of waypoints we will publish. You can change this number
PREDICT_TIME = 1.0
NEAR_ZERO = 0.00001

class WaypointUpdater(object):

    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.seeRedTL = False
        self.redTL_wp_id = None
        self.next_wp = None
        
        #rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
	
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_wp_cb, queue_size=1)

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

        self.current_pose = msg.pose
        if len(self.lane.waypoints) > 0:
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
        
            if self.seeRedTL:
                rospy.loginfo("SEE RED LIGHT!!!!!!")
                self.final_waypoints = self.gen_see_red_tl_wps()
            else:
                self.final_waypoints = self.gen_common_wps()
            
            wp_v = []
            for i in range(len(self.final_waypoints)):
                wp_v.append(self.final_waypoints[i].twist.twist.linear.x)
            rospy.loginfo("velocity cmd = {}".format(wp_v))
            rospy.loginfo("wp[292], v = {}".format(self.lane.waypoints[292].twist.twist.linear.x))
        
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
        angle = math.fabs(theta - heading)
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
        
    def traffic_wp_cb(self, tl_wp):
        tl_wp_id = tl_wp.data
        rospy.loginfo("Got RED LIGHT TOPIC====wp_id is {}".format(tl_wp_id))
        visible = False
        if self.next_wp:
            rospy.loginfo("====next wp is {}".format(self.next_wp))
            if self.next_wp < tl_wp_id:
                if self.next_wp + LOOKAHEAD_WPS > tl_wp_id:
                    visible = True
            else:
                if self.next_wp + LOOKAHEAD_WPS > tl_wp_id + len(self.lane.waypoints):
                    visible = True
        
        self.seeRedTL = visible
        self.redTL_wp_id = tl_wp_id
        
    def gen_see_red_tl_wps(self):
        waypoints = []
        next_wp_id = self.next_wp
        red_tl_wp_id = self.redTL_wp_id
        v = self.current_linear_x
        if next_wp_id == red_tl_wp_id:
            dist = 0
            a = -1
            pass_by_tl = True
        else:
            dist = self.distance(self.lane.waypoints, next_wp_id, red_tl_wp_id)
            a = v**2/2/dist
            rospy.loginfo("dist={}, a={}, v={}".format(dist, a, v))
            pass_by_tl = False
            
        for i in range(LOOKAHEAD_WPS):
            p = copy.deepcopy(self.lane.waypoints[next_wp_id])
            if pass_by_tl:
                p.twist.twist.linear.x = 0
            else:
                s = self.distance(self.lane.waypoints, self.next_wp, next_wp_id)
                check = v**2 - 2*a*s
                if check > 0:
                    rospy.loginfo("check={}".format(check))
                    #divided = v - math.sqrt(check)
                    p.twist.twist.linear.x = math.sqrt(check)
                    
                else:
                    rospy.loginfo("check<=0!!")
                    p.twist.twist.linear.x = 0
                #divided = v - math.sqrt(v**2 - 2*a*s)
                #t = (v - math.sqrt(v**2 - 2*a*s))/a
                #t = divided/a
                #rospy.loginfo("/////////////start={}, end={}".format(self.next_wp, next_wp_id))
                #rospy.loginfo("s={}, divided={}".format(s, divided))
                rospy.loginfo("a={}//////////".format(a))
            
            waypoints.append(p)
            next_wp_id += 1
            if next_wp_id == len(self.lane.waypoints):
                next_wp_id = 0
            
            #if next_wp_id == self.redTL_wp_id + 1:
            #    pass_by_tl = True
        
        return waypoints
    
    def gen_common_wps(self):
        waypoints = []
        next_wp_id = self.next_wp
        for i in range(LOOKAHEAD_WPS):
            p = self.lane.waypoints[next_wp_id]
            waypoints.append(p)
            next_wp_id += 1
            if next_wp_id == len(self.lane.waypoints):
                next_wp_id = 0
        return waypoints

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
