#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller
from styx_msgs.msg import Lane

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass    = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity   = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband  = rospy.get_param('~brake_deadband', .1)
        decel_limit     = rospy.get_param('~decel_limit', -5)
        accel_limit     = rospy.get_param('~accel_limit', 1.)
        wheel_radius    = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base      = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio     = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel   = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub    = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub    = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `Controller` object
        self.controller = Controller(vehicle_mass,
                                     fuel_capacity,
                                     brake_deadband,
                                     decel_limit,
                                     accel_limit,
                                     wheel_radius,
                                     wheel_base,
                                     steer_ratio,
                                     max_lat_accel,
                                     max_steer_angle)

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)

        # For debug...
        rospy.Subscriber('/final_waypoints', Lane, self.final_wp_cb)


        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        self.enabled = False
        self.reset_dt = True
        self.sample_internal = 0.02
        self.last_sample_time = rospy.get_rostime()
        self.last_final_wp_time = rospy.get_rostime()
        self.final_waypoints = []

        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            #throttle, brake, steering = self.controller.control(self.cmd_linear_x,
            #                                                    self.cmd_angular_z,
            #                                                    self.current_linear_x,
            #                                                    self.current_angular_z,
            #                                                    self.sample_internal)

            #rospy.loginfo('Throttle is %f, Brake is %f', throttle, brake)

            if self.enabled == True:

               rospy.loginfo('Last sample time     secs=%d, nsecs=%d', self.last_sample_time.secs, self.last_sample_time.nsecs)
               rospy.loginfo('Final waypoints time secs=%d, nsecs=%d', self.last_final_wp_time.secs, self.last_final_wp_time.nsecs)


               throttle, brake, steering = self.controller.control(self.cmd_linear_x,
                                                                self.cmd_angular_z,
                                                                self.current_linear_x,
                                                                self.current_angular_z,
                                                                self.reset_dt)

               rospy.loginfo('Throttle is %f, Brake is %f', throttle, brake)

               self.publish(throttle, brake, steering)
               self.reset_dt = False
            else:
                self.reset_dt = True

            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def twist_cmd_cb(self, cmd):
        self.cmd_linear_x = cmd.twist.linear.x
        self.cmd_angular_z = cmd.twist.angular.z

    def current_velocity_cb(self, velocity):
        self.current_linear_x = velocity.twist.linear.x
        self.current_angular_z = velocity.twist.angular.z

        self.last_sample_time = rospy.get_rostime()

        """
        current_stamp = rospy.get_rostime()
        if current_stamp.secs > self.last_sample_time.secs:
            duration = current_stamp.secs - self.last_sample_time.secs
            duration += (1000000000 - self.last_sample_time.nsecs + current_stamp.nsecs)/1000000000.0
        elif current_stamp.secs == self.last_sample_time.secs:
            duration = (current_stamp.nsecs - self.last_sample_time.nsecs)/1000000000.0
        else:
            rospy.loginfo('Current stamp secs is %d, last secs is %d', current_stamp.secs, self.last_sample_time.secs)
            duration = 0.0

        if duration > 0:
            self.sample_internal = duration
            rospy.loginfo('duration(%f) > 0', duration)
        else:
            rospy.loginfo('Error!! duration(%d) <= 0', duration)

        self.last_sample_time = current_stamp
        """

    def dbw_enabled_cb(self, enabled):
        self.enabled = enabled.data
        if self.enabled == True:
            rospy.loginfo('DBW enabled')

    def final_wp_cb(self, lane):
        self.last_final_wp_time = lane.header.stamp
        self.final_waypoints = lane.waypoints

if __name__ == '__main__':
    DBWNode()
