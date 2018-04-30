#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3
VISIBLE_DIST = 30

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = Pose()
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.light_classifier = TLClassifier()
        self.has_image = False
        self.use_sim_TF = False
        
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        #rospy.loginfo('self.config is %s', self.config)
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        #self.light_classifier = TLClassifier() //Move to the position before the subscriber
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        
        #self.last_pose = Pose()
        #self.last_pose.position.x = 0.0
        #self.last_pose.position.y = 0.0
        self.current_heading = 0.0

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=1)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)

        rospy.spin()

    def pose_cb(self, msg):
        #cur_x = self.pose.position.x
        #cur_y = self.pose.position.y
        self.pose = msg.pose
        self.current_heading = msg.pose.orientation.z
        """
        if cur_x != msg.pose.position.x and cur_y != msg.pose.position.y:
            self.pose = msg.pose
            last_x = self.last_pose.position.x
            last_y = self.last_pose.position.y
            self.current_heading = math.atan2(cur_y - last_y, cur_x - last_x)
            self.last_pose.position.x = cur_x
            self.last_pose.position.y = cur_y
        """        

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights
        #for i in range(len(self.lights)):
        #    rospy.loginfo("TL[{}] state is {}".format(i, self.lights[i].state))
        light_wp, state = self.process_sim_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        """
        if state == TrafficLight.RED or state == TrafficLight.YELLOW or state == TrafficLight.GREEN or state == TrafficLight.UNKNOWN:
            
            if self.state != state:
                self.state_count = 0
                self.state = state
            elif self.state_count >= STATE_COUNT_THRESHOLD:
                self.last_state = self.state
                light_wp = light_wp if state == TrafficLight.RED else -1
                self.last_wp = light_wp
                self.upcoming_red_light_pub.publish(Int32(light_wp))
            else:
                self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            self.state_count += 1
        
            self.use_sim_TF = True
        else:
            self.use_sim_TF = False
        """
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1
        
        self.use_sim_TF = True

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        
        self.has_image = True
        self.camera_image = msg
        if self.use_sim_TF:
            """Using the /vehicle/traffic_lights information
               But we can detect the image, and compare the result with the /vehicle/traffic_lights
            """
            light_wp, state = self.process_traffic_lights()
            rospy.loginfo("image detection result is {}".format(state))
            if state != self.state:
                #rospy.loginfo("image detection result is {}".format(state))
                rospy.loginfo("But current state is {}".format(self.state))
            
        else:
            
            light_wp, state = self.process_traffic_lights()

            '''
            Publish upcoming red lights at camera frequency.
            Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
            of times till we start using it. Otherwise the previous stable state is
            used.
            '''
            
            if self.state != state:
                self.state_count = 0
                self.state = state
            elif self.state_count >= STATE_COUNT_THRESHOLD:
                self.last_state = self.state
                light_wp = light_wp if state == TrafficLight.RED else -1
                self.last_wp = light_wp
                self.upcoming_red_light_pub.publish(Int32(light_wp))
            else:
                self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            self.state_count += 1
        
    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        min_dist = dl(self.waypoints[0].pose.pose.position, pose.position)
        closest_wp_id = 0
        for i in range(1, len(self.waypoints)):
            dist = dl(self.waypoints[i].pose.pose.position, pose.position)
            if dist < min_dist:
                min_dist = dist
                closest_wp_id = i
        
        cur_x = pose.position.x
        cur_y = pose.position.y
        theta = math.atan2(cur_y, cur_x)
        map_x = self.waypoints[closest_wp_id].pose.pose.position.x
        map_y = self.waypoints[closest_wp_id].pose.pose.position.y

        heading = math.atan2(map_y - cur_y, map_x - cur_x)
        angle = math.fabs(theta - heading)
        angle = min(2*math.pi - angle, angle)
        
        # The following is to find the nearest waypoint behind the position
        # in order to find the nearest waypoint before reaching the stop line of tf
        if angle > math.pi/4:
            next_wp_id = closest_wp_id
        else:
            next_wp_id = closest_wp_id - 1
            if next_wp_id == -1:
                next_wp_id = len(self.waypoints) - 1

        return next_wp_id

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        tl_position = None
        
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            #car_position = self.get_closest_waypoint(self.pose)
            light, tl_position, index = self.find_next_tl_stop(stop_line_positions, self.pose)

        #TODO find the closest visible traffic light (if one exists)
        #light_wp = []
        if light and self.waypoints:
            rospy.loginfo('tl_position is {}'.format(tl_position))
            #rospy.loginfo("Currently car at {}, {}".format(self.pose.position.x, self.pose.position.y))

            # Using the classifier
            state = self.get_light_state(light)

            #rospy.loginfo('TL state is %d', state)
            if state == TrafficLight.RED:
                tl_pose = Pose()
                tl_pose.position.x = tl_position[0]
                tl_pose.position.y = tl_position[1]
                light_wp_id = self.get_closest_waypoint(tl_pose)
                #light_wp.append(light_wp_id)
                #light_wp_x = self.waypoints[light_wp_id].pose.pose.position.x
                #light_wp_y = self.waypoints[light_wp_id].pose.pose.position.y
                #rospy.loginfo("light_wp[{}] at {}, {}".format(light_wp_id, light_wp_x, light_wp_y))
                #light_wp_x = self.waypoints[light_wp_id-1].pose.pose.position.x
                #light_wp_y = self.waypoints[light_wp_id-1].pose.pose.position.y
                #rospy.loginfo("former_wp[{}] at {}, {}".format(light_wp_id-1, light_wp_x, light_wp_y))
                #light_wp_x = self.waypoints[light_wp_id+1].pose.pose.position.x
                #light_wp_y = self.waypoints[light_wp_id+1].pose.pose.position.y
                #rospy.loginfo("next_wp[{}] at {}, {}".format(light_wp_id+1, light_wp_x, light_wp_y))
                return light_wp_id, state
            else:
                return -1, state
        else:
            return -1, TrafficLight.UNKNOWN
        #self.waypoints = None
        #return -1, TrafficLight.UNKNOWN


    def process_sim_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        tl_position = None
        
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            #car_position = self.get_closest_waypoint(self.pose)
            light, tl_position, index = self.find_next_tl_stop(stop_line_positions, self.pose)

        #TODO find the closest visible traffic light (if one exists)
        #light_wp = []
        if light and self.waypoints:
            #rospy.loginfo('tl_position is {}'.format(tl_position))
            #rospy.loginfo("Currently car at {}, {}".format(self.pose.position.x, self.pose.position.y))

            # Using the simulator information
            state = self.lights[index].state

            #rospy.loginfo('TL state is %d', state)
            if state == TrafficLight.RED:
                tl_pose = Pose()
                tl_pose.position.x = tl_position[0]
                tl_pose.position.y = tl_position[1]
                light_wp_id = self.get_closest_waypoint(tl_pose)

                return light_wp_id, state
            else:
                return -1, state
        else:
            return -1, TrafficLight.UNKNOWN

        
    def find_next_tl_stop(self, stop_line_positions, pose):
        dl = lambda x1, y1, x2, y2: math.sqrt((x1-x2)**2 + (y1-y2)**2)
        cur_x = pose.position.x
        cur_y = pose.position.y
        #theta = math.atan2(cur_y, cur_x)
        possible_tls = []
        for i in range(len(stop_line_positions)):
            tl_x = stop_line_positions[i][0]
            tl_y = stop_line_positions[i][1]
            dist = dl(tl_x, tl_y, cur_x, cur_y)
            if dist < VISIBLE_DIST:
                heading = math.atan2(tl_y - cur_y, tl_x - cur_x)
                #rospy.loginfo("Cur_heading={}, tl_heading={}".format(self.current_heading, heading))
                angle = math.fabs(self.current_heading - heading)
                angle = min(2*math.pi - angle, angle)
                if angle < math.pi/2:
                     tl = (dist, stop_line_positions[i], i)
                     possible_tls.append(tl)
            
        if len(possible_tls) > 0:
            possible_tls = sorted(possible_tls)
            return True, possible_tls[0][1], possible_tls[0][2]
        else:
            return False, None, None

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
