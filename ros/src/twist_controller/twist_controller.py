
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
import math
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self,
                 vehicle_mass,
                 fuel_capacity,
                 brake_deadband,
                 decel_limit,
                 accel_limit,
                 wheel_radius,
                 wheel_base,
                 steer_ratio,
                 max_lat_accel,
                 max_steer_angle):
        # TODO: Implement
 
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle 
        self.throttle_controller = PID(1.5, 0.001, 0.0, 0 ,1)
        #self.throttle_controller = PID(8, 0.002, 0.5, 0 ,1)
        self.steer_controller = YawController(self.wheel_base,
                                              self.steer_ratio,
                                              0.001,
                                              self.max_lat_accel,
                                              self.max_steer_angle)
        self.previous_time = 0
        self.lpfilter = LowPassFilter(0.2,0.1)
        self.sim_scale_factor = 15 # set to 1 for Carla???

    def control(self, 
                cmd_linear_x,
                cmd_angular_z,
                current_linear_x,
                current_angular_z,
                reset_dt):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        #speed_error = current_linear_x - cmd_linear_x
        if reset_dt:
            self.previous_time = rospy.get_time()
            throttle = 0.0
            steer    = 0.0
            brake    = 0.0
        else:
            delta_t = rospy.get_time() - self.previous_time
        
            rospy.loginfo("cmd_linear_x is {}, current_linear_x is {}".format(cmd_linear_x, current_linear_x))
            speed_error = cmd_linear_x - current_linear_x



            steer = self.steer_controller.get_steering(cmd_linear_x, cmd_angular_z, current_linear_x)
            steer = self.lpfilter.filt(steer)
            
            if speed_error >= 0.0:
                brake = 0.0
                speed_error = self.sim_scale_factor*min(speed_error, self.accel_limit*delta_t)
                throttle = self.throttle_controller.step(speed_error, delta_t)

            else:
                throttle = 0.0
                #decel = max(self.decel_limit, speed_error/delta_t)
                decel = speed_error/delta_t
                rospy.loginfo('decel is %f', decel)
                if abs(decel) < self.brake_deadband:
                    brake = 0.0
                else:        
                    mass = self.vehicle_mass + self.fuel_capacity*GAS_DENSITY
                    brake = abs(decel)*mass*self.wheel_radius*-1
                
            rospy.loginfo('speed_error is %f', speed_error)
            #rospy.loginfo('delta_t is %f', delta_t)
            #rospy.loginfo('pid result is %f', throttle)    
            self.previous_time = rospy.get_time()

        return throttle, brake, steer
