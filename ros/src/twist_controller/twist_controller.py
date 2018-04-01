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
        self.throttle_controller = PID(1.5, 0.001, 0.0, 0, 1)
        self.steer_controller = YawController(self.wheel_base,
                                              self.steer_ratio,
                                              0.001,
                                              self.max_lat_accel,
                                              self.max_steer_angle)
        self.previous_time = 0
        self.lpfilter = LowPassFilter(0.5, 0.2)
        #self.sim_scale_factor = 15  # set to 1 for Carla???

    def control(self,
                cmd_linear_x,
                cmd_angular_z,
                current_linear_x,
                current_angular_z,
                reset_dt):
        if reset_dt:
            self.previous_time = rospy.get_time()
            throttle = 0.0
            steer = 0.0
            brake = 0.0
        else:
            delta_t = rospy.get_time() - self.previous_time
            self.previous_time = rospy.get_time()

            current_linear_x = self.lpfilter.filt(current_linear_x)
            speed_error = cmd_linear_x - current_linear_x

            steer = self.steer_controller.get_steering(cmd_linear_x, cmd_angular_z, current_linear_x)
            throttle = self.throttle_controller.step(speed_error, delta_t)
            brake   = 0.0
            
            
            if cmd_linear_x == 0.  and current_linear_x < 0.1:
                throttle = 0.0
                brake = 400
                #speed_error = self.sim_scale_factor * min(speed_error, self.accel_limit * delta_t)
                

            elif throttle < 0.1 and speed_error < 0:
                throttle = 0.0
                decel = max(self.decel_limit, speed_error)
                #if abs(decel) < self.brake_deadband:
                #    brake = 0.0
                #else:
                 #   mass = self.vehicle_mass + self.fuel_capacity * GAS_DENSITY
                 #   brake = abs(decel) * mass * self.wheel_radius
                mass = self.vehicle_mass + self.fuel_capacity * GAS_DENSITY
                brake = abs(decel) * mass * self.wheel_radius


            rospy.loginfo('speed_error is %f', speed_error)
            rospy.loginfo('pid result is %f', throttle)
            

        return throttle, brake, steer
