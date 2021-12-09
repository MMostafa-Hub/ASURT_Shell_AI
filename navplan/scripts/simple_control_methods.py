#!/usr/bin/env python3
import numpy as np
import rospy

from std_msgs.msg import Float64MultiArray, Float64
from modules.sprites import *

rospy.init_node('pure_pursuit_control')
long_drive_pub = rospy.Publisher('controls/throttle', Float64, queue_size=10)
steering_drive_pub = rospy.Publisher('controls/steer', Float64, queue_size=10)

current_state = np.zeros(6)


def update_car_state(data):
    global current_state
    # [x, y, theta, speed, beta (slip angle), theta_dot]
    current_state = np.array(data.data)


curr_waypoint = None


def update_waypoint(data):
    global curr_waypoint
    curr_waypoint = np.array(data.data)  # [x, y, yaw_path] of next waypoint


rospy.Subscriber("vehicle_model/state", Float64MultiArray, update_car_state)
rospy.Subscriber("waypoints", Float64MultiArray, update_waypoint)


class Controller:
    def __init__(self, L=4.9):
        self.L = L
        self.sigma_e = 0
        self.cur_e = 0
        self.past_e = 0
        self.kp = 0.3
        self.ki = 0.01
        self.kd = 0.1
        self.cross_e_gain = 5
        self.ks = 0.001

    def CG_to_rear(self, CG_xy, theta):
        rear_x = CG_xy[0] - ((self.L/2) * np.cos(theta))
        rear_y = CG_xy[1] - ((self.L/2) * np.sin(theta))
        return np.array([rear_x, rear_y])

    def get_longitudinal_control(self, v_current, v_desired, dt):
        '''
        PID Longitudinal controller
        Parameters
        ----------
        v_current: float
            Current speed of the vehicle
        v_desired: float
            Desired speed of the vehicle
        dt: float
            Delta time since last time the function was called

        Returns
        -------
        throttle_output: float
            Value in the range [-1,1]
        '''
        self.cur_e = v_desired - v_current
        self.sigma_e += self.cur_e
        self.der_e = (self.cur_e - self.past_e) / dt

        self.past_e = self.cur_e
        action = (self.kp * self.cur_e) + (self.ki *
                                           self.sigma_e) + (self.kd * self.der_e)
        return np.tanh(action)

    def get_lateral_pure_pursuit(self, current_xy, current_yaw, next_waypoint):
        '''
        Pure Pursuit, lateral controller

        Parameters
        ----------
        current_xy: np.array of floats, shape=2
            Current position of the vehicle [x,y] given in CG frame
        current_yaw: float
            Current heading of the vehicle
        next_waypoint: np.array of floats, shape=2
            Next waypoint for the vehicle to reach [x,y]

        Returns
        -------
        steer_angle: float
            Steering angle in rad
        '''
        pass

    def get_lateral_stanley(self, current_xy, current_yaw, current_speed, next_waypoint):
        '''
        Stanley, lateral controller

        Parameters
        ----------
        current_xy: np.array of floats, shape=2
            Current position of the vehicle [x,y] given in CG frame
        current_yaw: float
            Current heading of the vehicle
        current_speed: float
            Current speed of the vehicle
        next_waypoint: np.array of floats, shape=3
            Next waypoint for the vehicle to reach [x,y,yaw_path]

        Returns
        -------
        steer_angle: float
            Steering angle in rad
        '''
        pass


controller = Controller()

rate = 10
r = rospy.Rate(rate)

while not rospy.is_shutdown():
    r.sleep()
    if curr_waypoint is None:
        continue

    des_v = 20
    dt = 0.00001
    # Getting states variables from current car state (position, heading, speed)
    cur_cg_x, cur_cg_y, cur_theta, cur_v, cur_slip_angle, cur_theta_dot = current_state

    # Longitudinal and lateral control
    longitudinal_cont = controller.get_longitudinal_control(
        v_current=cur_v, v_desired=des_v, dt=dt)
    # lateral_cont = controller.get_lateral_pure_pursuit(
    #     np.array([cur_cg_x, cur_cg_y]), cur_theta, curr_waypoint)
    lateral_cont = controller.get_lateral_stanley(
        np.array([cur_cg_x, cur_cg_y]), cur_theta, cur_v, curr_waypoint)

    # Create longitudinal and lateral messages (of type Float64 imported above)

    lat_cont_msg = Float64(lateral_cont)
    long_cont_msg = Float64(longitudinal_cont)

    # Publish the 2 messages
    steering_drive_pub.publish(lat_cont_msg)
    long_drive_pub.publish(long_cont_msg)

    print("Throttle: {:.2f}, Steering angle: {:.2f}".format(
        longitudinal_cont, lateral_cont))
