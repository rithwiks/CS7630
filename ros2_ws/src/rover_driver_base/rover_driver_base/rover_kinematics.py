#!/usr/bin/env python
from geometry_msgs.msg import Twist
import numpy as np
from numpy.linalg import pinv
from math import atan2, hypot, pi, cos, sin

prefix=["FL","FR","CL","CR","RL","RR"]

class RoverMotors:
    def __init__(self):
        self.steering={}
        self.drive={}
        for k in prefix:
            self.steering[k]=0.0
            self.drive[k]=0.0
    def copy(self,value):
        for k in prefix:
            self.steering[k]=value.steering[k]
            self.drive[k]=value.drive[k]

class DriveConfiguration:
    def __init__(self,radius,x,y,z):
        self.x = x
        self.y = y
        self.z = z
        self.radius = radius


class RoverKinematics:
    def __init__(self):
        self.X = np.asmatrix(np.zeros((3,1)))
        self.motor_state = RoverMotors()
        self.ICR = (pi/2,pi/2)
        self.first_run = True

    def ICR_cart_to_polar(x,y):
        return (atan2(y,x), atan(hypot(y,x)))

    def ICR_polar_to_cart(theta,phi):
        r = tan(phi)
        return (r*cos(theta),r*sin(theta))

    def ICR_from_twist(vx,vy,wz):
        theta=atan2(vy,vx)+pi/2
        phi=atan2(hypot(vx,vy),wz)
        return (theta,phi)

    def ICR_to_twist(theta,phi,v):
        r = tan(phi)
        T = Twist()
        T.angular.z = v / r
        T.linear.x = v * cos(theta-pi/2)
        T.linear.y = v * sin(theta-pi/2)
        return T

    def filter_twist(self, twist_in, drive_cfg):
        twist_out = Twist()
        vx = twist_in.linear.x; vy = twist_in.linear.y; wz = twist_in.angular.z
        v = hypot(vx,vy)
        if abs(v)<1e-2:
            # not moving while not changing the current ICR
            theta,phi = self.ICR
            return ICR_to_twist(theta,phi,1e-3)
        elif abs(wz)<1e-2:
            pass
        else:
            pass
        return twist_out

    def twist_to_motors(self, twist, drive_cfg, skidsteer=False, drive_state=None):
        motors = RoverMotors()
        if skidsteer:
            for k in drive_cfg.keys():
                dist = hypot(drive_cfg[k[0]+'L'].x, drive_cfg[k[0]+'L'].y)
                if k[-1] == 'L':
                    v = twist.linear.x - twist.angular.z * dist
                else:
                    v = twist.linear.x + twist.angular.z * dist
                motors.steering[k] = 0
                motors.drive[k] = v / drive_cfg[k].radius
        else:
            for k in drive_cfg.keys():
                x, y, r = drive_cfg[k].x, drive_cfg[k].y, drive_cfg[k].radius
                vx = twist.linear.x - twist.angular.z * y
                vy = twist.linear.y + twist.angular.z * x 
                theta = atan2(vy, vx)
                direction = vx * cos(theta) + vy * sin(theta)
                sign = 1.0 if direction > 0 else -1.0
                if theta > pi/2.0:
                    theta -= pi
                    sign *= -1
                elif theta < -pi/2.0:
                    theta += pi
                    sign *= -1
                motors.steering[k] = theta
                motors.drive[k] = hypot(vy, vx) / r * sign
        return motors

    def prepare_inversion_matrix(self,drive_cfg):
        # TODO: Build pseudo inverse of W using the notation from the class. The matrix size below is wrong.
        iW = np.asmatrix(np.zeros((3,2)))
        return iW

    def prepare_displacement_matrix(self, motor_state_t1, motor_state_t2, drive_cfg):
        # then compute odometry using least square
        # TODO: Build S using the notation from the class. The matrix size below is wrong.
        S = np.asmatrix(np.zeros((2,1)))
        return S

    def compute_displacement(self, motor_state, drive_cfg):
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            return np.asmatrix(np.zeros((3,1)))
        # then compute odometry using least square
        # We assume that S = W * dX, hence dX = iW * S, where iW is the pseudo inverse of W
        # TODO: First compute iW based on the wheel positions given in drive_cfg
        iW = self.prepare_inversion_matrix(drive_cfg)
        # TODO: Then compute the displacement matrix S based on the current motor_state, and the previous one stored in self.motor_state
        S = self.prepare_displacement_matrix(self.motor_state,motor_state,drive_cfg)
        # Finally the displacement in the local frame is the product
        dX = iW * S 
        # And we backup the current motor state
        self.motor_state.copy(motor_state)
        return dX

    def integrate_odometry(self, motor_state, drive_cfg):
        # First compute the local displacement in the robot frame
        dX = self.compute_displacement(motor_state,drive_cfg)
        # TODO: Now integrate the local displacement in the global frame
        self.X[0,0] += 0.0
        self.X[1,0] += 0.0
        self.X[2,0] += 0.0
        return self.X



def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = cos(ai)
    si = sin(ai)
    cj = cos(aj)
    sj = sin(aj)
    ck = cos(ak)
    sk = sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

