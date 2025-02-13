#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from numpy import *
from numpy.linalg import inv
from math import pi, sin, cos
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import threading
from rover_driver_base.rover_kinematics import RoverKinematics
from ar_loc_base.rover_odo import RoverOdo


class RoverKF(RoverOdo):
    def __init__(self, node, initial_pose, initial_uncertainty):
        super().__init__(node,initial_pose, initial_uncertainty)
        self.X = mat(vstack(initial_pose))
        self.P = mat(diag(initial_uncertainty))
        self.ellipse_pub = node.create_publisher(Marker,"~/ellipse",1)
        self.pose_with_cov_pub = node.create_publisher(PoseWithCovarianceStamped,"~/pose_with_covariance",1)

    def getRotationFromWorldToRobot(self):
        return self.getRotation(-self.X[2,0])

    def predict(self, logger, motor_state, drive_cfg, encoder_precision):
        self.lock.acquire()
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            self.lock.release()
            return (self.X, self.P)
        # print "-"*32
        # then compute odometry using least square
        iW = self.prepare_inversion_matrix(drive_cfg)
        S = self.prepare_displacement_matrix(self.motor_state,motor_state,drive_cfg)
        self.motor_state.copy(motor_state)
        
        # TODO: Implement Kalman prediction here
        
        # ultimately : 
        # self.X =  
        # self.P = 
        DeltaX = iW @ S
        theta = self.X[2,0]
        Rtheta = mat([[cos(theta), -sin(theta), 0], [sin(theta),  cos(theta), 0], [0,0,1]])

        # self.X = self.X + self.X + Rtheta @ mat(DeltaX).reshape(3,1)
        self.X += Rtheta @ DeltaX[:, newaxis]
        dx = float(DeltaX[0])
        dy = float(DeltaX[1])

        J_x = mat([[1,0, -dx*sin(theta) - dy*cos(theta)], [0, 1, dx*cos(theta) - dy*sin(theta)], [0,0,1]])
        J_u = Rtheta

        scale = sqrt(dx**2 + dy**2)
        Q = mat(diag([encoder_precision * scale] * 3))

        self.P = J_x @ self.P @ J_x.T + J_u @ Q @ J_u.T

        self.lock.release()
        return (self.X,self.P)

    def update_ar(self, logger, Z, L, uncertainty):
        self.lock.acquire()
        logger.info("Update: L="+str(L.T)+" X="+str(self.X.T))
        # TODO
        # self.X = 
        # self.P = 
        dx = L[0,0] - float(self.X[0,0])
        dy = L[1,0] - float(self.X[1,0])
        theta = float(self.X[2,0])

        Rtheta = mat([[cos(theta), sin(theta)], [-sin(theta), cos(theta)]])
        h = Rtheta @ mat([[dx], [dy]])

        H = mat([
            [-cos(theta), -sin(theta), -dx * sin(theta) + dy*cos(theta)], 
            [sin(theta), -cos(theta), -dx*cos(theta) - dy*sin(theta)]
        ])

        R = mat(diag([uncertainty, uncertainty]))
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ inv(S)
        print(dx, dy)
        self.X = self.X + K @ (mat(Z).reshape(2,1) - h)
        self.P = (eye(3) - K @ H) @ self.P

        self.lock.release()
        return (self.X,self.P)

    def update_compass(self, logger, Z, uncertainty):
        self.lock.acquire()
        logger.info("Update: S="+str(Z)+" X="+str(self.X.T))
        # Implement kalman update using compass here
        # TODO
        # self.X = 
        # self.P = 
        theta = self.X[2,0]
        H = mat([[0,0,1]])
        R = mat([[uncertainty]])

        i = Z - theta
        while i > pi:
            i -= 2 * pi
        while i < -pi:
            i += 2 * pi

        i = mat([[i]])
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ inv(S)

        self.X = self.X + K @ i
        self.P = (eye(3) - K @ H) @ self.P

        self.lock.release()
        return (self.X,self.P)

    def publish(self, pose_pub, odom_pub, target_frame, stamp, child_frame):
        pose_simple = super().publish(pose_pub, odom_pub, target_frame, stamp, child_frame)
        pose = PoseWithCovarianceStamped()
        pose.header = pose_simple.header
        pose.pose.pose = pose_simple.pose
        C = [0.]*36
        C[ 0] = self.P[0,0]; C[ 1] = self.P[0,1]; C[ 5] = self.P[0,2]
        C[ 6] = self.P[1,0]; C[ 7] = self.P[1,1]; C[11] = self.P[1,2]
        C[30] = self.P[2,0]; C[31] = self.P[2,1]; C[35] = self.P[2,2]
        pose.pose.covariance = C
        self.pose_with_cov_pub.publish(pose)

        marker = Marker()
        marker.header = pose.header
        marker.ns = "kf_uncertainty"
        marker.id = 1
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = pose.pose.pose
        marker.scale.x = 3*sqrt(self.P[0,0])
        marker.scale.y = 3*sqrt(self.P[1,1]);
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        self.ellipse_pub.publish(marker)

        

