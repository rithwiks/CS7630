#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from numpy.linalg import inv
from math import pi, sin, cos,hypot,sqrt
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import threading
from rover_driver_base.rover_kinematics import RoverKinematics
from ar_loc_base.rover_odo import RoverOdo


class MappingKF(RoverOdo):
    def __init__(self, node, initial_pose, initial_uncertainty):
        super().__init__(node,initial_pose,initial_uncertainty)
        self.lock = threading.Lock()
        self.X = np.mat(np.vstack(initial_pose))
        self.P = np.mat(np.diag(initial_uncertainty))
        self.idx = {}
        self.marker_pub = node.create_publisher(MarkerArray,"~/landmarks",1)

    def predict_rover(self, logger, motor_state, drive_cfg, encoder_precision):
        self.lock.acquire()
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            self.lock.release()
            return (self.X, self.P)
        # print("-"*32)
        # then compute odometry using least square
        iW = self.prepare_inversion_matrix(drive_cfg)
        S = self.prepare_displacement_matrix(self.motor_state,motor_state,drive_cfg)
        self.motor_state.copy(motor_state)
        
        # Implement Kalman prediction here
        # Compute the update in the body frame and the resulting uncertainty in the body frame
        DeltaX = iW @ S
        dx = float(DeltaX[0])
        dy = float(DeltaX[1])

        scale = np.sqrt(dx**2 + dy**2)
        DeltaP = np.mat(np.diag([encoder_precision * scale] * 3))
        
        self.lock.release()
        return self.predict_delta(logger,DeltaX,DeltaP)

    def predict_delta(self, logger, DeltaX, DeltaP):
        self.lock.acquire()
        # Update the state using the provided displacement, but we only need to deal with a subset of the state
        # Assumption: deltaX and deltaP are defined in the body frame and need to be rotated to account for the jacobian 
        # of the transfer function
        # TODO
        theta = self.X[2,0]
        Rtheta = np.mat([[cos(theta), -sin(theta), 0], 
                      [sin(theta),  cos(theta), 0],
                      [         0,           0, 1]]);
        # self.X[0:3,0] = ...
        # self.P[0:3,0:3] = ...
        self.X[0:3, 0] += Rtheta @ DeltaX[:, np.newaxis]
        dx = float(DeltaX[0])
        dy = float(DeltaX[1])

        J_x = np.mat([[1,0, -dx*sin(theta) - dy*cos(theta)], [0, 1, dx*cos(theta) - dy*sin(theta)], [0,0,1]])
        J_u = Rtheta
        self.P[0:3, 0:3] = J_x @ self.P[:3, :3] @ J_x.T + J_u @ DeltaP @ J_u.T
        self.lock.release()
        return (self.X,self.P)


    def update_ar(self, logger, Z, id, uncertainty):
        # Z is a dictionary of id->vstack([x,y])
        self.lock.acquire()
        # TODO
        logger.info("Update: Z="+str(Z.T)+" X="+str(self.X.T)+" Id="+str(id))
        print("Update: Z="+str(Z.T)+" X="+str(self.X.T)+" Id="+str(id))
        # Update the full state self.X and self.P based on landmark id
        # be careful that this might be the first time that id is observed
        # TODO
        # self.X = ...
        # self.P = ...
        theta = self.X[2,0]
        Rtheta = np.mat([[cos(theta), -sin(theta)], [sin(theta),  cos(theta)]])
        if id in self.idx:
            l = self.idx[id]
        else:
            lx, ly = self.X[:2] + Rtheta @ Z
            lx, ly = float(lx), float(ly)
            l = len(self.X)
            self.idx[id] = l
            self.X = np.append(self.X, np.array([[lx], [ly]]), axis=0)
            self.P = np.pad(self.P, ((0, 2), (0, 2)), mode='constant', constant_values=0)
            R = np.mat(np.diag([uncertainty, uncertainty]))
            self.P[l:l+2, l:l+2] = R
        lx, ly = self.X[l, 0], self.X[l+1, 0]
        dx = lx - float(self.X[0,0])
        dy = ly - float(self.X[1,0])
        theta = float(self.X[2,0])
        prev_Z = Rtheta.T @ np.mat([[dx], [dy]])
        y = Z - prev_Z

        H = np.zeros((2, len(self.X)))

        H[:, :3] = np.mat([
            [-cos(theta), -sin(theta), -dx * sin(theta) + dy*cos(theta)], 
            [sin(theta), -cos(theta), -dx*cos(theta) - dy*sin(theta)]
        ])

        H[:, l:l+2] = Rtheta.T

        R = np.mat(np.diag([uncertainty, uncertainty]))
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ inv(S)
        self.X = self.X + K @ y
        self.P = (np.eye(len(self.X)) - K @ H) @ self.P
        self.lock.release()
        return (self.X,self.P)

    def update_compass(self, logger, Z, uncertainty):
        self.lock.acquire()
        # TODO
        logger.info("Update: S="+str(Z)+" X="+str(self.X.T))
        # Update the full state self.X and self.P based on compass measurement
        # TODO
        # self.X = ...
        # self.P = ...
        theta = self.X[2,0]
        H = np.mat([[0,0,1]])
        R = np.mat([[uncertainty]])

        i = Z - theta
        while i > pi:
            i -= 2 * pi
        while i < -pi:
            i += 2 * pi

        i = np.mat([[i]])
        S = H @ self.P[:3, :3] @ H.T + R
        K = self.P[:3, :3] @ H.T @ inv(S)

        self.X[:3] = self.X[:3] + K @ i
        self.P[:3, :3] = (np.eye(3) - K @ H) @ self.P[:3, :3]
        self.lock.release()
        return (self.X,self.P)


    def publish(self, pose_pub, odom_pub, target_frame, stamp, child_frame):
        pose = super().publish(pose_pub, odom_pub, target_frame, stamp, child_frame)
        ma = MarkerArray()
        marker = Marker()
        marker.header = pose.header
        marker.ns = "kf_uncertainty"
        marker.id = 5000
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.pose.position.z = -0.1
        marker.scale.x = 3*sqrt(self.P[0,0])
        marker.scale.y = 3*sqrt(self.P[1,1]);
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        ma.markers.append(marker)
        for id in self.idx:
            marker = Marker()
            marker.header = pose.header
            marker.ns = "landmark_kf"
            marker.id = id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            l = self.idx[id]
            marker.pose.position.x = self.X[l,0]
            marker.pose.position.y = self.X[l+1,0]
            marker.pose.position.z = -0.1
            marker.pose.orientation.x = 0.
            marker.pose.orientation.y = 0.
            marker.pose.orientation.z = 1.
            marker.pose.orientation.w = 0.
            marker.scale.x = 3*sqrt(self.P[l,l])
            marker.scale.y = 3*sqrt(self.P[l+1,l+1]);
            marker.scale.z = 0.1;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.lifetime = rclpy.time.Duration(seconds=3.).to_msg()
            ma.markers.append(marker)
            marker = Marker()
            marker.header = pose.header
            marker.ns = "landmark_kf"
            marker.id = 1000+id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = self.X[l+0,0]
            marker.pose.position.y = self.X[l+1,0]
            marker.pose.position.z = 1.0
            marker.pose.orientation.x = 0.
            marker.pose.orientation.y = 0.
            marker.pose.orientation.z = 1.
            marker.pose.orientation.w = 0.
            marker.text = str(id)
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 0.2
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.lifetime = rclpy.time.Duration(seconds=3.).to_msg()
            ma.markers.append(marker)
        self.marker_pub.publish(ma)

