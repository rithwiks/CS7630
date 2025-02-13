#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from numpy import *
from numpy.linalg import inv
from math import pi, sin, cos
from visualization_msgs.msg import Marker, MarkerArray
import threading

#  Reminder
# Y = G(X)
# Cov(X) = C
# 
# Cov(Y) = dG/dX C dG/dX^T
# 
# Z = H(X,Y)
# Cov(X) = C
# Cov(Y) = D
# 
# Cov(Z) = dG/dX C dG/dX^T + dG/dY D dG/dY^T



class Landmark:
    def __init__(self, Z, X , R):
        # Initialise a landmark based on measurement Z, 
        # current position X and uncertainty R
        # TODO
        # L = g(X,Z)
        # Cov(L) = dG/dX Cov(X) dG/dX^T + dG/dZ Cov(Z) dG/dZ^T
        theta = X[2,0]
        Rtheta = mat([[cos(theta), -sin(theta)], [sin(theta),  cos(theta)]])
        self.L = X[:2] + Rtheta @ Z
        dx = float(X[0])
        dy = float(X[1])

        J_u = Rtheta
        R = mat(diag([R, R]))
        self.P = J_u @ R @ J_u.T

    def update(self,Z, X, R):
        # Update the landmark based on measurement Z, 
        # current position X and uncertainty R
        # TODO
        theta = X[2,0]
        Rtheta = mat([[cos(theta), -sin(theta)], [sin(theta),  cos(theta)]])
        new_L = X[:2] + Rtheta @ Z
        dx = float(X[0])
        dy = float(X[1])
        y = new_L - self.L
        J_u = Rtheta
        R = mat(diag([R, R]))
        S = self.P + J_u @ R @ J_u.T
        K = self.P @ inv(S)
        self.L += K @ y 
        self.P = (eye(2) - K) @ self.P
        return
        


class MappingKF:
    def __init__(self, node):
        self.lock = threading.Lock()
        self.marker_list = {}
        self.marker_pub = node.create_publisher(MarkerArray,"~/landmarks",1)

    def update_ar(self, logger, Z, X, Id, uncertainty):
        self.lock.acquire()
        # TODO
        logger.info("Update: Z="+str(Z.T)+" X="+str(X.T)+" Id="+str(Id))
        R = mat(diag([uncertainty,uncertainty]))
        if Id in self.marker_list:
            # Known landmark, we can run the KF update
            # TODO
            self.marker_list[Id].update(Z, X, uncertainty)
        else:
            # New landmark, we need to create it
            # TODO
            self.marker_list[Id] = Landmark(Z, X, uncertainty)
            logger.info("Initialised landmark %d at %s" %
                    (Id,str(self.marker_list[Id].L.T)))
        self.lock.release()


    def publish(self, target_frame, timestamp):
        ma = MarkerArray()
        for id in self.marker_list:
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = target_frame
            marker.ns = "landmark_kf"
            marker.id = id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            Lkf = self.marker_list[id]
            marker.pose.position.x = Lkf.L[0,0]
            marker.pose.position.y = Lkf.L[1,0]
            marker.pose.position.z = 0.
            marker.pose.orientation.x = 0.
            marker.pose.orientation.y = 0.
            marker.pose.orientation.z = 1.
            marker.pose.orientation.w = 0.
            marker.scale.x = max(3*sqrt(Lkf.P[0,0]),0.05)
            marker.scale.y = max(3*sqrt(Lkf.P[1,1]),0.05)
            marker.scale.z = 0.5;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.lifetime = rclpy.time.Duration(seconds=3.).to_msg()
            ma.markers.append(marker)
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = target_frame
            marker.ns = "landmark_kf"
            marker.id = 1000+id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            Lkf = self.marker_list[id]
            marker.pose.position.x = Lkf.L[0,0]
            marker.pose.position.y = Lkf.L[1,0]
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

