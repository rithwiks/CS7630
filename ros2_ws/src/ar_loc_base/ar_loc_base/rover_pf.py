#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy
from numpy import *
from numpy.linalg import inv
from math import pi, sin, cos
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
import bisect
import threading
from rover_driver_base.rover_kinematics import RoverKinematics
from ar_loc_base.rover_odo import RoverOdo


class RoverPF(RoverOdo):
    def __init__(self, node, initial_pose, initial_uncertainty):
        super().__init__(node,initial_pose, initial_uncertainty)
        self.N = 500
        self.particles = [self.X + self.drawNoise(initial_uncertainty) for i in range(0,self.N)]
        self.pa_pub = node.create_publisher(PoseArray,"~/particles",1)
        # print self.particles

    def getRotationFromWorldToRobot(self):
        return self.getRotation(-self.X[2,0])

    def drawNoise(self, norm):
        if type(norm)==list:
            return mat(vstack(norm)*(2*random.rand(len(norm),1)-vstack([1]*len(norm))))
        else:
            return mat(multiply(norm,((2*random.rand(3,1)-vstack([1,1,1])))))

    def applyDisplacement(self,X,DeltaX,Uncertainty):
        # TODO: apply the displacement DeltaX, in the robot frame, to the particle X expressed in the world frame,
        # including the uncertainty present in variable uncertainty
        theta = X[2,0]
        Rtheta = mat([[cos(theta), -sin(theta), 0], [sin(theta),  cos(theta), 0], [         0,           0, 1]])
        X = X + Rtheta @ (DeltaX[:, newaxis] + Uncertainty)
        return X 


    def predict(self, logger, motor_state, drive_cfg, encoder_precision):
        self.lock.acquire()
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            self.lock.release()
            return self.X
        # then compute odometry using least square
        iW = self.prepare_inversion_matrix(drive_cfg)
        S = self.prepare_displacement_matrix(self.motor_state,motor_state,drive_cfg)
        self.motor_state.copy(motor_state)

        # Apply the particle filter prediction step here
        # TODO

        # DeltaX = iW*S
        # Note, using the function applyDisplacement could be useful to compute the new particles
        # self.particles = ...
        DeltaX = iW @ S
        for i in range(len(self.particles)):
            uncertainty = self.drawNoise(encoder_precision)
            self.particles[i] = self.applyDisplacement(self.particles[i], DeltaX, uncertainty)
        self.updateMean()
        self.lock.release()

    def evalParticleAR(self,X, Z, L, Uncertainty):
        # Returns the fitness of a particle with state X given observation Z of landmark L
        # theta = X[2]
        # Rtheta = mat([[cos(theta), -sin(theta)], [sin(theta),  cos(theta)]])
        # L_part = X[:2] + Rtheta @ Z
        # temp = L - L_part
        # dist = temp[0]**2 + temp[1]**2
        # p_part = exp(-(dist) / (2*Uncertainty**2))
        # return p_part[0]
        theta = X[2]
        Rtheta = mat([[cos(theta), -sin(theta)], [sin(theta),  cos(theta)]])
        temp = Z - Rtheta.T @ L
        dist = temp[0]**2 + temp[1]**2
        p_part = exp(-(dist) / (2*Uncertainty**2))
        return p_part[0]

    def evalParticleCompass(self,X, Value, Uncertainty):
        # Returns the fitness of a particle with state X given compass observation value
        # Beware of the module when computing the difference of angles
        angle_diff = abs(X[2].item() - Value)
        angle_diff = min(angle_diff, 2 * pi - angle_diff)

        return exp(-0.5 * (angle_diff**2) / (Uncertainty **2))


    def update_ar(self, logger, Z, L, Uncertainty):
        self.lock.acquire()
        # TODO
        logger.info("Update: L="+str(L.T)+" X="+str(self.X.T))
        # Implement particle filter update using landmarks here. Using the function evalParticleAR could be useful

        # TODO

        # self.particles = ...
        weights = [0]*len(self.particles)
        for i, part in enumerate(self.particles):
            weights[i] = self.evalParticleAR(part, Z, L, Uncertainty)
        weights = array(weights).squeeze()
        s = weights.sum()
        weights /= s 
        inds = random.choice(len(self.particles), size=len(self.particles), p=weights)
        new_parts = [self.particles[i] for i in inds]
        self.particles = new_parts


        self.updateMean()
        self.lock.release()

    def update_compass(self, logger, angle, Uncertainty):
        self.lock.acquire()
        # TODO
        # print self.particles
        logger.info("Update: S="+str(angle)+" X="+str(self.X.T))
        # Implement particle filter update using landmarks here. Using the function evalParticleCompass could be useful

        # TODO

        weights = []
        for particle in self.particles:
            weights.append(self.evalParticleCompass(self.X, angle, Uncertainty))

        weights /= sum(weights)
        inds = random.choice(len(self.particles), size=len(self.particles), p=weights)
        new_parts = [self.particles[i] for i in inds]
        self.particles = new_parts
        
        self.updateMean()
        self.lock.release()

    def updateMean(self):
        X = mat(zeros((3,1)))
        for x in self.particles:
            X += x
        self.X = X / len(self.particles)
        
        return self.X

    def publish(self, pose_pub, odom_pub, target_frame, stamp, child_frame):
        pose = super().publish(pose_pub, odom_pub, target_frame, stamp, child_frame)

        pa = PoseArray()
        pa.header = pose.header
        for p in self.particles:
            po = Pose()
            po.position.x = p[0,0]
            po.position.y = p[1,0]
            q = self.quaternion_from_euler(0, 0, p[2,0])
            po.orientation.x = q[0]
            po.orientation.y = q[1]
            po.orientation.z = q[2]
            po.orientation.w = q[3]
            pa.poses.append(po)
        self.pa_pub.publish(pa)


