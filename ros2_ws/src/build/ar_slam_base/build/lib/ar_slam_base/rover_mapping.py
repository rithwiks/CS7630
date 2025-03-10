#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.time import Time,Duration
from std_msgs.msg import Float64,Float32
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,Pose
from math import atan2, hypot, pi, cos, sin
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs.tf2_geometry_msgs
import message_filters
from geometry_msgs.msg import PointStamped
from aruco_opencv_msgs.msg import ArucoDetection
import tf2_kdl
import numpy as np

from rover_driver_base.rover_kinematics import prefix, RoverMotors, DriveConfiguration
from ar_slam_base.mapping_kf import *



class RoverMapping(Node):
    def __init__(self,name):
        super().__init__("rover_mapping")
        self.name = name
        self.declare_parameter('~/rover_name', self.name)
        self.declare_parameter('~/target_frame', "world")
        self.declare_parameter('~/use_ar', False)
        self.declare_parameter('~/use_compass', False)
        self.declare_parameter('~/ignore_id', False)
        self.declare_parameter('~/encoder_precision', 0.01) #[m]
        self.declare_parameter('~/ar_precision', 0.50) #[m]
        self.declare_parameter('~/compass_precision', 10 * pi/180.) #[rad]
        self.declare_parameter('~/initial_x', -5.0) #[m]
        self.declare_parameter('~/initial_y', 2.5) #[m]
        self.declare_parameter('~/initial_theta', -pi/4) #[m]
        self.name = self.get_parameter('~/rover_name').get_parameter_value().string_value
        self.use_ar = self.get_parameter('~/use_ar').get_parameter_value().bool_value
        self.use_compass = self.get_parameter('~/use_compass').get_parameter_value().bool_value
        self.ignore_id = self.get_parameter('~/ignore_id').get_parameter_value().bool_value
        self.target_frame = self.get_parameter('~/target_frame').get_parameter_value().string_value
        self.encoder_precision = self.get_parameter('~/encoder_precision').get_parameter_value().double_value
        self.ar_precision = self.get_parameter('~/ar_precision').get_parameter_value().double_value
        self.compass_precision = self.get_parameter('~/compass_precision').get_parameter_value().double_value
        self.initial_x = self.get_parameter('~/initial_x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('~/initial_y').get_parameter_value().double_value
        self.initial_theta = self.get_parameter('~/initial_theta').get_parameter_value().double_value
        self.get_logger().info("Starting rover SLAM for rover '%s'" % self.name)
        self.last_cmd = self.get_clock().now().nanoseconds/1e9
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.ready = False
        self.connected = False

        # Instantiate the right filter based on launch parameters
        initial_vec = [self.initial_x, self.initial_y, self.initial_theta]
        initial_unc = [0.01, 0.01, 0.01]
        self.mapper = MappingKF(self,initial_vec,initial_unc)


        self.get_logger().info("Waiting for initial transforms")
        self.radius={}
        for k in prefix:
            try:
                if not self.waittf('%s_ground'%(self.name), '%sDrive'%k, 60.0):
                    raise TransformException
                t = self.tf_buffer.lookup_transform('%s_ground'%(self.name),
                        '%sDrive'%k, rclpy.time.Time())
                self.radius[k] = t.transform.translation.z
                self.get_logger().info("Got transform for " + k)
            except TransformException as ex:
                self.get_logger().error("TF exception: " + repr(ex))

        self.get_logger().info("Setting up subscribers/publishers")
        self.ar_sub = self.create_subscription(ArucoDetection,"/aruco_detections", self.ar_cb, 1)
        self.compass_sub = self.create_subscription(Float32,"/vrep/compass", self.compass_cb, 1)
        self.steering_sub={}
        self.drive_sub={}
        self.pose_pub = self.create_publisher(PoseStamped,"~/pose",1)
        self.odom_pub = self.create_publisher(Odometry,"~/odom",1)
        # print "Initialising wheel data structure"
        for k in prefix:
            self.steering_sub[k] = message_filters.Subscriber(self, JointState,"/vrep/%s/%sSteerEncoder" % (self.name,k))
            self.drive_sub[k] = message_filters.Subscriber(self, JointState,"/vrep/%s/%sDriveEncoder" % (self.name,k))
            # print "Initialised wheel " + k
        self.ts = message_filters.TimeSynchronizer(list(self.steering_sub.values())+list(self.drive_sub.values()), 50)
        self.ts.registerCallback(self.sync_odo_cb)
        self.get_logger().info("Rover is ready")
        self.ready = True

    def waittf(self,tf_from,tf_to,duration):
        rate = self.create_rate(2)
        t0 = self.get_clock().now().nanoseconds/1e9
        while rclpy.ok():
            self.get_logger().info("waiting...")
            rclpy.spin_once(self,timeout_sec=0.5)
            try:
                res = self.tf_buffer.can_transform(tf_from,tf_to,rclpy.time.Time(),rclpy.time.Duration(seconds=1.),True)
                if res[0]:
                    return True
                self.get_logger().warn(res[1])
            except TransformExpection as e:
                self.get_logger().warn(f"{e}")
            now = self.get_clock().now().nanoseconds/1e9
            if now - t0 > duration:
                return False
            # rate.sleep()


    def publish(self, stamp):
        self.mapper.publish(self.pose_pub,self.odom_pub,self.target_frame,stamp,"%s_ground" % self.name)
        self.mapper.broadcast(self.broadcaster, self.target_frame, stamp)


    def sync_odo_cb(self,*args):
        self.connected = True
        if not self.ready:
            return
        if len(args)!=12:
            rospy.logerr("Invalid number of argument in OdoCallback")
            return
        steering_val = [s.position[0] for s in args[0:6]]
        drive_val = [s.position[0] for s in args[6:12]]
        motors = RoverMotors()
        motors.steering = dict(zip(self.steering_sub.keys(),steering_val))
        motors.drive = dict(zip(self.drive_sub.keys(),drive_val))
        self.odo_cb(args[0].header.stamp,motors)

    def odo_cb(self,timestamp,motors):
        # Get the pose of all drives
        drive_cfg={}
        to_frame_rel = '%s_ground'%(self.name)
        for k in prefix:
            try:
                from_frame_rel = '%sDrive'%k
                t = self.tf_buffer.lookup_transform(
                    to_frame_rel, from_frame_rel, rclpy.time.Time())
                t = t.transform.translation
                drive_cfg[k] = DriveConfiguration(self.radius[k],t.x,t.y,t.z)
            except TransformException as ex:
                self.get_logger().info( f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                return
        self.mapper.predict_rover(self.get_logger(),motors, drive_cfg, self.encoder_precision)
        self.publish(timestamp)


    def compass_cb(self, value):
        if not self.use_compass:
            return
        self.mapper.update_compass(self.get_logger(),value.data,self.compass_precision)

    def ar_cb(self, markers):
        if not self.use_ar:
            return
        self.get_logger().info("Received marker array with %d detections" % len(markers.markers))
        for m in markers.markers:
            try:
                res = self.tf_buffer.can_transform('%s_ground'% self.name, markers.header.frame_id, markers.header.stamp, rclpy.time.Duration(seconds=1.0),True)
                if not res[0]:
                    self.get_logger().info(res[1])
                    continue
                m_pose = PointStamped()
                m_pose.header = markers.header
                m_pose.point = m.pose.position
                t = self.tf_buffer.lookup_transform('%s_ground'%(self.name),markers.header.frame_id,markers.header.stamp)
                m_pose = tf2_geometry_msgs.do_transform_point(m_pose,t)
                Z = np.vstack([m_pose.point.x,m_pose.point.y])

                # TODO
                if self.ignore_id:
                    self.mapper.update_ar(self.get_logger(),Z,0,self.ar_precision)
                else:
                    self.mapper.update_ar(self.get_logger(),Z,m.marker_id,self.ar_precision)
            except BaseException as e:
                self.get_logger().error(f'{e}')
                continue
        self.publish(markers.header.stamp)



def main(args=None):
    rclpy.init(args=args)

    rover_mapping = RoverMapping('rover')

    rclpy.spin(rover_mapping)

    rover_mapping.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

