#!/usr/bin/env python3
# ROS specific imports
import sys
import rclpy
from numpy import pi
import numpy.random
from task_manager_client_py.TaskClient import *

rclpy.init(args=sys.argv)
tc = TaskClient('/floor_tasks', 0.2)


while True:
    w4face = tc.WaitForFace(foreground=False, num_faces=0)
    tc.addCondition(ConditionIsCompleted("Face detector",tc,w4face))
    try:
        tc.Wander(max_linear_speed=0.7, safety_range=0.8, dont_care_range=2.5, max_angular_speed=0.6,multiplier=0.3,front_sector=False,angular_range=pi/6, task_timeout=60.0)
    except TaskConditionException as e:
        tc.get_logger().info("Path following interrupted on condition: %s" % \
                " or ".join([str(c) for c in e.conditions]))
        tc.StareAtFace(max_angular_velocity=1.0, k_theta=1.0, pixel_threshold=2.0)
        tc.Wait(duration=2.0)
        tc.SetHeading(target=pi/2,relative=True,max_angular_velocity=0.6,angle_threshold=0.1,k_theta=3.0)
    except TaskException as e:
        tc.SetHeading(target=pi/2,relative=True,max_angular_velocity=0.6,angle_threshold=0.1,k_theta=3.0)
    
    tc.Wait(duration=1.0)

tc.get_logger().info("Mission completed")
