#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float64

rospy.init_node('arm_mover')

pub_joint1 = rospy.Publisher('simple_arm/joint_1_position_controller/command', Float64, queue_size=10)
pub_joint2 = rospy.Publisher('simple_arm/joint_2_position_controller/command', Float64, queue_size=10)

rate = rospy.Rate(10)
joint1_angle = Float64()
joint2_angle = Float64()
start_time = 0

while not start_time:
  start_time = rospy.Time().now().to_sec()

while not rospy.is_shutdown():
  time = rospy.Time().now().to_sec() - start_time

  period = 10
  frequency = 1 / period
  omega = 2 * math.pi * frequency
  amplitude = math.pi / 2
  
  joint1_angle.data = amplitude * math.sin(omega * time)
  joint2_angle.data = amplitude * math.sin(omega * time)

  pub_joint1.publish(joint1_angle)
  pub_joint2.publish(joint2_angle)

  rate.sleep()
