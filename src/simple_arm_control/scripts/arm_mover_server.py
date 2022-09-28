#!/usr/bin/env python3

import rospy
from simple_arm_control.srv import GoToPosition, GoToPositionResponse
from std_msgs.msg import Float64


class ArmMoverServer:

  def __init__(self):
    # Define two publishers to publish Float64 messages on joints respective topics
    self.joint1_pub = rospy.Publisher('simple_arm/joint_1_position_controller/command', Float64, queue_size=10)
    self.joint2_pub = rospy.Publisher('simple_arm/joint_2_position_controller/command', Float64, queue_size=10)
    # Define a safe_move service with a handle_save_move_request callback function
    self.service = rospy.Service('arm_mover/safe_move', GoToPosition, self.handle_safe_move_request)
    self.joint_angles = []
    self.joint1_angle = Float64()
    self.joint2_angle = Float64()
  
  def clamp_at_boundaries(self, requested_joint_1, requested_joint_2):
    # Define clamped joint angles and assign them to the requested ones
    clampled_j1 = requested_joint_1
    clampled_j2 = requested_joint_2

    # Get node name
    node_name = rospy.get_name()

    # Get joints min and max parameters
    min_j1 = rospy.get_param(node_name + "/min_joint_1_angle")
    max_j1 = rospy.get_param(node_name + "/max_joint_1_angle")
    min_j2 = rospy.get_param(node_name + "/min_joint_2_angle")
    max_j2 = rospy.get_param(node_name + "/max_joint_2_angle")

    # Check if joint 1 falls in the safe zone, otherwise clamp it
    if (requested_joint_1 < min_j1) or (requested_joint_1 > max_j1):
      clampled_j1 = min(max(min_j1, requested_joint_1), max_j1)
      rospy.logwarn(f"Joint_1 is out of bounds, valid range [{min_j1, max_j1}], clampling to: {clampled_j1:.2f}")

    # Check if joint 2 falls in the safe zone, otherwise clamp it
    if (requested_joint_2 < min_j2) or (requested_joint_2 > max_j2):
      clampled_j2 = min(max(min_j2, requested_joint_1), max_j2)
      rospy.logwarn(f"Joint_2 is out of bounds, valid range [{min_j2, max_j2}], clampling to: {clampled_j2:.2f}")

    # Store clamped joint angles in a clamped_data list
    clamped_data = [clampled_j1, clampled_j2]

    return clamped_data

  def handle_safe_move_request(self, req):
    rospy.loginfo(f"GoToPositionRequest received - Joint_1: {req.joint_1:.2f}, Joint_2: {req.joint_2:.2f}")
    
    # Check if requested joint angles are in the safe zone, otherwise clamp them
    self.joint_angles = self.clamp_at_boundaries(req.joint_1, req.joint_2)
    
    # Publish clamped joint angles to the arm
    self.joint1_angle.data = self.joint_angles[0]
    self.joint2_angle.data = self.joint_angles[1]

    self.joint1_pub.publish(self.joint1_angle)
    self.joint2_pub.publish(self.joint2_angle)

    # Wait 3 seconds for arm to settle
    rospy.sleep(3)
    
    # Return a response message
    msg_response = f"Joint angles set - Joint_1: {self.joint_angles[0]:.2f}, Joint_2: {self.joint_angles[1]:.2f}"
    return GoToPositionResponse(msg_response)

if __name__ == '__main__':
  # Initialize the arm_mover_server node
  rospy.init_node('arm_mover_server')
  
  # Create ArmMoverServer
  ArmMoverServer()
  
  # Handle ROS communication events
  rospy.spin()
