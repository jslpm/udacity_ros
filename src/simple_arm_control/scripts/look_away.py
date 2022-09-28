#!/usr/bin/env python3

import rospy
import math
from simple_arm_control.srv import GoToPosition
from sensor_msgs.msg import JointState, Image


class LookAway:
  
  def __init__(self):
    # Define a cliente service capable of requesting services from safe_move
    rospy.wait_for_service('arm_mover/safe_move')
    self.mover_client = rospy.ServiceProxy('arm_mover/safe_move', GoToPosition)

    # Subscribe to /simple_arm/joint_states topic to read the arm joints 
    # position inside the joint_states_callback function
    self.sub1 = rospy.Subscriber("simple_arm/joint_states", JointState, 
      self.joint_states_callback)

    # Subscribe to rgb_camera/image_raw topic to read the image data 
    # inside the look_away_callback function
    self.sub2 = rospy.Subscriber('rgb_camera/image_raw', Image, 
      self.look_away_callback)

    # Define vector of joints last position and moving state of the arm
    self.joints_last_position = [0, 0]
    self.joints_current_position = [0, 0]
    self.moving_state = False

    # Define a tolerance theshhold to compare double values
    self.tolerance = 0.0005


  def joint_states_callback(self, js):
    """This callback function continuously executes and reads the arm joint
    angles position"""

    # Get joints current position
    self.joints_current_position = js.position

    # Check if the arm is moving by comparing its current joints position to
    # its latest
    if (
      math.fabs(self.joints_current_position[0] - self.joints_last_position[0]) 
      < self.tolerance and math.fabs(self.joints_current_position[1] - 
      self.joints_last_position[1])) < self.tolerance:
      self.moving_state = False
    else:
      self.moving_state = True
      self.joint_last_position = self.joints_current_position


  def look_away_callback(self, img):
    """This callback function continously executes and reads the image data"""
    # uniform_image = True

    # Loop through each pixel in the image and check if its equal to 
    # the first one
    # for pixel in img.data:
    #   if (pixel - img.data[0]) != 0:
    #     uniform_image = False
    #     break
    
    # If the image is uniform and the arm is not moving,
    # move the arm to the center
    if self.moving_state == False:
      self.move_arm_center()

  def move_arm_center(self):
    '''This function calls the safe_move service to safely move the arm
    to the center position.'''
    rospy.loginfo('Moving the arm to the center')
    
    # Request centered joint angle [1.57, 1.57]
    try:
      mover_resp = self.mover_client(1.57, 1.57)
    except rospy.ServiceException as exc:
      rospy.logerr(f'Service did not process request: ' + str(exc))
    
    rospy.loginfo('Done! ' + str(mover_resp))



if __name__ == '__main__':
  # Initialize the look_away node and create a handle to it
  rospy.init_node('look_away')
  
  # LookAway instance
  look_away = LookAway()
  #look_away.move_arm_center()

  # Handle ROS communication events
  rospy.spin()