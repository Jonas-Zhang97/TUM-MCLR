#!/usr/bin/python3.8

import os
import sys
# ros
import rospy
# numpy
import numpy as np
# tf
import tf2_ros
import tf
from tf.transformations import euler_from_matrix as mat2eul
from tf.transformations import quaternion_matrix as quat2mat
from tf.transformations import euler_from_quaternion as quat2eul
from tf.transformations import quaternion_from_matrix as mat2quat
# pinocchio
import pinocchio as pin

from geometry_msgs.msg import TwistStamped
from visualization_msgs.msg import Marker

def initNode():
  rospy.init_node('t12')
  rospy.loginfo('t12 node started')

  return 0

def genCage(center_M):

  # def the relative poses of the corners
  # translations
  corner_tran = [
    [-0.5, -0.3, -0.4],
    [0.5, -0.3, -0.4],
    [0.5, 0.3, -0.4],
    [-0.5, 0.3, -0.4],
    [-0.5, -0.3, 0.4],
    [0.5, -0.3, 0.4],
    [0.5, 0.3, 0.4],
    [-0.5, 0.3, 0.4]
  ]
  # euler angles
  o_eul_ang = [
    [0.0, 0.0, 0.0],
    [0.0, 0.0, 1.57],
    [0.0, 0.0, 3.14],
    [0.0, 0.0, -1.57],
    [-1.57, 0.0, 0.0],
    [3.14, 0.0, 3.14],
    [3.14, 0.0, -1.57],
    [3.14, 0.0, 0.0]
  ]
  # construct the cage corners as SE3 objects
  
  corners = [center_M]
  for corner_id in range(8):
    # get the translation and euler angles
    corner_t = np.array(corner_tran[corner_id])
    corner_r = pin.utils.rpyToMatrix(o_eul_ang[corner_id][0], o_eul_ang[corner_id][1], o_eul_ang[corner_id][2])
    # convert to SE3 object
    corner_M = pin.SE3(corner_r, corner_t)
    corners.append(corner_M)
  
  return corners


def updateCorners(corners, ref_corner_id):
  for corner_id, corner in enumerate(corners):
    if corner_id == ref_corner_id:
      corners[corner_id] = corners[ref_corner_id] * (corners[ref_corner_id].inverse() * corners[corner_id])
  return corners


def updateCageCenterTwist(o0_M, twist, rate):
  # get the translation and rotation
  T = o0_M.np
  t = T[0:3, 3]
  R = T[0:3, 0:3]
  euler = mat2eul(R)
  
  # get the linear and angular velocities
  vel_lin = twist[0:3]
  vel_ang = twist[3:6]

  # get the time step
  dt = 1.0 / rate

  # update the translation and rotation
  t+= vel_lin * dt
  euler += vel_ang * dt
  
  # update the SE3 object
  R = pin.utils.rpyToMatrix(euler[0], euler[1], euler[2])
  T[0:3, 3] = t
  T[0:3, 0:3] = R
  return pin.SE3(T)


def publishCageAsPIN(broadcaster, corners, o_ref):
  for corner_id, corner in enumerate(corners):
    corner_name = "o_" + str(corner_id)
    if corner_name == o_ref:
      broadcaster.sendTransform(corner.translation, pin.Quaternion(corner.rotation).coeffs(), rospy.Time.now(), corner_name, 'world')
    else:
      broadcaster.sendTransform(corner.translation, pin.Quaternion(corner.rotation).coeffs(), rospy.Time.now(), corner_name, o_ref)
  return 0

def convertTwistRefFrame(twist, trans, quat):
  """
  args:
    twist: numpy array of shape (6,), twist in current frame
    trans: numpy array of shape (3,), translation of the reference frame
    quat: numpy array of shape (4,), quaternion of the reference frame
  output:
    twist_ref: numpy array of shape (6,)
  """
  # define basic mathmatic operations
  def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
  
  def constructAdT(T):
    AdT = np.zeros((6, 6))
    AdT[0:3, 0:3] = T[0:3, 0:3]
    AdT[3:6, 3:6] = T[0:3, 0:3]
    AdT[3:6, 0:3] = np.dot(skew(T[0:3, 3]), T[0:3, 0:3])
    return AdT
  
  def constructT(trans, quat):
    T = quat2mat(quat)
    T[0:3, 3] = trans
    return T
  
  T = constructT(trans, quat)
  AdT = constructAdT(T)
  twist_ref = np.dot(AdT, twist)

  return twist_ref

def applyTwistInFrame(motion, corner_id, corners):
  '''
  args:
    motion: pin.Motion object, twist in the an desired frame
    corner_id: int, id of the corner frame where the twist is applied, -1 stands for world frame
    corners: list of SE3 objects, corners of the cage
  '''
  if corner_id == 0:
    exp6_mot_0 = pin.exp6(motion)
    corners[0] = corners[0].act(exp6_mot_0)
  elif corner_id == -1:
    motion_0 = corners[0].inverse() * motion
    exp6_mot_0 = pin.exp6(motion_0)
    corners[0] = corners[0].act(exp6_mot_0)
  else:
    motion_0 = corners[corner_id] * motion
    exp6_mot_0 = pin.exp6(motion_0)
    corners[0] = corners[0].act(exp6_mot_0)
  return corners

def publishTwist(motion, frame_id, twist_pub):
  # Create a TwistStamped message and fill in its values
  msg = TwistStamped()
  msg.header.frame_id = frame_id
  msg.header.stamp = rospy.Time.now()
  msg.twist.angular.x = motion.angular[0]
  msg.twist.angular.y = motion.angular[1]
  msg.twist.angular.z = motion.angular[2]
  msg.twist.linear.x = motion.linear[0]
  msg.twist.linear.y = motion.linear[1]
  msg.twist.linear.z = motion.linear[2]
  # publish the twist
  twist_pub.publish(msg)

  return 0

def main(args):
  # init node
  initNode()

  broadcaster = tf.TransformBroadcaster()
  listener = tf.TransformListener()

  o_0_twist_pub = rospy.Publisher('/twist_o_0', TwistStamped, queue_size=10)
  o_w_twist_pub = rospy.Publisher('/twist_world', TwistStamped, queue_size=10)
  o_5_twist_pub = rospy.Publisher('/twist_o_5', TwistStamped, queue_size=10)
  # tfBuffer = tf2_ros.Buffer()
  # listener = tf2_ros.TransformListener(tfBuffer)

  center = np.array([1.0, 0.0, 1.0])
  center_euler = np.array([0.0, 0.0, 0.0])
  center_M = pin.SE3(pin.utils.rpyToMatrix(center_euler[0], center_euler[1], center_euler[2]), center)

  corners = genCage(center_M)

  # init publish
  publishCageAsPIN(broadcaster, corners, o_ref="o_0")

  # define twist frame
  twist = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.01])
  # convert twist to motion
  motion = pin.Motion(twist)
  
  # define updating rate
  rospy.loginfo('publishing cage tf...')
  rate = 10
  loop_rate = rospy.Rate(rate)
  while not rospy.is_shutdown():
    # update the cage center
    # treat the twist as a motion in the world frame
    corners = applyTwistInFrame(motion, -1, corners)
    # treat the twist as a motion in the center frame
    corners = applyTwistInFrame(motion, 0, corners)
    # treat the twist as a motion in the corner 5 frame
    corners = applyTwistInFrame(motion, 5, corners)
    # publish the cage
    publishTwist(motion, 'world', o_w_twist_pub)
    publishTwist(motion, 'o_0', o_0_twist_pub)
    publishTwist(motion, 'o_5', o_5_twist_pub)
    publishCageAsPIN(broadcaster, corners, o_ref="o_0")
    loop_rate.sleep()
  return 0

if __name__ == '__main__':
  main(sys.argv)