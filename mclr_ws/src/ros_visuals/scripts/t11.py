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

from visualization_msgs.msg import Marker

def initNode():
  rospy.init_node('t11')
  rospy.loginfo('t11 node started')

  # init listener and broadcaster
  
  
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


def updateCageCenterPIN(o0_M, exp6_mot):
  o0_M = o0_M.act(exp6_mot)
  return o0_M 


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


def publishCageAsPIN(broadcaster, corners):
  for corner_id, corner in enumerate(corners):
    corner_name = "o_" + str(corner_id)
    if corner_id == 0:
      broadcaster.sendTransform(corner.translation, pin.Quaternion(corner.rotation).coeffs(), rospy.Time.now(), corner_name, 'world')
    else:
      broadcaster.sendTransform(corner.translation, pin.Quaternion(corner.rotation).coeffs(), rospy.Time.now(), corner_name, "o_0")
  return 0


def spawnMarker(marker_pose, marker_rgb, ref_frame, marker_pub):
  marker = Marker()

  marker.header.frame_id = ref_frame
  marker.header.stamp = rospy.Time.now()
  
  marker.id = Marker.CUBE
  
  marker.type = 0
  
  marker.action = Marker.ADD
  
  marker.pose.position.x = marker_pose[0]
  marker.pose.position.y = marker_pose[1]
  marker.pose.position.z = marker_pose[2]
  
  marker.pose.orientation.x = marker_pose[3]
  marker.pose.orientation.y = marker_pose[4]
  marker.pose.orientation.z = marker_pose[5]
  marker.pose.orientation.w = marker_pose[6]
  
  marker.scale.x = 0.1
  marker.scale.y = 0.01
  marker.scale.z = 0.05
  
  marker.color.a = 1.0
  marker.color.r = marker_rgb[0]
  marker.color.g = marker_rgb[1]
  marker.color.b = marker_rgb[2]
  
  marker.lifetime = rospy.Duration()
  
  marker_pub.publish(marker)
  
  return 0

def convertToWorld(marker_pose, t_world, q_world):
  # get the translation and rotation
  t_marker_o5 = np.array(marker_pose[0:3])
  q_marker_o5 = np.array(marker_pose[3:])

  # construct the transformation matrix
  T_marker_o5 = quat2mat(q_marker_o5)
  T_marker_o5[0:3, 3] = t_marker_o5
  # print("T_marker_o5", T_marker_o5)

  # get the o5 in world frame
  T_o5_world = quat2mat(q_world)
  T_o5_world[0:3, 3] = t_world
  # print("T_o5_world", T_o5_world)

  # get the marker in world frame
  T_marker_world = np.matmul(T_o5_world, T_marker_o5)

  # get the translation and rotation
  t_marker = T_marker_world[0:3, 3]
  q_marker = mat2quat(T_marker_world)

  return np.concatenate((t_marker, q_marker))

def main(args):
  # init node
  initNode()
  broadcaster = tf.TransformBroadcaster()
  listener = tf.TransformListener()
  # tfBuffer = tf2_ros.Buffer()
  # listener = tf2_ros.TransformListener(tfBuffer)

  marker_1_pub = rospy.Publisher('/vis_marker_1', Marker, queue_size=10)
  marker_2_pub = rospy.Publisher('/vis_marker_2', Marker, queue_size=10)

  center = np.array([1.0, 0.0, 1.0])
  center_euler = np.array([0.0, 0.0, 0.0])
  center_M = pin.SE3(pin.utils.rpyToMatrix(center_euler[0], center_euler[1], center_euler[2]), center)

  corners = genCage(center_M)

  # NOTE: TWIST DEFINED AND PROCESSED
  # define twist in world frame
  world_twist = np.array([0.01, 0.0, 0.0, 0.01, 0.0, 0.01])
  # convert twist to local frame
  local_twist = corners[0].actInv(pin.Motion(world_twist)).vector
  # generate a pin motion from twist
  exp6_mot = pin.exp6(pin.Motion(local_twist))

  # NOTE: MARKER DEFINED
  # assume the marker pose is in the 5th corner frame
  # format for marker pose: [x, y, z, r, p, y, w]
  marker_pose_corner = [0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
  
  # define updating rate
  rospy.loginfo('publishing cage tf...')
  rate = 10
  loop_rate = rospy.Rate(rate)
  while not rospy.is_shutdown():
    # NOTE: UNCOMMENT TO PUBLISH STATIC CAGE
    # publishCage(brodcaster)
    
    # NOTE: UNCOMMENT ANYONE TO PUBLISH MOVING CAGE BY UPDATING CENTER
    # pinocchio approach for E1.7
    corners[0] = updateCageCenterPIN(corners[0], exp6_mot)
    # formula approach for E1.7
    # corners[0] = updateCageCenterTwist(corners[0], local_twist, rate)
    
    publishCageAsPIN(broadcaster, corners)
    
    # get transform from world to 5th corner and spawn marker
    try:
      (trans_world, quat_world) = listener.lookupTransform('world', 'o_5', rospy.Time(0))
      # NOTE: CONVERT MARKER POSE TO WORLD FRAME FOR E1.9
      marker_pose_world = convertToWorld(marker_pose_corner, trans_world, quat_world)
      spawnMarker(marker_pose_world, [0, 1, 0], "world", marker_2_pub)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      continue
    # NOTE: SPAWN MARKER IN 5TH CORNER FRAME FOR E1.8
    spawnMarker(marker_pose_corner, [1, 0, 0], "o_5", marker_1_pub)
    loop_rate.sleep()
  return 0

if __name__ == '__main__':
  main(sys.argv)