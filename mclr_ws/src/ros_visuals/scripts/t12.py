#!/usr/bin/python3.8

import os
import sys
# ros
import rospy
# numpy
import numpy as np
# tf
import tf
from tf.transformations import euler_from_matrix as mat2eul
from tf.transformations import euler_from_quaternion as quat2eul
# pinocchio
import pinocchio as pin

from visualization_msgs.msg import Marker

def initNode():
  rospy.init_node('t11')
  rospy.loginfo('t11 node started')

  # init listener and broadcaster
  
  
  return 0

def initCage():
# def the center of the cage
  o0_t = np.array([0.5, 0.0, 1])
  o0_r = pin.utils.rpyToMatrix(0.0, 3.14/4, 0.0)
  # convert cage center to SE3 object
  o0_M = pin.SE3(o0_r, o0_t)



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
  
  corners = [o0_M]
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

def convertMarkerFromCenterToWorld(marker_pose_world, exp6_mot):
  # convert marker pose into se3 object
  rot_eul = quat2eul(marker_pose_world[3:])
  rot_mat = pin.utils.rpyToMatrix(rot_eul[0], rot_eul[1], rot_eul[2])
  tran_vec = np.array(marker_pose_world[0:3])
  marker_pose_se3 = pin.SE3(rot_mat, tran_vec)
  # convert marker pose from corner to world
  marker_pose_world = marker_pose_se3.act(exp6_mot)
  # convert rotation to quaternion
  rot_eul = mat2eul(marker_pose_world.rotation)
  # convert to marker pose
  marker_pose_world = [marker_pose_world.translation[0], marker_pose_world.translation[1], marker_pose_world.translation[2], 
                       pin.Quaternion(marker_pose_world.rotation).coeffs()[0], pin.Quaternion(marker_pose_world.rotation).coeffs()[1], pin.Quaternion(marker_pose_world.rotation).coeffs()[2], pin.Quaternion(marker_pose_world.rotation).coeffs()[3]]
  return marker_pose_world

# Exercise 2.2
def twistTransform(twist, corner_id, corners):
  # get the twist in local frame
  local_twist = corners[corner_id].actInv(pin.Motion(twist)).vector
  return local_twist

def main(args):
  # init node
  initNode()
  broadcaster = tf.TransformBroadcaster()
  listener = tf.TransformListener()
  marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

  corners = initCage()

  # define twist in world frame
  world_twist = np.array([0.01, 0.0, 0.0, 0.0, 0.0, 0.1])
  # convert twist to local frame
  local_twist = twistTransform(world_twist, 3, corners)
  # generate a pin motion from twist
  exp6_mot = pin.exp6(pin.Motion(local_twist))

  marker_pose_corner = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
  marker_pose_world = marker_pose_corner
  
  # define updating rate
  rospy.loginfo('publishing cage tf...')
  rate = 10
  loop_rate = rospy.Rate(rate)
  while not rospy.is_shutdown():
    # publish transform from world to cage center
    # publishCage(brodcaster)
    corners[0] = updateCageCenterPIN(corners[0], exp6_mot)
    # corners[0] = updateCageCenterTwist(corners[0], local_twist, rate)
    publishCageAsPIN(broadcaster, corners)
    spawnMarker(marker_pose_corner, [1, 0, 0], "o_5", marker_pub)
    marker_pose_world = convertMarkerFromCenterToWorld(marker_pose_world, exp6_mot)
    spawnMarker(marker_pose_world, [0, 1, 0], "world", marker_pub)
    loop_rate.sleep()
  return 0

if __name__ == '__main__':
  main(sys.argv)