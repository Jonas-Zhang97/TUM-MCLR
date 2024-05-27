#!/usr/bin/python3.8

import os
import sys
# ros
import rospy
# numpy
import numpy as np
# tf
import tf
from tf.transformations import quaternion_from_euler as euler2quat
# pinocchio
import pinocchio as pin

def main(args):
  # init node
  rospy.init_node('t11')
  rospy.loginfo('t11 node started')
  rospy.loginfo('working directory: %s', os.getcwd())
  py_paths = sys.path
  rospy.loginfo('pinoccio version: %s', pin.__version__)
  dirs = dir(pin)

  # init listener and broadcaster
  listener = tf.TransformListener()
  brodcaster = tf.TransformBroadcaster()

  # define updating rate
  rate = rospy.Rate(1.0)

  rospy.loginfo('publishing transform from world to map')

  while not rospy.is_shutdown():
    # publish transform from world to cage center
    brodcaster.sendTransform((1.0, 0.0, 1.0), euler2quat(0.0, 0.0, 0.0), rospy.Time.now(), 'world', 'center')
    twist = np.array([0.5, 0.0, 0.0, 0.0, 0.0, 1.0])
    transformation_matrix = pin.exp6(twist)
    rospy.loginfo('transformation_matrix', transformation_matrix)
    rate.sleep()
  return 0

if __name__ == '__main__':
  main(sys.argv)