import numpy as np
import numpy.linalg as la

from simulator.pybullet_wrapper import PybulletWrapper
from simulator.robot import Robot

# modeling
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

from enum import Enum

# ROS
import rospy
import rospkg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

import sys
import os

# TODO: make the joint space controller always active to prevent robot from falling

################################################################################
# utility functions
################################################################################

class State(Enum):
    JOINT_SPLINE = 0,
    CART_SPLINE = 1

################################################################################
# Robot
################################################################################

class Talos(Robot):
  def __init__(self, simulator, q=None, verbose=True, useFixedBase=True):
    # TODO: Create RobotWrapper (fixed base), Call base class constructor, make publisher 
    description_path = rospkg.RosPack().get_path('talos_description')
    urdf_path = os.path.join(description_path, "robots/talos_reduced.urdf")
    path_meshes = os.path.join(description_path, "meshes/../..")

    self._wrapper = pin.RobotWrapper.BuildFromURDF(urdf_path, path_meshes, verbose = True)
    self._model = self._wrapper.model
    self._data = self._model.createData()

    super().__init__(simulator,
                     filename=urdf_path,
                     model=self._wrapper.model,
                     basePosition=np.array([0, 0, 1.15]),
                     baseQuationerion=np.array([0, 0, 0, 1]),
                     q=q,
                     useFixedBase=useFixedBase,
                     verbose=verbose)
    self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
      
  def update(self):
    # TODO: update base class, update pinocchio robot wrapper's kinematics
    super().update()
    
    # Update Pinocchio robot wrapper's kinematics
    self._wrapper.forwardKinematics(self.q(), self.v())
    self._wrapper.computeJointJacobians(self.q())
    self._wrapper.framesForwardKinematics(self.q())
    None
  
  def wrapper(self):
    return self._wrapper

  def data(self):
    return self._wrapper.data
  
  def publish(self):
    # TODO: publish robot state to ros
    # FIXME: modify the actuatedJointNames()
    joint_state_msg = JointState()
    joint_state_msg.header.stamp = rospy.Time.now()
    # joint_reindexing = [20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 0, 1, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 2, 3]
    joint_state_msg.name = self.actuatedJointNames()
    joint_state_msg.position = self.actuatedJointPosition().tolist()
    joint_state_msg.velocity = self.actuatedJointVelocity().tolist()
    joint_state_msg.effort = [0.0] * len(joint_state_msg.name)  # Assuming zero effort for simplicity

    self.joint_state_pub.publish(joint_state_msg)

################################################################################
# Controllers
################################################################################

class JointSpaceController:
  """JointSpaceController
  Tracking controller in jointspace
  """
  def __init__(self, robot, Kp, Kd):        
    # Save gains, robot ref
    self.Kp = Kp
    self.Kd = Kd
    self.robot = robot
    
  def update(self, q_r, q_r_dot, q_r_ddot):
    # Compute jointspace torque, return torque
    e_q = q_r - self.robot._q
    e_v = q_r_dot - self.robot._v
    M = pin.crba(self.robot._model, self.robot._data, self.robot._q) 
    tau = M.dot(q_r_ddot + self.Kd.dot(e_v) + self.Kp.dot(e_q))
    # print("tau", tau)
    # self.robot.setActuatedJointTorques(tau)
    
    return tau
    
class CartesianSpaceController:
    """CartesianSpaceController
    Tracking controller in cartspace
    """
    def __init__(self, robot, joint_name, Kp, Kd):
        # save gains, robot ref
        self.Kp = Kp
        self.Kd = Kd
        self.robot = robot
        self.joint_name = joint_name
        None
        
    def update(self, X_r, X_dot_r, X_ddot_r):
        # compute cartesian control torque, return torque
        joint_id = self.robot._model.getJointId(self.joint_name)
        jacobian = pin.computeJointJacobian(self.robot._model, self.robot._data, self.robot._q, joint_id)
        jacobian_inv = la.pinv(jacobian)

        q_curr = self.robot._data.oMi[joint_id]
        v_curr = jacobian @ self.robot._v

        v_err = v_curr - X_dot_r

        a_des = X_ddot_r - self.Kd @ v_err - self.Kp @ (pin.log(q_curr.inverse() * X_r).vector)

        M = pin.crba(self.robot._model, self.robot._data, self.robot._q)

        tau = M.dot(jacobian_inv @ a_des - jacobian_inv @ v_curr)

        return tau

################################################################################
# Application
################################################################################
    
class Envionment:
    def __init__(self):        
        # state
        self.cur_state = State.JOINT_SPLINE
        
        # create simulation
        self.simulator = PybulletWrapper()
        
        ########################################################################
        # spawn the robot
        ########################################################################
        self.q_home = np.zeros(32)
        self.q_home[14:22] = np.array([0, +0.45, 0, -1, 0, 0, 0, 0 ])
        self.q_home[22:30] = np.array([0, -0.45, 0, -1, 0, 0, 0, 0 ])
        self.vel_home = np.zeros(32)  
        self.acc_home = np.zeros(32)
        
        self.q_init = np.zeros(32)

        # TODO: spawn robot
        self.robot = Talos(self.simulator, q=self.q_init)

        ########################################################################
        # joint space spline: init -> home
        ########################################################################

        # TODO: create a joint spline 
        # TODO: create a joint controller
        
        ########################################################################
        # cart space: hand motion
        ########################################################################

        # TODO: create a cartesian controller
        self.Kp = np.eye(32)
        self.Kd = np.eye(32)
        # left leg
        self.Kp[0: 6, 0: 6] = 1500 * np.eye(6)
        # right leg
        self.Kp[6: 12, 6: 12] = 1500 * np.eye(6)
        # torso
        self.Kp[12: 14, 12: 14] = 500 * np.eye(2)
        # left arm
        self.Kp[14: 22, 14: 22] = 400 * np.eye(8)
        # right arm
        self.Kp[22: 30, 22: 30] = 400 * np.eye(8)
        # head
        self.Kp[30: 32, 30: 32] = 300 * np.eye(2)

        self.Kp_cart = 50 * np.eye(6)
        self.Kd_cart = 1500 * np.eye(6)
        
        ########################################################################
        # logging
        ########################################################################
        
        # TODO: publish robot state every 0.01 s to ROS
        self.t_publish = 0.0
        self.publish_period = 0.01

        self.target_joint = "arm_right_7_joint"
        self.target_pose = None
    
    def markerCallback(self, msg):
        pose_array = [msg.pose.position.x, 
                      msg.pose.position.y, 
                      msg.pose.position.z, 
                      msg.pose.orientation.x, 
                      msg.pose.orientation.y, 
                      msg.pose.orientation.z, 
                      msg.pose.orientation.w]
        self.target_pose = pin.XYZQUATToSe3(pose_array)
        
    def update(self, t, dt):

      # TODO: update the robot and model
      self.robot.update()
      duration = 5

      # update the controllers
      # TODO: Do inital jointspace, switch to cartesianspace control
      if self.cur_state == State.JOINT_SPLINE:
        self.controller = JointSpaceController(self.robot, self.Kp, self.Kd)
        if t < duration:
          if t == 0:
            q_prev = self.q_init
          else:
            q_prev = pin.interpolate(self.robot._model, self.q_init, self.q_home, (t- dt)/duration)
          q_curr = pin.interpolate(self.robot._model, self.q_init, self.q_home, t/duration)
          q_next = pin.interpolate(self.robot._model, self.q_init, self.q_home, (t+ dt)/duration)
          v_curr = (q_curr - q_prev)/dt
          v_next = (q_next - q_curr)/dt
          a_curr = (v_next - v_curr)/dt
          tau_curr = self.controller.update(q_curr, v_curr, a_curr)

        else:
          self.controller = CartesianSpaceController(self.robot, self.target_joint, self.Kp_cart, self.Kd_cart)
          target_joint_id = self.robot._model.getJointId(self.target_joint)

          self.target_pose = self.controller.robot._data.oMi[target_joint_id]
          rospy.Subscriber('/marker_pose', PoseStamped, self.markerCallback)
          
          target_vel = np.zeros(6)
          target_acc = np.zeros(6)
          tau_curr = self.controller.update(self.target_pose, target_vel, target_acc)

        # command the robot
        self.robot.setActuatedJointTorques(tau_curr)
            
        # TODO: publish ros stuff
        self.robot.publish()
        
        

def main():    
    env = Envionment()
    print("env established")
    
    while not rospy.is_shutdown():
        t = env.simulator.simTime()
        dt = env.simulator.stepTime()
        
        env.update(t, dt)
        
        env.simulator.debug()
        env.simulator.step()
        
if __name__ == '__main__': 
    rospy.init_node('tutorial_3_robot_sim')
    main()
    