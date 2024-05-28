import numpy as np
import numpy.linalg as la

# simulator (#TODO: set your own import path!)
from simulator.pybullet_wrapper import PybulletWrapper
from simulator.robot import Robot

# modeling
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

from enum import Enum

# ROS
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

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
    #TODO: Create RobotWrapper (fixed base), Call base class constructor, make publisher 
    urdf_path = "src/talos/talos_description/robots/talos_reduced.urdf"
    self._wrapper = pin.robot_wrapper.RobotWrapper.BuildFromURDF(urdf_path, [pin.utils.URDF_PATH])
    super().__init__(simulator,
                     filename=urdf_path,
                     model=self._wrapper.model,
                     basePosition=np.array([0, 0, 0]),
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
    joint_state_msg = JointState()
    joint_state_msg.header.stamp = rospy.Time.now()
    joint_state_msg.name = self.actuatedJointNames()
    joint_state_msg.position = self.actuatedJointPosition().tolist()
    joint_state_msg.velocity = self.actuatedJointVelocity().tolist()
    joint_state_msg.effort = [0.0] * len(joint_state_msg.name)  # Assuming zero effort for simplicity

    self.joint_state_publisher.publish(joint_state_msg)

################################################################################
# Controllers
################################################################################

class JointSpaceController:
    """JointSpaceController
    Tracking controller in jointspace
    """
    def __init__(self, robot, Kp, Kd):        
        # Save gains, robot ref
        None
    
    def update(self, q_r, q_r_dot, q_r_ddot):
        # Compute jointspace torque, return torque
        None
    
class CartesianSpaceController:
    """CartesianSpaceController
    Tracking controller in cartspace
    """
    def __init__(self, robot, joint_name, Kp, Kd):
        # save gains, robot ref
        None
        
    def update(self, X_r, X_dot_r, X_ddot_r):
        # compute cartesian control torque, return torque
        None

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
        
        self.q_init = np.zeros(32)

        # TODO: spawn robot

        ########################################################################
        # joint space spline: init -> home
        ########################################################################

        # TODO: create a joint spline 
        # TODO: create a joint controller
        
        ########################################################################
        # cart space: hand motion
        ########################################################################

        # TODO: create a cartesian controller
        
        ########################################################################
        # logging
        ########################################################################
        
        # TODO: publish robot state every 0.01 s to ROS
        self.t_publish = 0.0
        self.publish_period = 0.01
        
    def update(self, t, dt):
        
        # TODO: update the robot and model
        # self.robot.update()

        # update the controllers
        # TODO: Do inital jointspace, switch to cartesianspace control
        
        # command the robot
        # self.robot.setActuatedJointTorques(tau)
            
        # TODO: publish ros stuff
        
        None

def main():    
    env = Envionment()
    
    while not rospy.is_shutdown():
        t = env.simulator.simTime()
        dt = env.simulator.stepTime()
        
        env.update(t, dt)
        
        env.simulator.debug()
        env.simulator.step()
        
if __name__ == '__main__': 
    rospy.init_node('tutorial_3_robot_sim')
    main()
    