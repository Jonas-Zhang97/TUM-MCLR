import numpy as np
from numpy import nan
from numpy.linalg import norm as norm
import matplotlib.pyplot as plt

# pinocchio
import pinocchio as pin

# simulator
import pybullet as pb
from simulator.pybullet_wrapper import PybulletWrapper
from simulator.robot import Robot

# robot and controller
from tutorial_4.tsid_wrapper import TSIDWrapper
import tutorial_4.config as conf

# ROS
import rospy
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

################################################################################
# settings
################################################################################

DO_PLOT = True

################################################################################
# Robot
################################################################################

class Talos(Robot):
    def __init__(self, simulator, urdf, model, q=None, verbose=True, useFixedBase=True):
        # TODO call base class constructor
        super().__init__(simulator,
                         urdf,
                         model,
                         basePosition=np.array([0, 0, 1.11]),
                         baseQuationerion=np.array([0, 0, 0, 1]),
                         q=q,
                         useFixedBase=useFixedBase,
                         verbose=verbose)
        # TODO add publisher
        self.joint_state_pub = rospy.Publisher("/joint_states",JointState,queue_size=10)
        # TODO add tf broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()

    def update(self):
        # TODO update base class
        super().update()
    
    def publish(self, T_b_w):
        # TODO publish jointstate
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = self.actuatedJointNames()
        joint_state_msg.position = self.actuatedJointPosition().tolist()
        joint_state_msg.velocity = self.actuatedJointVelocity().tolist()
        joint_state_msg.effort = [0.0] * len(joint_state_msg.name)  # Assuming zero effort for simplicity

        self.joint_state_pub.publish(joint_state_msg)
        # TODO broadcast transformation T_b_wl
        # T_b_w = [position, quaternion]
        tran = T_b_w[0].translation
        quat = tf.transformations.quaternion_from_matrix(T_b_w[0].homogeneous)
        self.tf_broadcaster.sendTransform(tran, quat, rospy.Time.now(), "base_link", "world")
################################################################################
# main
################################################################################

def main(): 
    # TODO init TSIDWrapper
    tsid = TSIDWrapper(conf)
    # TODO init Simulator
    simulator = PybulletWrapper()
    # TODO init ROBOT
    robot = Talos(simulator=simulator, urdf=conf.urdf, model=tsid.model, q=conf.q_home, verbose=True, useFixedBase=False)
    
    t_publish = 0.0

    # set flag for COM (center of mass) and LF(left foot) reset
    com_reset = False
    lf_reset = False
    

    while not rospy.is_shutdown():

        # elaped time
        t = simulator.simTime()

        # TODO update the simulator and the robot
        simulator.step()
        robot.update()
        
        # TODO update TSID controller
        tau_sol, _ = tsid.update(robot._q, robot._v, t)

        # command to the robot
        robot.setActuatedJointTorques(tau_sol)

        # wait for robot steady state
        if t > 2.0 and not com_reset:
            # get current COM height
            com_height = tsid.comState().pos()[2]
            # get right foot position
            rf_position = tsid.get_placement_RF().translation
            # move COM position
            tsid.setComRefState(np.array([rf_position[0], rf_position[1], com_height]))
            com_reset = True
        
        # wait for another 3 sec, perform one leg standing
        if t > 4.0 and not lf_reset:
            tsid.remove_contact_LF()
            # get current left foot position
            lf_pose_target = tsid.get_placement_LF()
            lf_pose_target.translation[2] = 0.3
            # set new left foot position
            tsid.set_LF_pose_ref(lf_pose_target)
            lf_reset = True


        # publish to ros
        if t - t_publish > 1./30.:
            t_publish = t
            # get current BASE Pose
            T_b_w = tsid.baseState()
            robot.publish(T_b_w)
    
if __name__ == '__main__': 
    rospy.init_node('tutorial_4_one_leg_stand')
    main()
