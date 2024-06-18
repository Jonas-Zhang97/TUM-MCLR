import numpy as np
from numpy import nan
from numpy.linalg import norm as norm
import matplotlib.pyplot as plt
import csv
import os
import rospkg

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
import rospkg

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
    # reset data file
    csv_file = os.path.join(rospkg.RosPack().get_path('bullet_sims'), 'doc/data.csv')
    f = open(csv_file, "w")
    f.truncate()
    title_writer = csv.writer(f)
    title_writer.writerow(["time", "reference_height", "computed_height", "simulator_height"])
    f.close()

    # TODO init TSIDWrapper
    tsid = TSIDWrapper(conf)
    # TODO init Simulator
    simulator = PybulletWrapper()
    # TODO init ROBOT
    robot = Talos(simulator=simulator, urdf=conf.urdf, model=tsid.model, q=conf.q_home, verbose=True, useFixedBase=False)
    
    t_publish = 0.0
    t_plot = 0.0

    # set flags
    com_reset = False
    lf_reset = False
    motion_task_set = False
    
    # set param for squatting
    rl_amplitude = 0.05         # m
    rl_period = 2 * np.pi * 0.5 # s

    rh_circle_center = np.array([0.4, -0.2, 1.1])
    rh_circle_radius = 0.2
    rh_circle_omega = 2 * np.pi * 0.1

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
        if t > 1. and not com_reset:
            # get current COM height
            com_height = tsid.comState().pos()[2]
            # get right foot position
            rf_position = tsid.get_placement_RF().translation
            # move COM position
            tsid.setComRefState(np.array([rf_position[0], rf_position[1], com_height]))
            com_reset = True
        
        # wait for another sec for stability, perform one leg standing
        if t > 3. and not lf_reset:
            tsid.remove_contact_LF()
            # get current left foot position
            lf_pose_target = tsid.get_placement_LF()
            lf_pose_target.translation[2] = 0.3
            # set new left foot position
            tsid.set_LF_pose_ref(lf_pose_target)
            lf_reset = True
            
            # get a reference for subequent tasks
            com_position_ref = tsid.comState().pos()
            rh_pose_ref = tsid.get_placement_RF()

        # after 4 sec, start squatting
        if t > 4.: 
            # get current COM height
            height_target = com_position_ref[2] + rl_amplitude * np.sin(rl_period * (t - 4))
            vel = rl_amplitude * rl_period * np.cos(rl_period * (t - 4))
            acc = - rl_amplitude * rl_period**2 * np.sin(rl_period * (t - 4))
            # set new COM position
            tsid.setComRefState(np.array([com_position_ref[0], com_position_ref[1], height_target]), np.array([0, 0, vel]), np.array([0, 0, acc]))
            None

        # after 8 sec, start moving right hand in a circle in Y-Z plane
        if t > 8.:
            if not motion_task_set:
                motion_task_set = True
                tsid.formulation.addMotionTask(tsid.rightHandTask, conf.w_hand, 1, 0.0001)
            # Compute the desired position, velocity and acceleration
            rh_pose_target = rh_pose_ref
            rh_pose_target.translation = rh_circle_center + np.array([0, rh_circle_radius * np.cos(rh_circle_omega * (t - 8)), rh_circle_radius * np.sin(rh_circle_omega * (t - 8))])
            rh_velocity_target = np.array([0, - rh_circle_radius * rh_circle_omega * np.sin(rh_circle_omega * (t - 8)), rh_circle_radius * rh_circle_omega * np.cos(rh_circle_omega * (t - 8))])
            rh_acceleration_target = np.array([0, - rh_circle_radius * rh_circle_omega**2 * np.cos(rh_circle_omega * (t - 8)), - rh_circle_radius * rh_circle_omega**2 * np.sin(rh_circle_omega * (t - 8))])
            
            tsid.set_RH_pose_ref(rh_pose_target, rh_velocity_target, rh_acceleration_target)
            
            



        # publish to ros
        if t - t_plot > 1./50.:
            # plot required data
            t_plot = t
            '''
            [t, tsid.comReference().pos()[2], tsid.comState().pos()[2], robot.baseWorldPosition()[2]
            '''
            with open(csv_file, 'a') as f:
                writer = csv.writer(f)  # Note: writes lists, not dicts.
                writer.writerow([t, tsid.comReference().pos()[2], tsid.comState().pos()[2], robot.baseWorldPosition()[2]])
            
        if t - t_publish > 1./30.:
            t_publish = t
            # get current BASE Pose
            T_b_w = tsid.baseState()
            robot.publish(T_b_w)
    
if __name__ == '__main__': 
    rospy.init_node('tutorial_4_one_leg_stand')
    main()
