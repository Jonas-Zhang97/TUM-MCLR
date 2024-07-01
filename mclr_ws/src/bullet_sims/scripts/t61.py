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
import tf.transformations
from tutorial_4.tsid_wrapper import TSIDWrapper
import tutorial_4.config as conf

# ROS
import rospy
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, TransformStamped

import rospkg
import csv
import os

import tsid


################################################################################
# settings
################################################################################

DO_PLOT = True

################################################################################
# Robot
################################################################################

class Talos(Robot):
    '''
    Robot class for Talos robot
     __init__(self, 
              simulator,
              filename,
              model, 
              basePosition=np.array([0,0,0]), 
              baseQuationerion=np.array([0,0,0,1]), 
              q=None, 
              useFixedBase=True, 
              verbose=True,
              baseFrameName=None)
    '''
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
        self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
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
        # TODO broadcast transformation T_b_w
        # T_b_w = [position, quaternion]
        tran = T_b_w[0].translation
        quat = tf.transformations.quaternion_from_matrix(T_b_w[0].homogeneous)
        self.tf_broadcaster.sendTransform(tran, quat, rospy.Time.now(), "base_link", "world")

def applyForce(force, robot, t, t_period=4., t_duration=0.5):
    '''
    Apply force to the robot for a certain duration
    args:
        force: amplitude of the force, in Newton
        robot: robot object
        t: current time
    '''
    if t_period < t < t_period + t_duration:
        # from right
        robot.applyForce([0, force, 0])
    elif 2*t_period + t_duration < t < 2* t_period + 2 * t_duration:
        # from left
        robot.applyForce([0, -force, 0])
        pass
    elif 3 * t_period + 2 * t_duration < t < 3 * t_period + 3 * t_duration:
        # from back
        robot.applyForce([force, 0, 0])
        pass
        
# TODO: implement the following functions
class GroundReferencePoints():
    def __init__(self, wr_lankel, wr_rankel, H_w_lankle, H_w_rankle, H_w_lsole, H_w_rsole, d, com_state):
        '''
        GroundReferencePoints class
        args:
            wr_lankel: wrench of the left ankle
            wr_rankel: wrench of the right ankle
            H_w_lankel: Homogeneous matrix of the left ankle
            H_w_rankel: Homogeneous matrix of the right ankle
            d: distance between the two ankles
            com_state: state of the center of mass
        '''
        self.wr_lankel = wr_lankel
        self.wr_rankel = wr_rankel

        self.d = d

        self.H_w_lankle = H_w_lankle
        self.H_w_rankle = H_w_rankle

        self.H_w_lsole = H_w_lsole
        self.H_w_rsole = H_w_rsole

        self.com_state = com_state

        self.zmp = np.zeros(3)
        self.cmp = np.zeros(3)
        self.cp = np.zeros(3)

    def update(self):
        '''
        Update the reference points
        returns:
            [zmp, cmp, cp]
        '''
        self.estimateZMP()
        self.estimateCMP()
        self.estimateCP()
        return [self.zmp, self.cmp, self.cp]

    def estimateZMP(self):
        '''
        Estimate the Zero Moment Point (ZMP)
        '''
        p_lx = (-self.wr_lankel.angular[1] - self.wr_lankel.linear[0] * self.d) / self.wr_lankel.linear[2]
        p_ly = (self.wr_lankel.angular[0] - self.wr_lankel.linear[1] * self.d) / self.wr_lankel.linear[2]
        p_l = np.array([p_lx, p_ly, 0])
        p_l_w = self.H_w_lsole.inverse() * p_l

        p_rx = (-self.wr_rankel.angular[1] - self.wr_rankel.linear[0] * self.d) / self.wr_rankel.linear[2]
        p_ry = (self.wr_rankel.angular[0] - self.wr_rankel.linear[1] * self.d) / self.wr_rankel.linear[2]
        p_r = np.array([p_rx, p_ry, 0])
        p_r_w = self.H_w_rsole.inverse() * p_r
        
        self.zmp[0] = (p_r_w[0] * self.wr_rankel.linear[2] + p_l_w[0] * self.wr_lankel.linear[2]) / (self.wr_rankel.linear[2] + self.wr_lankel.linear[2])
        self.zmp[1] = (p_r_w[1] * self.wr_rankel.linear[2] + p_l_w[1] * self.wr_lankel.linear[2]) / (self.wr_rankel.linear[2] + self.wr_lankel.linear[2])

    def estimateCMP(self):
        '''
        Estimate the Centroidal Moment Pivot (CMP)
        '''
        # Homogeneous matrix from world to ZMP
        H_w_zmp = pin.SE3.Identity()
        H_w_zmp.translation = self.zmp

        # Homogeneous matrix from world to left and right ankle
        H_zmp_lankle = H_w_zmp.inverse() * self.H_w_lankle
        H_zmp_rankle = H_w_zmp.inverse() * self.H_w_rankle

        wr_p = H_zmp_lankle * self.wr_lankel + H_zmp_rankle * self.wr_rankel

        self.cmp[0] = self.com_state.pos()[0] - (wr_p.linear[0] / wr_p.linear[2]) * self.com_state.pos()[2]
        self.cmp[1] = self.com_state.pos()[1] - (wr_p.linear[1] / wr_p.linear[2]) * self.com_state.pos()[2]

    def estimateCP(self):
        '''
        Estimate the Capture Point (CP)
        '''
        x_p = [self.com_state.pos()[0], self.com_state.pos()[1], 0]
        x_p_dot = [self.com_state.vel()[0], self.com_state.vel()[1], 0]
        
        self.cp = x_p + x_p_dot / np.sqrt(9.81 / self.com_state.pos()[2])
    
class BalanceController():
    def __init__(self, tw):
        '''
        BalanceController class
        args:
            com_state: state of the center of mass
            tw: TSIDWrapper object
            x_ref: reference position of the CoM
            x_ref_dot: reference velocity of the CoM (only available when the robot is walking)
        '''
        self.tw = tw
        self.x_offset = np.zeros(3)

    def ankleBalencingStategy(self, x, x_ref, K_x, K_p, p, dt, p_ref=np.zeros(3), x_ref_dot=np.zeros(3)):
        '''
        Use hip strategy to balance the robot
        args:
            x: current CoM position
            x_ref: reference CoM position
            K_x: CoM gain
            K_p: ZMP gain
            p: current ZMP position
            p_ref: reference ZMP position
            x_ref_dot: reference CoM velocity
        '''
        # compute the desired CoM position
        x_d_dot = x_ref_dot - K_x * (x - x_ref) + K_p * (p - p_ref)
        self.x_offset += x_d_dot * dt
        x_d = x_ref + self.x_offset

        # create sampler
        ankle_traj = tsid.TrajectorySample(3, 3)
        # compute the reference trajectory
        ankle_traj.pos(x_d)
        ankle_traj.vel(x_d_dot)

        # set the reference trajectory
        self.tw.amTask.setReference(ankle_traj)
    
    def hipBalencingStategy(self, r, K_Gamma, r_ref=np.zeros(3)):
        '''
        Use hip strategy to balance the robot
        args:
            r: current CMP position
            K_Gamma: CMP gain
            r_ref: reference CMP position, no need for standing tasks
        '''
        # compute desired angular momentum
        Gamma_d = r_ref + K_Gamma * (r - r_ref)

        # create sampler
        hip_traj = tsid.TrajectorySample(3)
        # compute the reference trajectory
        hip_traj.pos(Gamma_d)

        # set the reference trajectory
        self.tw.amTask.setReference(hip_traj)

################################################################################
# main
################################################################################

def main(strategy): 
    # reset data file
    csv_file = os.path.join(rospkg.RosPack().get_path('bullet_sims'), 'doc/grp.csv')
    file = open(csv_file, "w")
    file.truncate()
    title_writer = csv.writer(file)
    title_writer.writerow(["time", 
                           "zmp_x", "zmp_y", "zmp_z", 
                           "cmp_x", "cmp_y", "cmp_z", 
                           "cp_x", "cp_y", "cp_z"])
    file.close()

    # TODO init TSIDWrapper
    tw = TSIDWrapper(conf)
    # TODO init Simulator
    simulator = PybulletWrapper()
    # TODO init ROBOT
    '''
    Talos init method
    __init__(self, simulator, urdf, model, q=None, verbose=True, useFixedBase=True)
    '''
    # robot = Talos(simulator, conf.urdf, tw.model, q=conf.q_home)
    robot = Talos(simulator=simulator, urdf=conf.urdf, model=tw.model, q=conf.q_home, verbose=True, useFixedBase=False)
    
    t_publish = 0.0
    t_plot = 0.0

    # add sensors
    pb.enableJointForceTorqueSensor(robot.id(), robot.jointNameIndexMap()['leg_right_6_joint'], True)
    pb.enableJointForceTorqueSensor(robot.id(), robot.jointNameIndexMap()['leg_left_6_joint'], True)

    bc = BalanceController(tw)

    x_ref = tw.comState().pos()

    info_prompted = False

    while not rospy.is_shutdown():

        # elaped time
        t = simulator.simTime()
        dt = simulator.stepTime()

        # TODO: update the simulator and the robot
        simulator.step()
        robot.update()

        # right ankle wrench
        wren = pb.getJointState(robot.id(), robot.jointNameIndexMap()['leg_right_6_joint'])[2]
        wnp = np.array([-wren[0], -wren[1], -wren[2], -wren[3], -wren[4], -wren[5]])
        wr_rankle = pin.Force(wnp)
        # left ankle wrench
        wlen = pb.getJointState(robot.id(), robot.jointNameIndexMap()['leg_left_6_joint'])[2]
        f = wr_rankle.linear
        wnp = np.array([-wlen[0], -wlen[1], -wlen[2], -wlen[3], -wlen[4], -wlen[5]])
        wl_rankle = pin.Force(wnp)
        # get transformation matrices
        data = robot._model.createData()
        pin.framesForwardKinematics(robot._model, data, robot._q)
        H_w_lsole = data.oMf[robot._model.getFrameId("left_sole_link")]
        H_w_rsole = data.oMf[robot._model.getFrameId("right_sole_link")]
        H_w_lankle = data.oMf[robot._model.getFrameId("leg_left_6_joint")]
        H_w_rankle = data.oMf[robot._model.getFrameId("leg_right_6_joint")]
        # get d
        d = H_w_lankle.translation[2]
        # get CoM state
        com_state = tw.comState()

        # update the ground reference points
        grp = GroundReferencePoints(wr_rankle, wl_rankle, H_w_lankle, H_w_rankle, H_w_lsole, H_w_rsole, d, com_state)
        grp_data = grp.update()
        
        # get reference ground reference points data
        if 2.9 < t < 3.5:
            x_ref[2] = tw.comState().pos()[2]
            grp_ref_data = grp.update()

        # apply force to the robot and balance it
        if 13.5 > t > 4.:
            force_amp = 50
            applyForce(force_amp, robot, t)
            if strategy == 'ankle':
                if not info_prompted:
                    info_prompted = True
                    rospy.logwarn('========= ANKLE BALANCING STRATEGY ACTIVATED =========')
                bc.ankleBalencingStategy(com_state.pos(), x_ref, 1.5, 1, grp_data[0], dt, grp_ref_data[0])
            elif strategy == 'hip':
                if not info_prompted:
                    info_prompted = True
                    rospy.logwarn('========= HIP BALANCING STRATEGY ACTIVATED =========')
                bc.hipBalencingStategy(grp_data[1], 0.3)   # can work with 90N force
            elif strategy == 'combined':
                if not info_prompted:
                    info_prompted = True
                    rospy.logwarn('========= COMBINED BALANCING STRATEGY ACTIVATED =========')
                bc.ankleBalencingStategy(com_state.pos(), x_ref, 0.6, 0.2, grp_data[0], dt, grp_ref_data[0])
                bc.hipBalencingStategy(grp_data[1], 0.3)
            elif strategy == 'none':
                if not info_prompted:
                    info_prompted = True    
                    rospy.logwarn('========= NO BALANCING STRATEGY ACTIVATED =========')

        # write data to csv file for 50Hz, record data for 17s
        if t - t_plot > 1./50. and t < 17.:
            t_plot = t
            '''
            [t, tw.comReference().pos()[2], tw.comState().pos()[2], robot.baseWorldPosition()[2]
            '''
            with open(csv_file, 'a') as file:
                writer = csv.writer(file)  # Note: writes lists, not dicts.
                writer.writerow([t, grp_data[0][0], grp_data[0][1], grp_data[0][2], 
                                    grp_data[1][0], grp_data[1][1], grp_data[1][2], 
                                    grp_data[2][0], grp_data[2][1], grp_data[2][2]])
        
        # TODO: update TSID controller
        '''
        update(self, q, v, t, do_sove=True)
        '''
        tau_sol, _ = tw.update(robot._q, robot._v, t)

        # command to the robot
        robot.setActuatedJointTorques(tau_sol)

        # publish to ros
        if t - t_publish > 1./30.:
            t_publish = t
            T_b_w = tw.baseState()
            # get current BASE Pose
            robot.publish(T_b_w)
    
if __name__ == '__main__': 
    rospy.init_node('tutorial_4_standing')
    strategy = rospy.get_param('~balance_strategy', 'hip')
    if strategy not in ['ankle', 'hip', 'combined', 'none']:
        raise ValueError('Unknown balance strategy, please choose between "ankle", "hip", "combined", "none"')
    main(strategy)
