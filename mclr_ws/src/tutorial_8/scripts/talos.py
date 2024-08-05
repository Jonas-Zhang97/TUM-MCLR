import numpy as np
import pinocchio as pin

# simulator
#>>>>TODO Fix include
from robot import Robot

# whole-body controller
#>>>>TODO Fix include
from tsid_wrapper import TSIDWrapper

# robot configs
#>>>>TODO Fix include
import talos_conf as conf

#>>>>TODO Fix include
from footstep_planner import Side

# ROS visualizations
import rospy
import tf
from tf.transformations import quaternion_from_matrix as mat2quat
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from visualization_msgs.msg import MarkerArray, Marker

import pybullet as pb

class Talos:
    """Talos robot
    combines wbc with pybullet, functions to read and set
    sensor values.
    """
    def __init__(self, simulator):
        self.conf = conf
        self.sim = simulator

        # >>>>TODO: Like allways create the tsid wrapper
        self.stack = TSIDWrapper(self.conf)

        # spawn robot in simulation

        self.robot = Robot(
            self.sim,
            self.conf.urdf,
            self.stack.model,
            basePosition=np.array([0, 0, 1.1]),
            baseQuationerion=np.array([0, 0, 0, 1]),
            q=self.conf.q_home,
            useFixedBase=False,
            verbose=True,
        )
        
        ########################################################################
        # state
        ########################################################################
        self.support_foot = Side.RIGHT
        self.swing_foot = Side.LEFT
        
        ########################################################################
        # estimators
        ########################################################################
        self.zmp = None
        self.dcm = None
        
        ########################################################################
        # sensors
        ########################################################################
        # ft sensors
        #>>>>TODO Turn on the force torque sensor in the robots feet
        pb.enableJointForceTorqueSensor(self.robot.id(), self.robot.jointNameIndexMap()['leg_right_6_joint'], True)
        pb.enableJointForceTorqueSensor(self.robot.id(), self.robot.jointNameIndexMap()['leg_left_6_joint'], True)
        
        ########################################################################
        # visualizations
        ########################################################################
        
        #>>>> TODO joint state publisher
        self.joint_state_pub = rospy.Publisher('/joint_state', JointState, queue_size=10)
        
        #>>>> TODO floating base broadcaster
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        #>>>> TODO zmp and dcm point publisher 
        #>>>> Hint: Use visualization_msgs::MarkerArray, SPHERE to visualize 
        self.zmp_marker_pub = rospy.Publisher('/zmp_marker', MarkerArray, queue_size=10)
        self.dcm_marker_pub = rospy.Publisher('/dcm_marker', MarkerArray, queue_size=10)

        #>>>> TODO wrench publisher for left and right foot
        #>>>> Hint: use geometry_msgs::Wrench
        self.rwrench_pub = rospy.Publisher('/rwrench', WrenchStamped, queue_size=10)
        self.lwrench_pub = rospy.Publisher('/lwrench', WrenchStamped, queue_size=10)
           
    def update(self):
        """updates the robot
        """
        t = self.sim.simTime()
        dt = self.sim.stepTime()

        #>>>> TODO update the pybullet robot
        
        # update the estimators
        self._update_zmp_estimate()
        self._update_dcm_estimate()
        
        # update wbc and send back to pybullet
        self._solve(t, dt)
        
    def setSupportFoot(self, side):
        """sets the the support foot of the robot on given side
        args:
            side (Side): the side of the support foot
        """
        
        # The support foot is in rigid contact with the ground and should 
        # hold the weight of the robot
        self.support_foot = side
        
        #>>>> TODO Activate the foot contact on the support foot
        #>>>> TODO At the same time deactivate the motion task on the support foot
        if side == Side.LEFT:
            self.stack.contact_LF_active = True
            self.stack.add_contact_LF()
        else:
            self.stack.contact_RF_active = True
            self.stack.add_contact_RF()

    
    def setSwingFoot(self, side):
        """sets the swing foot of the robot on given side
        args:
            side (Side): the side of the swing foot
        """
        
        # The swing foot is not in contact and can move
        self.swing_foot = side
        
        #>>>> TODO Deactivate the foot contact on the swing foot
        #>>>> TODO At the same time turn on the motion task on the swing foot
        if side == Side.LEFT:
            self.stack.motion_LF_active = True
            self.stack.remove_contact_LF()
        else:  
            self.stack.motion_RF_active = True
            self.stack.remove_contact_RF()   
        
    def updateSwingFootRef(self, T_swing_w, V_swing_w, A_swing_w):
        """updates the swing foot motion reference
        """
        
        #>>>> TODO set the pose, velocity and acceleration on the swing foots
        # motion task
        if self.swing_foot == Side.LEFT:
            self.stack.set_LF_pos_ref(T_swing_w, V_swing_w, A_swing_w)
        else:
            self.stack.set_RF_pos_ref(T_swing_w, V_swing_w, A_swing_w)

    def swingFootPose(self):
        """return the pose of the current swing foot
        """
        #>>>>TODO return correct foot pose
        if self.swing_foot == Side.LEFT:
            return self.stack.get_RF_3d_pos_vel_acc()[0]
        else:
            return self.stack.get_LF_3d_pos_vel_acc()[0]
    
    def supportFootPose(self):
        """return the pose of the current support foot
        """
        #>>>>TODO return correct foot pose
        if self.support_foot == Side.LEFT:
            return self.stack.get_LH_3d_pos_vel_acc()[0]
        else:
            return self.stack.get_RH_3d_pos_vel_acc()[0]
        
    def publishJointStateMsg(self, t):
        """
        returns the joint state message
        """
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = t
        joint_state_msg.name = self.robot.actuatedJointNames()
        joint_state_msg.position = self.robot.actuatedJointPosition().tolist()
        joint_state_msg.velocity = self.robot.actuatedJointVelocity().tolist()
        joint_state_msg.effort = [0.0] * len(joint_state_msg.name)  # Assuming zero effort for simplicity
        self.joint_state_pub.publish(joint_state_msg)
        return None
    
    def publishWrenchMsg(self, t):
        """
        returns the wrench message
        """
        # Transform the wrench to the sole frame
        H_lsole_lankle = self.H_w_lsole.inverse().act(self.H_w_lankle)
        H_rsole_rankle = self.H_w_rsole.inverse().act(self.H_w_rankle)
        wr_rsole = H_rsole_rankle.act(self.wr_rankle)
        wr_lsole = H_lsole_lankle.act(self.wr_rankle)

        rwrench_msg = WrenchStamped()
        rwrench_msg.header.stamp = t
        rwrench_msg.header.frame_id = "leg_right_6_joint"
        rwrench_msg.wrench.force.x = wr_rsole.linear[0]
        rwrench_msg.wrench.force.y = wr_rsole.linear[1]
        rwrench_msg.wrench.force.z = wr_rsole.linear[2]
        rwrench_msg.wrench.torque.x = wr_rsole.angular[0]
        rwrench_msg.wrench.torque.y = wr_rsole.angular[1]
        rwrench_msg.wrench.torque.z = wr_rsole.angular[2]

        lwrench_msg = WrenchStamped()
        lwrench_msg.header.stamp = t
        lwrench_msg.header.frame_id = "leg_left_6_joint"
        lwrench_msg.wrench.force.x = wr_lsole.linear[0]
        lwrench_msg.wrench.force.y = wr_lsole.linear[1]
        lwrench_msg.wrench.force.z = wr_lsole.linear[2]
        lwrench_msg.wrench.torque.x = wr_lsole.angular[0]
        lwrench_msg.wrench.torque.y = wr_lsole.angular[1]
        lwrench_msg.wrench.torque.z = wr_lsole.angular[2]

        self.rwrench_pub.publish(rwrench_msg)
        self.lwrench_pub.publish(lwrench_msg)
        return None
    
    def publishZMPMarker(self, marker_pos, t):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = t
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.position.x = marker_pos[0]
        marker.pose.position.y = marker_pos[1]
        marker.pose.position.z = marker_pos[2]
        array = MarkerArray()
        array.markers.append(marker)
        self.zmp_marker_pub.publish(array)
        pass
    
    def publishDCMMarker(self, marker_pos, t):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = t
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.position.x = marker_pos[0]
        marker.pose.position.y = marker_pos[1]
        marker.pose.position.z = marker_pos[2]
        array = MarkerArray()
        array.markers.append(marker)
        self.dcm_marker_pub.publish(array)
        pass
    
    def updateSensor(self):
        ######################################################################## NOTE: This part is copied from the previous tutorial 6, not tested yet
        wren = pb.getJointState(self.robot.id(), self.robot.jointNameIndexMap()['leg_right_6_joint'])[2]
        wnp = np.array([-wren[0], -wren[1], -wren[2], -wren[3], -wren[4], -wren[5]])
        self.wr_rankle = pin.Force(wnp)
        wlen = pb.getJointState(self.robot.id(), self.robot.jointNameIndexMap()['leg_left_6_joint'])[2]
        wnp = np.array([-wlen[0], -wlen[1], -wlen[2], -wlen[3], -wlen[4], -wlen[5]])
        self.wr_lankle = pin.Force(wnp)

        data = self.robot._model.createData()
        pin.framesForwardKinematics(self.robot._model, data, self.robot._q)
        self.H_w_lsole = data.oMf[self.robot._model.getFrameId("left_sole_link")]
        self.H_w_rsole = data.oMf[self.robot._model.getFrameId("right_sole_link")]
        self.H_w_lankle = data.oMf[self.robot._model.getFrameId("leg_left_6_joint")]
        self.H_w_rankle = data.oMf[self.robot._model.getFrameId("leg_right_6_joint")]
        ########################################################################

        # get CoM state
        self.com_state = self.stack.comState()
        self.d = self.H_w_lankle.translation[2]

    def publish(self):
        t = rospy.Time.now()
        self.updateSensor()
        self._update_dcm_estimate()
        self._update_zmp_estimate()
        
        #>>>> TODO publish the jointstate
        self.publishJointStateMsg(t)
        #>>>> TODO broadcast odometry
        T_b_w = self.stack.baseState()
        self.odom_broadcaster.sendTransform(T_b_w[0].translation, mat2quat(T_b_w[0].rotation), t, "odom", "world")
        
        #>>>> TODO publish feet wrenches
        #>>>> TODO publish dcm and zmp marker
        self.publishDCMMarker(self.dcm, t)
        self.publishZMPMarker(self.zmp, t)

    ############################################################################
    # private funcitons
    ############################################################################

    def _solve(self, t, dt):
        # get the current state
        q = self.robot._q
        v = self.robot._v

        # solve the whole body qp
        # >>>> TODO: sovle the wbc and command the torque to pybullet robot
        self.robot.update()

        tau_sol, _ = self.stack.update(q, v, t)

        # command to the robot
        self.robot.setActuatedJointTorques(tau_sol)
    
    def _update_zmp_estimate(self):
        """update the estimated zmp position
        """
        #>>>> TODO compute the zmp based on force torque sensor readings
        ######################################################################### NOTE:  This part is copied from the previous tutorial 6, not tested yet
        self.updateSensor()
        p_lx = (self.wr_lankle.angular[1] - self.wr_lankle.linear[0] * self.d) / self.wr_lankle.linear[2]
        p_ly = (self.wr_lankle.angular[0] - self.wr_lankle.linear[1] * self.d) / self.wr_lankle.linear[2]
        p_l = np.array([p_lx, p_ly, 0])
        p_l_w = self.H_w_lsole.inverse() * p_l

        p_rx = (-self.wr_rankle.angular[1] - self.wr_rankle.linear[0] * self.d) / self.wr_rankle.linear[2]
        p_ry = (self.wr_rankle.angular[0] - self.wr_rankle.linear[1] * self.d) / self.wr_rankle.linear[2]
        p_r = np.array([p_rx, p_ry, 0])
        p_r_w = self.H_w_rsole.inverse() * p_r
        
        zmp = np.zeros(3)
        zmp[0] = (p_r_w[0] * self.wr_rankle.linear[2] + p_l_w[0] * self.wr_lankle.linear[2]) / (self.wr_rankle.linear[2] + self.wr_lankle.linear[2])
        zmp[1] = (p_r_w[1] * self.wr_rankle.linear[2] + p_l_w[1] * self.wr_lankle.linear[2]) / (self.wr_rankle.linear[2] + self.wr_lankle.linear[2])
        #########################################################################

        self.zmp = zmp
        
    def _update_dcm_estimate(self):
        """update the estimated dcm position
        """
        self.updateSensor()
        #>>>> TODO compute the com based on current center of mass state
        x_p = [self.com_state.pos()[0], self.com_state.pos()[1], 0]
        x_p_dot = [self.com_state.vel()[0], self.com_state.vel()[1], 0]
        
        dcm = x_p + x_p_dot / np.sqrt(9.81 / self.com_state.pos()[2])
        self.dcm = dcm

