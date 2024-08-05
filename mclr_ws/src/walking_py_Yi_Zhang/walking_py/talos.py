import numpy as np
import pinocchio as pin

# simulator
# >>>>TODO: Fix include
from robot import Robot

# whole-body controller
# >>>>TODO: Fix include
from tsid_wrapper import TSIDWrapper
# from t4_tsid_wrapper import TSIDWrapper

# robot configs
# >>>>TODO: Fix include
import talos_conf as conf

# >>>>TODO: Fix include
from footstep_planner import Side

# ROS visualizations
import rospy
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from visualization_msgs.msg import MarkerArray, Marker
import tf.transformations as tr


# PyBullet
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
        self.wrapper = TSIDWrapper(self.conf)

        # spawn robot in simulation

        self.robot = Robot(
            self.sim,
            self.conf.urdf,
            self.wrapper.model,
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
        # >>>>TODO: Turn on the force torque sensor in the robots feet
        pb.enableJointForceTorqueSensor(
            self.robot.id(), self.robot.jointNameIndexMap()["leg_right_6_joint"], True
        )
        pb.enableJointForceTorqueSensor(
            self.robot.id(), self.robot.jointNameIndexMap()["leg_left_6_joint"], True
        )
        ########################################################################
        # visualizations
        ########################################################################

        # >>>> TODO: joint state publisher
        self.joint_state_pub = rospy.Publisher(
            "/joint_state", JointState, queue_size=10
        )

        # >>>> TODO: floating base broadcaster
        self.broadcaster = tf.TransformBroadcaster()

        # >>>> TODO: zmp and dcm point publisher
        # >>>> Hint: Use visualization_msgs::MarkerArray, SPHERE to visualize
        self.marker_array_pub = rospy.Publisher("/zmp", MarkerArray, queue_size=10)

        # >>>> TODO: wrench publisher for left and right foot
        # >>>> Hint: use geometry_msgs::Wrench
        self.wrench_left_pub = rospy.Publisher(
            "/wrench_l", WrenchStamped, queue_size=10
        )
        self.wrench_right_pub = rospy.Publisher(
            "/wrench_r", WrenchStamped, queue_size=10
        )

    def update(self):
        """updates the robot"""
        t = self.sim.simTime()
        dt = self.sim.stepTime()

        # >>>> TODO: update the pybullet robot

        # update the estimators
        self._update_zmp_estimate()
        self._update_dcm_estimate()

        # update wbc and send back to pybullet
        self._solve(t, dt)

    def setSupportFoot(self, side):
        """sets the the support foot of the robot on given side"""

        # The support foot is in rigid contact with the ground and should
        # hold the weight of the robot
        self.support_foot = side

        # >>>> TODO: Activate the foot contact on the support foot
        # >>>> TODO: At the same time deactivate the motion task on the support foot
        if self.support_foot == Side.LEFT:
            # self.wrapper.contact_LF_active = True
            # self.wrapper.motion_LF_active = False
            self.wrapper.add_contact_LF()
            # self.wrapper.remove_motion_LF()
        else:
            # self.wrapper.contact_RF_active = True
            # self.wrapper.motion_RF_active = False
            self.wrapper.add_contact_RF()
            # self.wrapper.remove_motion_RF()

    def setSwingFoot(self, side):
        """sets the swing foot of the robot on given side"""

        # The swing foot is not in contact and can move
        self.swing_foot = side

        # >>>> TODO: Deactivate the foot contact on the swing foot
        # >>>> TODO: At the same time turn on the motion task on the swing foot
        if self.swing_foot == Side.LEFT:
            # self.wrapper.contact_LF_active = False
            # self.wrapper.motion_LF_active = True
            self.wrapper.remove_contact_LF()
            # self.wrapper.add_motion_LF()
      
        else:
            # self.wrapper.contact_RF_active = False
            # self.wrapper.motion_RF_active = True
            self.wrapper.remove_contact_RF()
            # self.wrapper.add_motion_RF()

    def updateSwingFootRef(self, T_swing_w, V_swing_w, A_swing_w):
        """updates the swing foot motion reference"""

        # >>>> TODO: set the pose, velocity and acceleration on the swing foots
        # motion task
        if self.swing_foot == Side.LEFT:
            self.wrapper.set_LF_pose_ref(pin.SE3(np.eye(3), T_swing_w), V_swing_w, A_swing_w)
            # self.wrapper.set_LF_pos_ref(T_swing_w, V_swing_w, A_swing_w)
        else:
            self.wrapper.set_RF_pose_ref(pin.SE3(np.eye(3), T_swing_w), V_swing_w, A_swing_w)
            # self.wrapper.set_RF_pos_ref(T_swing_w, V_swing_w, A_swing_w)

    def swingFootPose(self):
        """return the pose of the current swing foot"""
        # >>>>TODO: return correct foot pose
        pose = 0
        if self.swing_foot == Side.LEFT:
            pose, _ = self.wrapper.get_LF_3d_pos_vel_acc()
        else:
            pose, _ = self.wrapper.get_RF_3d_pos_vel_acc()
        return pose

    def supportFootPose(self):
        """return the pose of the current support foot"""
        # >>>>TODO: return correct foot pose
        pose = 0
        if self.support_foot == Side.LEFT:
            pose, _ = self.wrapper.get_LF_3d_pos_vel_acc()
        else:
            pose, _ = self.wrapper.get_RF_3d_pos_vel_acc()
        return pose

    def publish(self):
        current_ros_t = rospy.Time.now()

        # >>>> TODO: publish the jointstate
        joint_state_msg = JointState()

        joint_state_msg.header.stamp = current_ros_t

        joint_state_msg.name = \
            ['leg_left_1_joint', 'leg_left_2_joint', 'leg_left_3_joint', 'leg_left_4_joint', 'leg_left_5_joint', 'leg_left_6_joint', 
            'leg_right_1_joint', 'leg_right_2_joint', 'leg_right_3_joint', 'leg_right_4_joint', 'leg_right_5_joint', 'leg_right_6_joint',
            'torso_1_joint', 'torso_2_joint',  
            'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint', 'gripper_left_joint', 
            'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint', 'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint', 'gripper_right_joint', 
            'head_1_joint', 'head_2_joint']

        joint_state_msg.position = self.robot.q()
        joint_state_msg.velocity = self.robot.v()
        self.joint_state_pub.publish(joint_state_msg)

        # >>>> TODO: broadcast odometry
        T_b_w = self.wrapper.baseState()

        self.broadcaster.sendTransform(
            T_b_w[0].translation,
            tr.quaternion_from_matrix(T_b_w[0].homogeneous),
            current_ros_t,
            "base_link",
            "world",
        )

        # >>>> TODO: publish feet wrenches
        wren = pb.getJointState(
            self.robot.id(), self.robot.jointNameIndexMap()["leg_right_6_joint"]
        )[2]
        wnp = np.array([-wren[0], -wren[1], -wren[2], -wren[3], -wren[4], -wren[5]])
        wr_r_ankel = pin.Force(wnp)

        wren = pb.getJointState(
            self.robot.id(), self.robot.jointNameIndexMap()["leg_left_6_joint"]
        )[2]
        wnp = np.array([-wren[0], -wren[1], -wren[2], -wren[3], -wren[4], -wren[5]])
        wr_l_ankel = pin.Force(wnp)

        data = self.robot._model.createData()
        pin.framesForwardKinematics(self.robot._model, data, self.robot._q)

        H_w_lsole = data.oMf[self.robot._model.getFrameId("left_sole_link")]
        H_w_rsole = data.oMf[self.robot._model.getFrameId("right_sole_link")]
        H_w_lankle = data.oMf[self.robot._model.getFrameId("leg_left_6_joint")]
        H_w_rankle = data.oMf[self.robot._model.getFrameId("leg_right_6_joint")]

        wr_r_world = H_w_rankle * wr_r_ankel
        wr_l_world = H_w_lankle * wr_l_ankel

        wr_r_sole = np.inv(H_w_rsole) * wr_r_world
        wr_l_sole = np.inv(H_w_lsole) * wr_l_world

        wrench_msg_left = WrenchStamped()
        wrench_msg_left.header.stamp = current_ros_t
        wrench_msg_left.header.frame_id = "leg_left_6_joint"
        wrench_msg_left.wrench.force.x = wr_l_sole.linear[0]
        wrench_msg_left.wrench.force.y = wr_l_sole.linear[1]
        wrench_msg_left.wrench.force.z = wr_l_sole.linear[2]
        wrench_msg_left.wrench.torque.x = wr_l_sole.angular[0]
        wrench_msg_left.wrench.torque.y = wr_l_sole.angular[1]
        wrench_msg_left.wrench.torque.z = wr_l_sole.angular[2]

        wrench_msg_right = WrenchStamped()
        wrench_msg_right.header.stamp = current_ros_t
        wrench_msg_right.header.frame_id = "leg_right_6_joint"
        wrench_msg_right.wrench.force.x = wr_r_sole.linear[0]
        wrench_msg_right.wrench.force.y = wr_r_sole.linear[1]
        wrench_msg_right.wrench.force.z = wr_r_sole.linear[2]
        wrench_msg_right.wrench.torque.x = wr_r_sole.angular[0]
        wrench_msg_right.wrench.torque.y = wr_r_sole.angular[1]
        wrench_msg_right.wrench.torque.z = wr_r_sole.angular[2]

        self.wrench_left_pub.publish(wrench_msg_left)
        self.wrench_right_pub.publish(wrench_msg_right)

        # >>>> TODO: publish dcm and zmp marker
        zmp_marker = Marker()
        zmp_marker.header.frame_id = "world"
        zmp_marker.type = Marker.SPHERE
        zmp_marker.action = Marker.ADD
        zmp_marker.scale.x = 0.2
        zmp_marker.scale.y = 0.2
        zmp_marker.scale.z = 0.2
        zmp_marker.color.a = 1.0
        zmp_marker.color.r = 1.0
        zmp_marker.color.g = 1.0
        zmp_marker.color.b = 0.0
        zmp_marker.pose.orientation.w = 1.0
        zmp_marker.pose.orientation.w = 1.0
        zmp_marker.pose.orientation.x = 0.0
        zmp_marker.pose.orientation.y = 0.0
        zmp_marker.pose.orientation.z = 0.0
        zmp_marker.pose.position.x = self.zmp[0]
        zmp_marker.pose.position.y = self.zmp[1]
        zmp_marker.pose.position.z = self.zmp[2]

        dcm_marker = Marker()
        dcm_marker.header.frame_id = "world"
        dcm_marker.type = Marker.SPHERE
        dcm_marker.action = Marker.ADD
        dcm_marker.scale.x = 0.2
        dcm_marker.scale.y = 0.2
        dcm_marker.scale.z = 0.2
        dcm_marker.color.a = 1.0
        dcm_marker.color.r = 1.0
        dcm_marker.color.g = 1.0
        dcm_marker.color.b = 0.0
        dcm_marker.pose.orientation.w = 1.0
        dcm_marker.pose.orientation.x = 0.0
        dcm_marker.pose.orientation.y = 0.0
        dcm_marker.pose.orientation.z = 0.0
        dcm_marker.pose.position.x = self.zmp[0]
        dcm_marker.pose.position.y = self.zmp[1]
        dcm_marker.pose.position.z = self.zmp[2]

        markers = MarkerArray()
        markers.append(zmp_marker)
        markers.append(dcm_marker)

        self.marker_array_pub.publish(markers)

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

        tau_sol, _ = self.wrapper.update(q, v, t)

        # command to the robot
        self.robot.setActuatedJointTorques(tau_sol)

    def _update_zmp_estimate(self):
        """update the estimated zmp position"""
        # >>>> TODO: compute the zmp based on force torque sensor readings
        wren = pb.getJointState(
            self.robot.id(), self.robot.jointNameIndexMap()["leg_right_6_joint"]
        )[2]
        wnp = np.array([-wren[0], -wren[1], -wren[2], -wren[3], -wren[4], -wren[5]])
        wr_rankle = pin.Force(wnp)
        wren = pb.getJointState(
            self.robot.id(), self.robot.jointNameIndexMap()["leg_left_6_joint"]
        )[2]
        wnp = np.array([-wren[0], -wren[1], -wren[2], -wren[3], -wren[4], -wren[5]])
        wl_lankle = pin.Force(wnp)

        data = self.robot._model.createData()
        pin.framesForwardKinematics(self.robot._model, data, self.robot._q)

        H_w_lankle = data.oMf[self.robot._model.getFrameId("leg_left_6_joint")]
        H_w_rankle = data.oMf[self.robot._model.getFrameId("leg_right_6_joint")]

        p_xL = (
            -wl_lankle.angular[1] - wl_lankle.linear[0] * H_w_lankle.translation[2]
        ) / wl_lankle.linear[2]
        p_yL = (
            wl_lankle.angular[0] - wl_lankle.linear[1] * H_w_lankle.translation[2]
        ) / wl_lankle.linear[2]
        p_zL = 0

        p_L = np.array([p_xL, p_yL, p_zL])

        p_xR = (
            -wr_rankle.angular[1] - wr_rankle.linear[0] * H_w_rankle.translation[2]
        ) / wr_rankle.linear[2]
        p_yR = (
            wr_rankle.angular[0] - wr_rankle.linear[1] * H_w_rankle.translation[2]
        ) / wr_rankle.linear[2]
        p_zR = 0

        p_R = np.array([p_xR, p_yR, p_zR])

        # transform to world coordinate

        fz = wr_rankle.linear[2] + wl_lankle.linear[2]
        p_x = (p_R[0] * wr_rankle.linear[2] + p_L[0] * wl_lankle.linear[2]) / fz
        p_y = (p_R[1] * wr_rankle.linear[2] + p_L[1] * wl_lankle.linear[2]) / fz
        p_z = 0
        self.zmp = np.array([p_x, p_y, p_z])

    def _update_dcm_estimate(self):
        """update the estimated dcm position"""
        # >>>> TODO: compute the dcm based on current center of mass state
        com = self.wrapper.comState().value()
        com_vel = self.wrapper.comState().derivative()

        x_p = np.array(
            [com[0], com[1], 0]
        )  # project to zmp plane, which is ground plane here

        x_p_dot = np.array([com_vel[0], com_vel[1], 0])

        w = np.sqrt(abs(self.sim.gravity()[2]) / com[2])
        self.dcm = x_p + x_p_dot / w
