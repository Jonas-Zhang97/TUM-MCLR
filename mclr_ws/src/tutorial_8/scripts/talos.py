import numpy as np
import pinocchio as pin

# simulator
#>>>>TODO: Fix include
from robot import Robot

# whole-body controller
#>>>>TODO: Fix include
from tsid_wrapper import TSIDWrapper

# robot configs
#>>>>TODO: Fix include
import talos_conf as conf

#>>>>TODO: Fix include
from footstep_planner import Side

# ROS visualizations
import rospy
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from visualization_msgs.msg import MarkerArray

import pybullet as pb

class Talos:
    """Talos robot
    combines wbc with pybullet, functions to read and set
    sensor values.
    """
    def __init__(self, simulator):
        self.conf = conf
        self.sim = simulator
        
        #>>>>TODO: Like allways create the tsid wrapper
        self.stack = TSIDWrapper(conf)
        
        # spawn robot in simulation
        #>>>>TODO: Create the pybullet robot for the simulation
        self.robot = Robot(simulator,
                         conf.urdf,
                         self.stack.model,
                         basePosition=np.array([0, 0, 1.11]),
                         baseQuationerion=np.array([0, 0, 0, 1]),
                         q=conf.q_home,
                         useFixedBase=False,
                         verbose=True)
        
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
        #>>>>TODO: Turn on the force torque sensor in the robots feet
        pb.enableJointForceTorqueSensor(self.robot.id(), self.robot.jointNameIndexMap()['leg_right_6_joint'], True)
        pb.enableJointForceTorqueSensor(self.robot.id(), self.robot.jointNameIndexMap()['leg_left_6_joint'], True)
        
        ########################################################################
        # visualizations
        ########################################################################
        
        #>>>> TODO: joint state publisher
        self.joint_state_pub = rospy.Publisher('/joint_state', JointState, queue_size=10)
        
        #>>>> TODO: floating base broadcaster
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        #>>>> TODO: zmp and dcm point publisher 
        #>>>> Hint: Use visualization_msgs::MarkerArray, SPHERE to visualize 
        self.zmp_marker_pub = rospy.Publisher('/zmp_marker', MarkerArray, queue_size=10)
        self.dcm_marker_pub = rospy.Publisher('/dcm_marker', MarkerArray, queue_size=10)

        #>>>> TODO: wrench publisher for left and right foot
        #>>>> Hint: use geometry_msgs::Wrench
        self.lwrench_pub = rospy.Publisher('/lwrench', WrenchStamped, queue_size=10)
        self.rwrench_pub = rospy.Publisher('/rwrench', WrenchStamped, queue_size=10)
           
    def update(self):
        """updates the robot
        """
        t = self.sim.time()
        dt = self.sim.dt()

        #>>>> TODO: update the pybullet robot
        self.sim.step()
        
        # update the estimators
        self._update_zmp_estimate()
        self._update_dcm_estimate()
        
        # update wbc and send back to pybullet
        self._solve(t, dt)
        
    def setSupportFoot(self, side):
        """sets the the support foot of the robot on given side
        """
        
        # The support foot is in rigid contact with the ground and should 
        # hold the weight of the robot
        self.support_foot = side
        
        #>>>> TODO: Activate the foot contact on the support foot
        #>>>> TODO: At the same time deactivate the motion task on the support foot
        if side == Side.LEFT:
            self.stack.contact_LF_active = True
            self.stack.motion_LF_active = False
        else:
            self.stack.contact_RF_active = True
            self.stack.motion_RF_active = False

    
    def setSwingFoot(self, side):
        """sets the swing foot of the robot on given side
        """
        
        # The swing foot is not in contact and can move
        self.swing_foot = side
        
        #>>>> TODO: Deactivate the foot contact on the swing foot
        #>>>> TODO: At the same time turn on the motion task on the swing foot
        if side == Side.LEFT:
            self.stack.contact_LF_active = False
            self.stack.motion_LF_active = True
        else:  
            self.stack.contact_RF_active = False
            self.stack.motion_RF_active = True        
        
    def updateSwingFootRef(self, T_swing_w, V_swing_w, A_swing_w):
        """updates the swing foot motion reference
        """
        
        #>>>> TODO: set the pose, velocity and acceleration on the swing foots
        # motion task
        if self.swing_foot == Side.LEFT:
            self.stack.set_LF_pos_ref(T_swing_w, V_swing_w, A_swing_w)
        else:
            self.stack.set_RF_pos_ref(T_swing_w, V_swing_w, A_swing_w)

    def swingFootPose(self):
        """return the pose of the current swing foot
        """
        #>>>>TODO: return correct foot pose
        return None
    
    def supportFootPose(self):
        """return the pose of the current support foot
        """
        #>>>>TODO: return correct foot pose
        return None

    def publish(self):
        t = rospy.Time.now()
        
        #>>>> TODO: publish the jointstate
        
        #>>>> TODO: broadcast odometry
        
        #>>>> TODO: publish feet wrenches
        
        #>>>> TODO: publish dcm and zmp marker

    ############################################################################
    # private funcitons
    ############################################################################

    def _solve(self, t, dt):
        # get the current state
        q = self.robot.q()
        v = self.robot.v()
        
        # solve the whole body qp
        #>>>> TODO: sovle the wbc and command the torque to pybullet robot
    
    def _update_zmp_estimate(self):
        """update the estimated zmp position
        """
        #>>>> TODO: compute the zmp based on force torque sensor readings
        self.zmp = None
        
    def _update_dcm_estimate(self):
        """update the estimated dcm position
        """
        #>>>> TODO: compute the com based on current center of mass state
        self.dcm = None

