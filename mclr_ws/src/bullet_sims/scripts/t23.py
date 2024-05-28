#!/bin/python3.8
import sys
import pybullet as pb
import numpy as np
from simulator.pybullet_wrapper import PybulletWrapper
from simulator.robot import Robot
import pinocchio as pin
import rospy
from sensor_msgs.msg import JointState
# For REEM-C robot
#urdf = "src/reemc/reemc_description/robots/reemc.urdf"
#path_meshes = "src/reemc/reemc_description/meshes/../.."

def initNode():
    rospy.init_node('t23')
    rospy.loginfo('t23 node started')

def publishJointState(robot, tau, joint_state_pub):
    msg = JointState()

    # Set the header for the JointState message
    msg.header.stamp = rospy.Time.now()

    # Set the joint names for the JointState message
    msg.name = robot.actuatedJointNames()

    # Set the joint positions, velocities, and efforts for the JointState message
    msg.position = robot._q
    msg.velocity = robot._v
    msg.effort = tau

    joint_state_pub.publish(msg)

def main(args):

    # For Talos robot
    urdf = "src/talos/talos_description/robots/talos_reduced.urdf"
    path_meshes = "src/talos/talos_description/meshes/../.."

    '''
    actuated joint names:   32 ['torso_1_joint', 'torso_2_joint', 
                                'head_1_joint', 'head_2_joint', 
                                'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint', 
                                'gripper_left_joint', 
                                'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint', 'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint', 
                                'gripper_right_joint', 
                                'leg_left_1_joint', 'leg_left_2_joint', 'leg_left_3_joint', 'leg_left_4_joint', 'leg_left_5_joint', 'leg_left_6_joint', 
                                'leg_right_1_joint', 'leg_right_2_joint', 'leg_right_3_joint', 'leg_right_4_joint', 'leg_right_5_joint', 'leg_right_6_joint']
    actuated joint indexes: 32 [0, 1, 3, 4, 11, 12, 13, 14, 15, 16, 17, 21, 28, 29, 30, 31, 32, 33, 34, 38, 45, 46, 47, 48, 49, 50, 52, 53, 54, 55, 56, 57]

    Talos
    0, 1, 2, 3, 4, 5, 			    # left leg
    6, 7, 8, 9, 10, 11, 			# right leg
    12, 13,                         # torso
    14, 15, 16, 17, 18, 19, 20, 21  # left arm
    22, 23, 24, 25, 26, 27, 28, 29  # right arm
    30, 31                          # head

    REEMC
    0, 1, 2, 3, 4, 5, 			    # left leg
    6, 7, 8, 9, 10, 11, 			# right leg
    12, 13,                         # torso
    14, 15, 16, 17, 18, 19, 20,     # left arm
    21, 22, 23, 24, 25, 26, 27,     # right arm
    28, 29                          # head
    '''

    # Initial condition for the simulator an model
    z_init = 1.15
    q_actuated_home = np.zeros(32)
    q_actuated_home[:6] = np.array([0, 0, 0, 0, 0, 0])
    q_actuated_home[6:12] = np.array([0, 0, 0, 0, 0, 0])
    q_actuated_home[14:22] = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    q_actuated_home[22:30] = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    q_actuated_target = np.zeros(32)
    q_actuated_target[:6] = np.array([0, 0, -0.44, 0.9, -0.45, 0])
    q_actuated_target[6:12] = np.array([0, 0, -0.44, 0.9, -0.45, 0])
    q_actuated_target[14:22] = np.array([0, -0.24, 0, -1, 0, 0, 0, 0])
    q_actuated_target[22:30] = np.array([0, -0.24, 0, -1, 0, 0, 0, 0])

    # Initialization position including floating base
    q_home = np.hstack([np.array([0, 0, z_init, 0, 0, 0, 1]), q_actuated_home])
    q_target = np.hstack([np.array([0, 0, z_init, 0, 0, 0, 1]), q_actuated_target])

    # setup the task stack
    modelWrap = pin.RobotWrapper.BuildFromURDF(urdf,                        # Model description
                                               path_meshes,                 # Model geometry descriptors 
                                               pin.JointModelFreeFlyer(),   # Floating base model. Use "None" if fixed
                                               True,                        # Printout model details
                                               None)                        # Load meshes different from the descripor
    # Get model from wrapper
    model = modelWrap.model

    # setup the simulator
    simulator = PybulletWrapper(sim_rate=1000)

    #Create Pybullet-Pinocchio map
    robot = Robot(simulator,            # The Pybullet wrapper
                  urdf,                 # Robot descriptor
                  model,                # Pinocchio model
                  [0, 0, z_init],       # Floating base initial position
                  [0,0,0,1],            # Floating base initial orientation [x,y,z,w]
                  q=q_home,             # Initial state
                  useFixedBase=False,   # Fixed base or not
                  verbose=True)         # Printout details

    data = robot._model.createData()

    inertia_mat = pin.crba(robot._model, data, robot._q)
    print("inertia_mat", inertia_mat)

    #Needed for compatibility
    simulator.addLinkDebugFrame(-1,-1)

    # Setup pybullet camera
    pb.resetDebugVisualizerCamera(
        cameraDistance=1.2,
        cameraYaw=90,
        cameraPitch=-20,
        cameraTargetPosition=[0.0, 0.0, 0.8])

    # Joint command vector
    tau = q_actuated_home*0

    # Set PD gains
    K_p = np.eye(39)
    K_d = np.eye(39)

    # left leg
    K_p[7: 13, 7: 13] = np.eye(6) * 1500
    # right leg
    K_p[13: 19, 13: 19] = np.eye(6) * 1500

    # left arm
    K_p[21: 28, 21: 28] = np.eye(7) * 400
    # right arm
    K_p[28: 35, 28: 35] = np.eye(7) * 400

    # torso
    K_p[19: 21, 19: 21] = np.eye(2) * 500

    # head
    K_p[37: , 37: ] = np.zeros(2) * 300
    np.set_printoptions(threshold=sys.maxsize)
    print("K_p", K_p)

    initNode()
    joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    d = 0
    done = False
    loop_rate = rospy.Rate(30)
    while not done:
        # update the simulator and the robot
        simulator.step()
        simulator.debug()
        robot.update()

        if d <= 1:
            q_des = pin.interpolate(model, q_home, q_target, d)
        else:
            q_des = q_target

        d += 0.001

        tau = np.dot(K_p[7: , 7: ], (q_des[7: ] - robot._q[7: ])) - np.dot(K_d[7: , 7: ], robot._v[6: ])

        # command to the robot
        robot.setActuatedJointTorques(tau)
        publishJointState(robot, tau, joint_state_pub)
        # loop_rate.sleep()

if __name__ == "__main__":
    main(sys.argv)