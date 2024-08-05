"""
talos walking simulation
"""

import rospy
import numpy as np
import pinocchio as pin

# simulator
# >>>>TODO: import simulator
from pybullet_wrapper import PybulletWrapper

# robot configs
# >>>>TODO: Import talos robot
from talos import Talos
import talos_conf as conf

# modules
# >>>>TODO: Import all previously coded modules
from footstep_planner import Side, FootStepPlanner
from foot_trajectory import SwingFootTrajectory
from lip_mpc import LIPMPC, generate_zmp_reference, LIPInterpolator

import pybullet as pb

################################################################################
# main
################################################################################

def main(): 

    ############################################################################
    # setup
    ############################################################################

    # setup the simulator
    # >>>>TODO: simulation
    simulator = PybulletWrapper()

    # setup the robot
    # >>>> TODO: robot
    robot = Talos(simulator)

    # inital footsteps
    # T_swing_w = pin.SE3(np.eye(3), robot.swingFootPose())  # >>>>TODO: set intial swing foot pose to left foot
    # T_support_w = pin.SE3(np.eye(3), robot.supportFootPose())  # >>>>TODO: set intial swing foot pose to right foot

    # debug
    robot.swing_foot = Side.RIGHT
    robot.support_foot = Side.LEFT
    T_swing_w = pin.SE3(np.eye(3), robot.swingFootPose())  
    T_support_w = pin.SE3(np.eye(3), robot.supportFootPose())

    # setup the plan
    no_steps = 20
    # next_swing_side = Side.LEFT
    next_swing_side = Side.RIGHT  # debug
    planner = FootStepPlanner(conf)  #>>>>TODO: Create the planner
    plan = planner.planLine(
        T_swing_w, next_swing_side, no_steps, T_support_w
    )  # >>>>TODO: Create the plan
    # >>>>TODO: Append the two last steps once more to the plan so our mpc horizon will never run out
    last_second_step = plan[-2]
    last_step = plan[-1]
    plan.append(last_second_step)
    plan.append(last_step)

    planner.plot(simulator)

    # generate reference
    ZMP_ref = generate_zmp_reference(plan[1:],conf.no_mpc_samples_per_step) #>>>>TODO: Generate the mpc reference
    # >>>>TODO: plot the plan (make sure this workes first)

    # setup the lip models
    mpc = LIPMPC(conf) #>>>>TODO: setup mpc

    # Assume the com is over the first support foot
    p0_x = T_support_w.translation[0]
    v0_x = 0
    p0_y = T_support_w.translation[1]
    v0_y = 0
    x_0 = np.array([p0_x, v0_x, p0_y, v0_y])  # >>>>TODO: Build the intial mpc state vector [p_x,v_x,p_y,v_y]
    interpolator = LIPInterpolator(x_0.copy(), conf)  # >>>>TODO: Create the interpolator and set the inital state

    # set the com task over the support foot
    # >>>>TODO: Set the COM reference to be over supporting foot
    u_k = np.array([0, 0.096])

    # simulator.addSphereMarker(current_zmp, radius=0.02, color=[0, 1, 1, 1])

    ############################################################################
    # logging
    ############################################################################

    pre_dur = 3.0   # Time to wait befor walking should start
    N_pre = int(pre_dur / simulator.stepTime())#>>>>TODO: number of sim steps before walking starts 
    N_sim = conf.no_sim_per_step * no_steps #>>>>TODO: total number of sim steps during walking
    N_mpc = conf.no_mpc_samples_per_step* no_steps #>>>>TODO: total number of mpc steps during walking

    # >>>>TODO: Create vectors to log all the data of the simulation
    # - COM_POS, COM_VEL, COM_ACC (from the planned reference, pinocchio and pybullet)
    # - Angular momentum (from pinocchio)
    # - Left and right foot POS, VEL, ACC (form planned reference, pinocchio)
    # - ZMP (from planned reference, from estimator )
    # - DCM (from estimtor)
    # - Normal forces in right and left foot (from pybullet ft sensors, from pinocchio)
    TIME = np.nan*np.empty(N_sim)
    COM_POS = np.nan * np.ones([N_sim, 3])
    COM_VEL = np.nan * np.ones([N_sim, 3])
    COM_ACC = np.nan * np.ones([N_sim, 3])
    Angular_momentum = np.nan * np.ones([N_sim, 3])
    LEFT_FOOT_POS = np.nan * np.ones([N_sim, 3])
    LEFT_FOOT_VEL = np.nan * np.ones([N_sim, 3])
    LEFT_FOOT_ACC = np.nan * np.ones([N_sim, 3])
    RIGHT_FOOT_POS = np.nan * np.ones([N_sim, 3])
    RIGHT_FOOT_VEL = np.nan * np.ones([N_sim, 3])
    RIGHT_FOOT_ACC = np.nan * np.ones([N_sim, 3])
    ZMP_REF_VEC = np.nan * np.ones([N_sim, 3])
    ZMP_REAL_VEC = np.nan * np.ones([N_sim, 3])
    DCM_REF_VEC = np.nan * np.ones([N_sim, 3])
    FORCE_LEFT_FOOT = np.nan * np.ones([N_sim, 3])
    FORCE_LEFT_FOOT_DES = np.nan * np.ones([N_sim, 3])
    FORCE_RIGHT_FOOT = np.nan * np.ones([N_sim, 3])
    FORCE_RIGHT_FOOT_DES = np.nan * np.ones([N_sim, 3])

    ############################################################################
    # logging
    ############################################################################

    # debug
    runonce = True
    id = -1
    swing_start = False

    k = 0                                               # current MPC index                          
    plan_idx = 1                                        # current index of the step within foot step plan
    t_step_elapsed = 0.0                                # elapsed time within current step (use to evaluate spline)
    t_publish = 0.0                                     # last publish time (last time we published something)

    flag = 1

    temp_pos = None

    for i in range(-N_pre, N_sim):
        t = simulator.simTime() #>>>>TODO: simulator time
        dt = simulator.stepTime() #>>>>TODO: simulator dt

        ########################################################################
        # Debug
        ########################################################################

        if i > -2000 and i < 0 and runonce:
            runonce = False
            c, _, _ = interpolator.comState()

            robot.wrapper.setComRefState(c)

        if i % 100 == 0:
            com = robot.wrapper.comState().pos()
            c, _, _ = interpolator.comState()
            simulator.addSphereMarker(com + np.array([0, 0, -conf.h]), radius=0.02, color=[0, 1, 1, 1])
            simulator.addSphereMarker(c + np.array([0, 0, -conf.h]), radius=0.02, color=[1, 0, 1, 1])
            # simulator.addSphereMarker(temp_pos, radius=0.02, color=[0, 0.2, 0.8, 1])

        # #######################################################################
        # update the mpc very no_sim_per_mpc steps
        # #######################################################################

        if i >= 0 and i % conf.no_sim_per_mpc == 0: # >>>>TODO: when to update mpc
            # >>>>TODO: Implement MPC update
            x_k = interpolator.x
            idx_terminal_k = N_mpc - k
            ZMP_ref_k = ZMP_ref[k : k + conf.no_mpc_samples_per_horizon]
            u_k = mpc.buildSolveOCP(x_k, ZMP_ref_k, idx_terminal_k)
            k += 1

            simulator.addSphereMarker(
                np.array([x_k[0], x_k[2], 0]), radius=0.02, color=[0, 1, 0, 1]
            )

            # simulator.addSphereMarker(
            #     robot.swingFootPose(), radius=0.02, color=[0.2, 1, 1, 1]
            # )
            # simulator.addSphereMarker(temp_pos, radius=0.02, color=[1, 0, 1, 1])

        ########################################################################
        # update the foot spline
        ########################################################################

        if i >= 0 and i % conf.no_sim_per_step == 0:  # >>>>TODO: when to update spline
            # >>>>TODO: Start next step
            t_step_elapsed = 0.0
            plan_idx += 1

            if next_swing_side == Side.LEFT:
                robot.setSwingFoot(Side.LEFT)
                robot.setSupportFoot(Side.RIGHT)
                next_swing_side = Side.RIGHT
                flag = 0
            else:
                robot.setSwingFoot(Side.RIGHT)
                robot.setSupportFoot(Side.LEFT)
                next_swing_side = Side.LEFT

            swing_foot_pos = robot.swingFootPose()
            next_swing_step = plan[plan_idx].pose.translation

            # if plan_idx > 2:
            #     if next_swing_side == Side.LEFT:
            #         next_swing_step += np.array([0.08, -0.042, 0])
            #     else:
            #         next_swing_step += np.array([0.08, 0.04, 0])

            swing_foot_traj = SwingFootTrajectory(
                swing_foot_pos,
                next_swing_step,
                conf.step_dur
            )

            # simulator.addSphereMarker(next_swing_step, radius=0.02, color=[0, 1, 0, 1])

        ########################################################################
        # in every iteration when walking
        ########################################################################

        if i >= 0 :
            pos, vel, acc = swing_foot_traj.evaluate(t_step_elapsed)
            temp_pos = pos

            robot.updateSwingFootRef(pos, vel, acc)


            interpolator.integrate(u_k)
            c, c_dot, c_ddot = interpolator.comState()

            robot.wrapper.setComRefState(c, c_dot, c_ddot)

            if flag == 1:
                pass
            else:
                if i % 10 == 0:
                    pass
            t_step_elapsed += dt

        ########################################################################
        # update the simulation
        ########################################################################

        # >>>>TODO: update the simulator and the robot
        simulator.debug()
        simulator.step()
        robot.update()

        # publish to ros
        if t - t_publish > 1./30.:
            t_publish = t
            # >>>>TODO: publish
            # robot.publish()

        # store for visualizations
        if i >= 0:
            TIME[i] = t
            # >>>>TODO: log information

    ########################################################################
    # enough with the simulation, lets plot
    ########################################################################

    import matplotlib.pyplot as plt
    plt.style.use('seaborn-dark')

    # >>>>TODO: plot everything

if __name__ == '__main__': 
    # rospy.init_node('talos_walking')
    main()
