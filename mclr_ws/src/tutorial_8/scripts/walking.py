"""
talos walking simulation
"""

import rospy
import numpy as np
import pinocchio as pin

# simulator
#>>>>TODO: import simulator
from pybullet_wrapper import PybulletWrapper

# robot configs
#>>>>TODO: Import talos robot
import talos
import talos_conf as conf

# modules
#>>>>TODO: Import all previously coded modules
from foot_trajectory import SwingFootTrajectory
from footstep_planner import FootStepPlanner, Side, FootStep
from lip_mpc import LIPMPC, LIPInterpolator, generate_zmp_reference
        
################################################################################
# main
################################################################################  
    
def main(): 
    
    ############################################################################
    # setup
    ############################################################################
    
    # setup the simulator
    #>>>>TODO simulation
    simulator = PybulletWrapper()
    
    # setup the robot
    #>>>> TODO robot
    robot = talos.Talos(simulator)
    # robot.stack = TSIDWrapper(conf)

    T_swing_w = robot.stack.get_placement_LF() #>>>>TODO set intial swing foot pose to left foot
    T_support_w = robot.stack.get_placement_RF() #>>>>TODO set intial swing foot pose to right foot
    
    # setup the plan
    no_steps = 20
    planner = FootStepPlanner(conf) #>>>>TODO Create the planner
    curr_support = Side.RIGHT
    curr_swing = Side.LEFT
    plan = planner.planLine(T_support_w, curr_support, no_steps, simulator, is_plot=True) #>>>>TODO Create the plan
    #>>>>TODO Append the two last steps once more to the plan so our mpc horizon will never run out 
    plan.append(plan[-2])
    plan.append(plan[-1])

    # generate reference
    #>>>>TODO: Generate the mpc reference
    #>>>>TODO: plot the plan (make sure this workes first)
    
    zmp_ref = generate_zmp_reference(plan, conf.no_mpc_samples_per_step) #>>>>TODO Generate the zmp reference
    # for i, zmp in enumerate(zmp_ref):
    #     i
    #     t = zmp.translation
    # setup the lip models
    mpc = LIPMPC(conf) #>>>>TODO setup mpc
    
    # Assume the com is over the first support foot
    x0 = np.array([plan[0].translation[0], 0, plan[0].translation[1], 0]) #>>>>TODO: Build the intial mpc state vector
    interpolator = LIPInterpolator(x0.copy(), conf) #>>>>TODO: Create the interpolator and set the inital state
    # set the com task over the support foot
    #>>>>TODO: Set the COM reference to be over supporting foot 
    # u_k = np.array([0, 0.096])
    
    
    ############################################################################
    # logging
    ############################################################################

    pre_dur = 3.0   # Time to wait befor walking should start
    N_pre = int(pre_dur * simulator.stepFreq()) #>>>>TODO: number of sim steps before walking starts 
    N_sim = conf.no_sim_per_step * no_steps#>>>>TODO: total number of sim steps during walking
    N_mpc = conf.no_mpc_samples_per_step* no_steps #>>>>TODO: total number of mpc steps during walking
    
    #>>>>TODO: Create vectors to log all the data of the simulation
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
    
    k = 0                                               # current MPC index                          
    plan_idx = 0                                        # current index of the step within foot step plan
    t_step_elapsed = 0.0                                # elapsed time within current step (use to evaluate spline)
    t_publish = 0.0                                     # last publish time (last time we published something)

    is_init_logged = False  
    initialized = False
    
    for i in range(-N_pre, N_sim):
        # rint("iteration: ", i)
        t = simulator.simTime() #>>>>TODO: simulator time
        dt = simulator.stepTime() #>>>>TODO: simulator 0.001

        
        # Move CoM to support foot
        if i > -2000 and i <= 0 and not is_init_logged:
            if not is_init_logged:
                rospy.loginfo("Waiting for the robot to stabilize")
                is_init_logged = True
            # get current COM height
            c, _, _ = interpolator.comState()
            robot.stack.setComRefState(c)

        if i % 10 == 0:
            com = robot.stack.comState().pos()
            c, _, _ = interpolator.comState()
            # simulator.addSphereMarker(com + np.array([0, 0, -conf.h]), radius=0.02, color=[0, 1, 1, 1])
            simulator.addSphereMarker(c + np.array([0, 0, -conf.h]), radius=0.02, color=[1, 0, 1, 1])

        
        ########################################################################
        # update the mpc very no_sim_per_mpc steps
        ########################################################################
        
        if i >= 0 and i % conf.no_sim_per_mpc == 0: #>>>>TODO: when to update mpc
            #>>>>TODO: Implement MPC update
            # com_ref = robot.stack.comState()
            x_k = interpolator.x
            zmp_ref_k = zmp_ref[k: k + conf.no_mpc_samples_per_horizon]
            terminal_idx_k = no_steps * conf.no_mpc_samples_per_step - k
            u_k = mpc.buildSolveOCP(x_k, zmp_ref_k, terminal_idx_k)
            k += 1

            simulator.addSphereMarker(
                np.array([x_k[0], x_k[2], 0]), radius=0.02, color=[0, 1, 0, 1]
            )

        ########################################################################
        # update the foot spline 
        ########################################################################

        if i >= 0 and i % conf.no_sim_per_step == 0: #>>>>TODO: when to update spline
            #>>>>TODO: Start next step
            t_step_elapsed = 0.0
            plan_idx += 1

            if initialized:
                t_step_elapsed = 0.0
                curr_support = curr_swing
                if curr_support == Side.RIGHT:
                    curr_swing = Side.LEFT
                else:  
                    curr_swing = Side.RIGHT

            if curr_support == Side.RIGHT:
                curr_swing = Side.LEFT
                curr_swing_pos = robot.stack.get_placement_LF()
            else:
                curr_swing = Side.RIGHT
                curr_swing_pos = robot.stack.get_placement_RF()

            robot.setSupportFoot(curr_support)
            robot.setSwingFoot(curr_swing)

            T_swing_w = plan[plan_idx].copy()
            foot_traj = SwingFootTrajectory(curr_swing_pos, T_swing_w, conf.step_dur)

            initialized = True
            
        ########################################################################
        # in every iteration when walking
        ########################################################################
        
        if i >= 0:
            pose, vel, acc = foot_traj.evaluate(t_step_elapsed)
            robot.updateSwingFootRef(pose, vel, acc)


            interpolator.integrate(u_k)
            c, c_dot, c_ddot = interpolator.comState()

            robot.stack.setComRefState(c, c_dot, c_ddot)
            t_step_elapsed += dt

        if i % 100 == 0:
            curr_state = robot.stack.comState().pos()



        ########################################################################
        # update the simulation
        ########################################################################

        #>>>>TODO: update the simulator and the robot
        simulator.debug()
        simulator.step()
        robot.update()

        # publish to ros
        if t - t_publish > 1./30.:
            t_publish = t
            #>>>>TODO: publish
            
        # store for visualizations
        if i >= 0:
            TIME[i] = t
            #>>>>TODO: log information
            

    ########################################################################
    # enough with the simulation, lets plot
    ########################################################################
    
    import matplotlib.pyplot as plt
    plt.style.use('seaborn-dark')
    
    #>>>>TODO: plot everything

if __name__ == '__main__': 
    rospy.init_node('talos_walking')
    main()