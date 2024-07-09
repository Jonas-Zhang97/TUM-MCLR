"""
talos walking simulation
"""

import rospy
import numpy as np
import pinocchio as pin

# simulator
#>>>>TODO: import simulator
from pybullet_wrapper import PybulletWrapper
from tsid_wrapper import TSIDWrapper

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
    no_steps = 10
    planner = FootStepPlanner(conf) #>>>>TODO Create the planner
    curr_support = Side.RIGHT
    curr_swing = Side.LEFT
    plan = planner.planLine(T_support_w, curr_support, no_steps, simulator, is_plot=True) #>>>>TODO Create the plan
    #>>>>TODO Append the two last steps once more to the plan so our mpc horizon will never run out 
    plan.append(plan[-2])
    plan.append(plan[-1])

    # generate reference
    robot._update_zmp_estimate() 
    #>>>>TODO: Generate the mpc reference
    #>>>>TODO: plot the plan (make sure this workes first)
    
    # setup the lip models
    mpc = LIPMPC(conf) #>>>>TODO setup mpc
    zmp_ref = generate_zmp_reference(plan, conf.no_mpc_samples_per_step) #>>>>TODO Generate the zmp reference
    for i, zmp in enumerate(zmp_ref):
        i
        t = zmp.translation
    
    # Assume the com is over the first support foot
    x0 = plan[0].translation #>>>>TODO: Build the intial mpc state vector
    interpolator = LIPInterpolator(np.array([x0[: 2], [0, 0]]), conf) #>>>>TODO: Create the interpolator and set the inital state
    init_u_k = np.array([0, 0.086])
    interpolator.integrate(init_u_k)
    
    robot.updateSensor()
    # set the com task over the support foot
    com_state_init = np.c_[interpolator.comState(), np.array([robot.com_state.pos()[2], 0, 0])] #>>>>TODO: Get the com state
    #>>>>TODO: Set the COM reference to be over supporting foot 
    
    
    ############################################################################
    # logging
    ############################################################################

    pre_dur = 3.0   # Time to wait befor walking should start
    sim_freq = 2000 # simulation frequency
    N_pre = int(pre_dur * sim_freq) #>>>>TODO: number of sim steps before walking starts 
    N_sim = 10 * N_pre#>>>>TODO: total number of sim steps during walking
    N_mpc = N_sim / 50 #>>>>TODO: total number of mpc steps during walking
    
    #>>>>TODO: Create vectors to log all the data of the simulation
    # - COM_POS, COM_VEL, COM_ACC (from the planned reference, pinocchio and pybullet)
    # - Angular momentum (from pinocchio)
    # - Left and right foot POS, VEL, ACC (form planned reference, pinocchio) 
    # - ZMP (from planned reference, from estimator )
    # - DCM (from estimtor)
    # - Normal forces in right and left foot (from pybullet ft sensors, from pinocchio)
    TIME = np.nan*np.empty(N_sim)
    
    ############################################################################
    # logging
    ############################################################################
    
    k = 0                                               # current MPC index                          
    plan_idx = 0                                        # current index of the step within foot step plan
    t_step_elapsed = 0.0                                # elapsed time within current step (use to evaluate spline)
    t_publish = 0.0                                     # last publish time (last time we published something)

    is_init_logged = False  
    
    for i in range(-N_pre, N_sim):
        # rint("iteration: ", i)
        t = simulator.simTime() #>>>>TODO: simulator time
        dt = simulator.stepTime() #>>>>TODO: simulator 0.001

        robot.updateSensor()

        # robot._solve(t, dt)
        tau_sol, _ = robot.stack.update(robot.robot._q, robot.robot._v, t)
        robot.robot.setActuatedJointTorques(tau_sol)
        
        # Move CoM to support foot
        if 2.0 < t < 4:
            if not is_init_logged:
                rospy.loginfo("Waiting for the robot to stabilize")
                is_init_logged = True
            # get current COM height
            com_height = robot.stack.comState().pos()[2]
            # get right foot position
            rf_position = robot.stack.get_placement_RF().translation[: 2]
            # move COM position
            robot.stack.setComRefState(np.array([rf_position[0], rf_position[1], com_height]))

        
        ########################################################################
        # update the mpc very no_sim_per_mpc steps
        ########################################################################
        
        if i >= 0 and i % conf.no_sim_per_mpc == 0: #>>>>TODO: when to update mpc
            #>>>>TODO: Implement MPC update
            x_k = interpolator.x
            zmp_ref_k = zmp_ref[k: k + conf.no_mpc_samples_per_horizon]
            terminal_idx_k = no_steps * conf.no_mpc_samples_per_step - k
            u_k = mpc.buildSolveOCP(x_k, zmp_ref_k, terminal_idx_k)
            k += 1        

        ########################################################################
        # update the foot spline 
        ########################################################################

        if i >= 0 and i % conf.no_sim_per_step == 0: #>>>>TODO: when to update spline
            #>>>>TODO: Start next step
            t_step_elapsed = 0.0
            plan_idx += 1

            if curr_support == Side.RIGHT:
                curr_swing_pos = robot.stack.get_placement_LF()
            else:
                curr_swing_pos = robot.stack.get_placement_RF()

            robot.setSupportFoot(curr_support)
            robot.setSwingFoot(curr_swing)

            if curr_support == Side.RIGHT:
                curr_swing_pos = robot.stack.get_placement_LF()
            else:
                curr_swing_pos = robot.stack.get_placement_RF()
            T_swing_w = plan[plan_idx].copy()
            foot_traj = SwingFootTrajectory(curr_swing_pos, T_swing_w, conf.step_dur)
            
        ########################################################################
        # in every iteration when walking
        ########################################################################
        
        if i >= 0:
            pose, vel, acc = foot_traj.evaluate(t_step_elapsed)
            if curr_swing == Side.LEFT:
                robot.stack.set_LF_pose_ref(pin.SE3(np.eye(3), pose), vel, acc)
            else:
                robot.stack.set_RF_pose_ref(pin.SE3(np.eye(3), pose), vel, acc)

            x_k = interpolator.integrate(u_k)
            com_state = interpolator.comState()
            com_state = np.c_[com_state, np.array([robot.com_state.pos()[2], 0, 0])]
            robot.stack.setComRefState(com_state[0], com_state[1], com_state[2])
            
            t_step_elapsed += dt

            if t_step_elapsed >= conf.step_dur:
                t_step_elapsed = 0.0
                curr_support = curr_swing
                if curr_support == Side.RIGHT:
                    curr_swing = Side.LEFT
                else:
                    curr_swing = Side.RIGHT
                
                robot.setSupportFoot(curr_support)
                robot.setSwingFoot(curr_swing)


        ########################################################################
        # update the simulation
        ########################################################################

        #>>>>TODO: update the simulator and the robot
        simulator.debug()
        simulator.step()
        robot.robot.update()

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
