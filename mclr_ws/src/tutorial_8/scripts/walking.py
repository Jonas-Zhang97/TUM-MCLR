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
from lip_mpc import LIPMPC, LIPInterpolator
        
################################################################################
# main
################################################################################  
    
def main(): 
    
    ############################################################################
    # setup
    ############################################################################
    
    # setup the simulator
    #>>>>TODO: simulation
    simulator = PybulletWrapper()
    
    # setup the robot
    #>>>> TODO: robot
    robot = talos.Talos(simulator)
    tsid_wrapper = TSIDWrapper(conf)

    T_swing_w = tsid_wrapper.get_placement_LF() #>>>>TODO: set intial swing foot pose to left foot
    T_support_w = tsid_wrapper.get_placement_RF() #>>>>TODO: set intial swing foot pose to right foot
    
    # setup the plan
    no_steps = 20
    planner = FootStepPlanner(conf) #>>>>TODO: Create the planner
    plan = planner.planLine(T_support_w, Side.RIGHT, no_steps, simulator, is_plot=True) #>>>>TODO: Create the plan
    #>>>>TODO: Append the two last steps once more to the plan so our mpc horizon will never run out 
    plan.append(plan[-2: ])

    # generate reference
    robot._update_zmp_estimate() 
    ZMP_ref = robot.zmp#>>>>TODO: Generate the mpc reference
    #>>>>TODO: plot the plan (make sure this workes first)
    
    # setup the lip models
    mpc = LIPMPC(conf) #>>>>TODO: setup mpc
    
    # Assume the com is over the first support foot
    x0 = plan[0].translation #>>>>TODO: Build the intial mpc state vector
    interpolator = LIPInterpolator(np.array([x0[: 2], [0, 0]]), conf) #>>>>TODO: Create the interpolator and set the inital state
    
    robot.updateSensor()
    # set the com task over the support foot
    com_state = np.c_[interpolator.comState(), np.array([robot.com_state.pos()[2], 0, 0])] #>>>>TODO: Get the com state
    #>>>>TODO: Set the COM reference to be over supporting foot 
    tsid_wrapper.setComRefState(com_state[0], com_state[1], com_state[2])
    
    ############################################################################
    # logging
    ############################################################################

    pre_dur = 3.0   # Time to wait befor walking should start
    sim_freq = 1000 # simulation frequency
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
    plan_idx = 1                                        # current index of the step within foot step plan
    t_step_elapsed = 0.0                                # elapsed time within current step (use to evaluate spline)
    t_publish = 0.0                                     # last publish time (last time we published something)
    
    for i in range(-N_pre, N_sim):
        # rint("iteration: ", i)
        t = simulator.simTime() #>>>>TODO: simulator time
        dt = simulator.stepTime() #>>>>TODO: simulator dt

        tau_sol, _ = tsid_wrapper.update(robot._q, robot._v, t)
        robot.setActuatedJointTorques(tau_sol)

        simulator.step()
        robot.update()
        tsid_wrapper.setComRefState(com_state[0], com_state[1], com_state[2])
        
        ########################################################################
        # update the mpc very no_sim_per_mpc steps
        ########################################################################
        
        if i >= 0 and None: #>>>>TODO: when to update mpc
            #>>>>TODO: Implement MPC update
            k += 1        

        ########################################################################
        # update the foot spline 
        ########################################################################

        if i >= 0 and None: #>>>>TODO: when to update spline
            #>>>>TODO: Start next step
            t_step_elapsed = 0.0
            plan_idx += 1
            
        ########################################################################
        # in every iteration when walking
        ########################################################################
        
        if i >= 0:
            t_step_elapsed += dt

        ########################################################################
        # update the simulation
        ########################################################################

        #>>>>TODO: update the simulator and the robot

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
