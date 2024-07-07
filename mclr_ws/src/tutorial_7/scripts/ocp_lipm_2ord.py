"""Task2: Linear inverted pendulum Trajectory planning

The goal of this file is to formulate the optimal control problem (OCP)
in equation 12. 

In this case we will solve the trajectory planning over the entire footstep plan
(= horizon) in one go.

Our state will be the position and velocity of the pendulum in the 2d plane.
x = [cx, vx, cy, vy]
And your control the ZMP position in the 2d plane
u = [px, py]

You will need to fill in the TODO to solve the task.
"""

import numpy as np

from pydrake.all import MathematicalProgram, Solve

import matplotlib.pyplot as plt
import matplotlib.patches as patches

plt.style.use('seaborn-dark')

################################################################################
# settings
################################################################################

# Robot Parameters:
# --------------

h           = 0.80   # fixed CoM height (assuming walking on a flat terrain)
g           = 9.81   # norm of the gravity vector
foot_length = 0.10   # foot size in the x-direction
foot_width  = 0.06   # foot size in the y-direciton

# OCP Parameters:
# --------------
T                     = 0.1                                # fixed sampling time interval of computing the ocp in [s]
STEP_TIME             = 0.8                                # fixed time needed for every foot step [s]

NO_SAMPLES_PER_STEP   = int(round(STEP_TIME/T))            # number of ocp samples per step

NO_STEPS              = 10                                 # total number of foot steps in the plan
TOTAL_NO_SAMPLES      = NO_SAMPLES_PER_STEP*NO_STEPS       # total number of ocp samples over the complete plan (= Horizon)

# Cost Parameters:
# ---------------
alpha       = 10**(-1)                                      # ZMP error squared cost weight (= tracking cost)
gamma       = 10**(-3)                                      # CoM velocity error squared cost weight (= smoothing cost)

################################################################################
# helper function for visualization and dynamics
################################################################################

def generate_foot_steps(foot_step_0, step_size_x, no_steps):
    """Write a function that generates footstep of step size = step_size_x in the 
    x direction starting from foot_step_0 located at (x0, y0).
    
    Args:
        foot_step_0 (_type_): first footstep position (x0, y0)
        step_size_x (_type_): step size in x direction
        no_steps (_type_): number of steps to take
    """

    #>>>>TODO: generate the foot step plan with no_steps
    #>>>>Hint: Check the pdf Fig.3 for inspiration
    foot_steps = [foot_step_0]
    for i in range(no_steps - 1):
        pass
        foot_steps.append([foot_steps[-1][0] + step_size_x, foot_steps[0][1] * (-1)**(i + 1)])
    return np.array(foot_steps)


def plot_foot_steps(foot_steps, XY_foot_print, ax):
    """Write a function that plots footsteps in the xy plane using the given
    footprint (length, width)
    You can use the function ax.fill() to gerneate a colored rectanges.
    Color the left and right steps different and check if the step sequence makes sense.

    Args:
        foot_steps (_type_): the foot step plan
        XY_foot_print (_type_): the dimensions of the foot (x,y)
        ax (_type_): the axis to plot on
    """
    #>>>>TODO: Plot the the footsteps into ax 
    # Loop over the foot steps and add a rectangle to the plot
    for i, foot_step in enumerate(foot_steps):
        # color the left and right steps different
        if i % 2 == 0:
            color = 'blue'
        else:
            color = 'red'
        # add a rectangle to the plot
        ax.add_patch(patches.Rectangle((foot_step[0] - (XY_foot_print[0] / 2), foot_step[1] - (XY_foot_print[1] / 2)), XY_foot_print[0], XY_foot_print[1], color=color))
    # set the axis limits
    # ax.set_xlim(min(foot_steps, key=lambda x: x[0])[0], max(foot_steps, key=lambda x: x[0])[0] + XY_foot_print[0])
    # ax.set_ylim(min(foot_steps, key=lambda x: x[1])[1], max(foot_steps, key=lambda x: x[1])[1] + XY_foot_print[1])

    plt.show()

def generate_zmp_reference(foot_steps, no_samples_per_step):
    """generate a function that computes a referecne trajecotry for the ZMP
    (We need this for the tracking cost in the cost function of eq. 12)
    Remember: Our goal is to keep the ZMP at the footstep center within each step.
    So for the # of samples a step is active the zmp_ref should be at that step.
    
    Returns a vector of size (TOTAL_NO_SAMPLES, 2)

    Args:
        foot_steps (_type_): the foot step plan
        no_samples_per_step (_type_): number of sampes per step
    """
    #>>>>TODO: Generate the ZMP reference based on given foot_steps
    zmp_ref = []

    for foot_step in foot_steps:
        for _ in range(no_samples_per_step):
            zmp_ref.append(foot_step)
    return np.array(zmp_ref)

################################################################################
# Dynamics of the simplified walking model
################################################################################

def continious_LIP_dynamics(g, h):
    """returns the matrices A,B of the continious LIP dynamics

    Args:
        g (_type_): gravity
        h (_type_): fixed height

    Returns:
        np.array: A, B
    """
    #>>>>TODO: Generate A, B for the continous linear inverted pendulum
    #>>>>Hint: Look at Eq. 4 and rewrite as a system first order diff. eq.
    # FIXME: not quite right
    omega = np.sqrt(g / h)

    cosh = np.cosh(omega * T)
    sinh = np.sinh(omega * T)

    A = np.array([[cosh, sinh / omega], [omega * sinh, cosh]])
    B = np.array([[1 - cosh], [-omega * sinh]])

    return A, B

def discrete_LIP_dynamics(delta_t, g, h):
    """returns the matrices static Ad,Bd of the discretized LIP dynamics

    Args:
        delta_t (_type_): discretization steps
        g (_type_): gravity
        h (_type_): height

    Returns:
        _type_: _description_
    """
    #>>>>TODO: Generate Ad, Bd for the discretized linear inverted pendulum
    omega = np.sqrt(g / h)

    cosh = np.cosh(omega * delta_t)
    sinh = np.sinh(omega * delta_t)

    Ad = np.array([[cosh, sinh / omega], [omega * sinh, cosh]])
    Bd = np.array([[1 - cosh], [-omega * sinh]])
    return Ad, Bd

################################################################################
# setup the plan references and system matrices
################################################################################

# inital state in x0 = [px0, vx0]
x_0 = np.array([0.0, 0.0])
# inital state in y0 = [py0, vy0]
y_0 = np.array([-0.09, 0.0])

# footprint
footprint = np.array([foot_length, foot_width])

# generate the footsteps
step_size = 0.2
#>>>>TODO: 1. generate the foot step plan using generate_foot_steps
foot_steps = generate_foot_steps([0.0, -0.1], step_size, NO_STEPS)

# zmp reference trajecotry
#>>>>TODO: 2. generate the ZMP reference using generate_zmp_reference
zmp_ref = generate_zmp_reference(foot_steps, NO_SAMPLES_PER_STEP)

#>>>>Note: At this point you can already start plotting things to see if they
# really make sense!
# plot_foot_steps(foot_steps, footprint, plt.gca())

# discrete LIP dynamics
#>>>>TODO: get the static dynamic matrix Ad, Bd
Ad, Bd = discrete_LIP_dynamics(T, g, h)

# continous LIP dynamics
#>>>>TODO: get the static dynamic matrix A, B
A, B = continious_LIP_dynamics(g, h)

################################################################################
# problem definition
################################################################################

# Define an instance of MathematicalProgram 
prog = MathematicalProgram() 

################################################################################
# variables
nx = 4 #>>>>TODO: State dimension = ?
nu = 2 #>>>>TODO: control dimension = ?

state = prog.NewContinuousVariables(TOTAL_NO_SAMPLES, nx, 'state')           # shape = [TOTAL_NO_SAMPLES, nx] = [80, 4]
control = prog.NewContinuousVariables(TOTAL_NO_SAMPLES, nu, 'control')       # shape = [TOTAL_NO_SAMPLES, nu] = [80, 2]

# intial state
state_inital = np.array([foot_steps[0][0], 0, foot_steps[0][1], 0]) #>>>>TODO: inital state if based on first footstep (+ zero velo)

# terminal state
state_terminal = np.array([foot_steps[-1][0], 0, foot_steps[-1][1], 0]) #>>>>TODO: terminal state if based on last footstep (+ zero velo)

################################################################################
# constraints

# 1. intial constraint
#>>>>TODO: Add inital state constrain, Hint: prog.AddConstraint
# AddConstraint(self, arg0: pydrake.symbolic.Formula)
for i in range(nx):
    prog.AddConstraint(state[0, i] == state_inital[i])

# 2. terminal constraint
#>>>>TODO: Add terminal state constrain, Hint: prog.AddConstraint
for i in range(nx):
    prog.AddConstraint(state[-1, i] == state_terminal[i])

# 3. at each step: respect the LIP descretized dynamics
#>>>>TODO: Enforce the dynamics at every time step
for i in range(TOTAL_NO_SAMPLES - 1):
    Ad @ np.array([state[i, 0], state[i, 2]]) 
    x_new = Ad @ np.array([[state[i, 0], state[i, 2]], [state[i, 1], state[i, 3]]]) + (Bd @ np.array([control[i]]))

    prog.AddConstraint(state[i + 1, 0] == x_new[0, 0])
    prog.AddConstraint(state[i + 1, 1] == x_new[1, 0])
    prog.AddConstraint(state[i + 1, 2] == x_new[0, 1])
    prog.AddConstraint(state[i + 1, 3] == x_new[1, 1])
    # prog.AddConstraint(state[i + 1] == Ad @ state[i] + Bd @ control[i])

# 4. at each step: keep the ZMP within the foot sole (use the footprint and planned step position)
#>>>>TODO: Add ZMP upper and lower bound to keep the control (ZMP) within each footprints
#Hint: first compute upper and lower bound based on zmp_ref then add constraints.
#Hint: Add constraints at every time step
for i in range(NO_STEPS):
    for j in range(NO_SAMPLES_PER_STEP):
        prog.AddConstraint(control[i * NO_SAMPLES_PER_STEP + j, 0] <= foot_steps[i][0] + footprint[0] / 2)
        prog.AddConstraint(control[i * NO_SAMPLES_PER_STEP + j, 0] >= foot_steps[i][0] - footprint[0] / 2)
        prog.AddConstraint(control[i * NO_SAMPLES_PER_STEP + j, 1] <= foot_steps[i][1] + footprint[1] / 2)
        prog.AddConstraint(control[i * NO_SAMPLES_PER_STEP + j, 1] >= foot_steps[i][1] - footprint[1] / 2)

################################################################################
# stepwise cost, note that the cost function is scalar!

# setup our cost: minimize zmp error (tracking), minimize CoM velocity (smoothing)
#>>>>TODO: add the cost at each timestep, hint: prog.AddCost

for i in range(TOTAL_NO_SAMPLES-1):
    prog.AddCost(alpha * ((control[i,0] - zmp_ref[i][0])**2  + (control[i,1] - zmp_ref[i][1])**2) + 
                 gamma * (state[i][1]**2 + state[i][3]**2))

################################################################################
# solve

result = Solve(prog)
if not result.is_success:
    print("failure")
print("solved")

# extract the solution
#>>>>TODO: extract your variables from the result object
state_sol = result.GetSolution(state)
control_sol = result.GetSolution(control)

t = T*np.arange(0, TOTAL_NO_SAMPLES)

# compute the acceleration
#>>>>TODO: compute the acceleration of the COM
acc_x =  (g / h) * (state_sol[:,0] - control_sol[:,0])
acc_y =  (g / h) * (state_sol[:,2] - control_sol[:,1])

################################################################################
# plot something

fig_x, axs_x = plt.subplots(3, 1)
fig_y, axs_y = plt.subplots(3, 1)
fig_xy, ax_xy = plt.subplots()

#>>>>TODO: plot everything in x-axis
axs_x[0].plot(t, state_sol[:, 0], label='x pos of CoM')
axs_x[0].plot(t, control_sol[:, 0], label='x pos of ZMP')
axs_x[0].plot(t, zmp_ref[:, 0], label='x reference pos of ZMP')
axs_x[0].plot(t, zmp_ref[:, 0] - footprint[0] / 2, label='x lower bound of ZMP')
axs_x[0].plot(t, zmp_ref[:, 0] + footprint[0] / 2, label='x upper bound of ZMP')

axs_x[1].plot(t, state_sol[:, 1], label='x vel of CoM')
axs_x[2].plot(t, acc_x, label='x acc of CoM')

axs_x[0].legend(loc='upper left', shadow=True)
axs_x[1].legend(loc='upper left', shadow=True)
axs_x[2].legend(loc='upper left', shadow=True)

#>>>>TODO: plot everything in y-axis
axs_y[0].plot(t, state_sol[:, 2], label='y pos of CoM')
axs_y[0].plot(t, control_sol[:, 1], label='y pos of ZMP')
axs_y[0].plot(t, zmp_ref[:, 1], label='y reference pos of ZMP')
axs_y[0].plot(t, zmp_ref[:, 1] - footprint[1] / 2, label='y lower bound of ZMP')
axs_y[0].plot(t, zmp_ref[:, 1] + footprint[1] / 2, label='y upper bound of ZMP')
axs_y[1].plot(t, state_sol[:, 3], label='y vel of CoM')
axs_y[2].plot(t, acc_y, label='y acc of CoM')

axs_y[0].legend(loc='upper left', shadow=True)
axs_y[1].legend(loc='upper left', shadow=True)
axs_y[2].legend(loc='upper left', shadow=True)

#>>>>TODO: plot everything in xy-plane
ax_xy.plot(state_sol[:, 0], state_sol[:, 2], label='CoM traj')
ax_xy.plot(control_sol[:, 0], control_sol[:, 1], label='ZMP traj')
plot_foot_steps(foot_steps, footprint, ax_xy)

plt.show()