import numpy as np

from pydrake.all import MathematicalProgram, Solve

################################################################################
# Helper fnc
################################################################################

def continious_LIP_dynamics(g, h):
    """returns the static matrices A,B of the continious LIP dynamics
    """
    # >>>>TODO: Compute
    w = g / h
    A = np.array([[0, 1, 0, 0], [w, 0, 0, 0], [0, 0, 0, 1], [0, 0, w, 0]])
    B = np.array([[0, 0], [-w, 0], [0, 0], [0, -w]])
    return A, B

def discrete_LIP_dynamics(g, h, dt):
    """returns the matrices static Ad,Bd of the discretized LIP dynamics
    """
    #>>>> Compute
    omega = np.sqrt(g / h)

    cosh = np.cosh(omega * dt)
    sinh = np.sinh(omega * dt)

    A_d = np.array([[cosh, sinh / omega], [omega * sinh, cosh]])
    B_d = np.array([[1 - cosh], [-omega * sinh]])

    return A_d, B_d

################################################################################
# LIPInterpolator
################################################################################

class LIPInterpolator:
    """Integrates the linear inverted pendulum model using the 
    continous dynamics. To interpolate the solution to hight 
    """
    def __init__(self, x_inital, conf):
        self.conf = conf
        self.dt = conf.dt
        self.x = x_inital
        self.x_dot = np.array([0, 0, 0, 0])
        #>>>>TODO: Finish
        self.g = conf.g
        self.h = conf.h
        self.A, self.B = continious_LIP_dynamics(conf.g, conf.h)
        
    def integrate(self, u):
        #>>>>TODO: integrate with dt
        self.x_dot = self.A @ self.x + self.B @ u
        self.x += self.dt * self.x_dot
        return self.x
    
    def comState(self):
        #>>>>TODO: return the center of mass state
        # that is position \in R3, velocity \in R3, acceleration \in R3
        c = np.array([self.x[0], self.x[2], self.conf.h])
        c_dot = np.array([self.x[1], self.x[3], 0])
        c_ddot = np.array([self.x_dot[1], self.x_dot[3], 0])
        return c, c_dot, c_ddot
    
    def dcm(self):
        #>>>>TODO: return the computed dcm
        dcm = None
        com_state = self.comState()
        dcm = com_state[0] + com_state[1] / np.sqrt(self.g / self.h)
        return dcm
    
    def zmp(self):
        #>>>>TODO: return the zmp
        zmp = None
        com_state = self.comState()
        zmp = com_state[0] - com_state[1] / np.sqrt(self.g / self.h)
        return zmp
        
    
################################################################################
# LIPMPC
################################################################################

class LIPMPC:
    def __init__(self, conf):
        self.conf = conf
        self.dt = conf.dt        
        self.no_samples = conf.no_mpc_samples_per_horizon
        
        # solution and references over the horizon
        self.X_k = None
        self.U_k = None
        self.ZMP_ref_k = None
        
    def buildSolveOCP(self, x_k, ZMP_ref_k, terminal_idx):
        """build and solve ocp

        Args:
            x_k (_type_): inital mpc state
            ZMP_ref_k (_type_): zmp reference over horizon
            terminal_idx (_type_): index within horizon to apply terminal constraint

        Returns:
            _type_: control
        """
        
        #>>>>TODO: build and solve the ocp
        #>>>>Note: start without terminal constraints
        nx = 4 #>>>>TODO: State dimension = ?
        nu = 2 #>>>>TODO: control dimension = ?
        prog = MathematicalProgram()
        
        state = prog.NewContinuousVariables(self.no_samples, nx, 'state')
        control = prog.NewContinuousVariables(self.no_samples, nu, 'control')
        
        # 1. intial constraint
        #>>>>TODO: Add inital state constraint, Hint: x_k
        # x_k = x_k.reshape(4, )
        for i in range(nx):
            prog.AddConstraint(state[0, i] == x_k[i])

        Ad, Bd = discrete_LIP_dynamics(self.conf.g, self.conf.h, self.dt)
    
        # 2. at each time step: respect the LIP descretized dynamics
        #>>>>TODO: Enforce the dynamics at every time step
        for i in range(self.no_samples - 1):
            # x_new = Ad @ np.array([[state[i, 0], state[i, 2]], [state[i, 1], state[i, 3]]]) + (Bd @ np.array([control[i]]))
            for i in range(self.no_samples - 1):  # tbc

                a = Ad @ (state[i].reshape(2, 2).T)
                b = Bd @ np.array([control[i]])
                x_next = (a + b).T.reshape(4)

                for j in range(nx):
                    prog.AddConstraint(state[i + 1, j] == x_next[j])

        
        # 3. at each time step: keep the ZMP within the foot sole (use the footprint and planned step position)
        #>>>>TODO: Add ZMP upper and lower bound to keep the control (ZMP) within each footprints
        #Hint: first compute upper and lower bound based on zmp_ref then add constraints.
        #Hint: Add constraints at every time step
        for i in range(self.no_samples):
            x_upper = ZMP_ref_k[i].translation[0] + self.conf.lfxp
            x_lower = ZMP_ref_k[i].translation[0] - self.conf.lfxn
            y_upper = ZMP_ref_k[i].translation[1] + self.conf.lfyp
            y_lower = ZMP_ref_k[i].translation[1] - self.conf.lfyn
            prog.AddConstraint(control[i, 0] <= x_upper)
            prog.AddConstraint(control[i, 0] >= x_lower)
            prog.AddConstraint(control[i, 1] <= y_upper)
            prog.AddConstraint(control[i, 1] >= y_lower)
        # 4. if terminal_idx < self.no_samples than we have the terminal state within
        # the current horizon. In this case create the terminal state (foot step pos + zero vel)
        # and apply the state constraint to all states >= terminal_idx within the horizon
        #>>>>TODO: Add the terminal constraint if requires
        #Hint: If you are unsure, you can start testing without this first!]
        if terminal_idx < self.no_samples:
            for i in range(terminal_idx, self.no_samples):
                prog.AddConstraint(state[i,0] == ZMP_ref_k[-1].translation[0])
                prog.AddConstraint(state[i,1] == 0)
                prog.AddConstraint(state[i,2] == ZMP_ref_k[-1].translation[1])
                prog.AddConstraint(state[i,3] == 0)
    
        # setup our cost: minimize zmp error (tracking), minimize CoM velocity (smoothing)
        #>>>>TODO: add the cost at each timestep, hint: prog.AddCost
        for i in range(self.no_samples-1):
            prog.AddCost(self.conf.alpha * ((control[i,0] - ZMP_ref_k[i].translation[0])**2  + (control[i,1] - ZMP_ref_k[i].translation[1])**2) + 
                         self.conf.gamma * (state[i][1]**2 + state[i][3]**2))
            
        # solve
        result = Solve(prog)
        if not result.is_success:
            print("failure")
            
        self.X_k = result.GetSolution(state)
        self.U_k = result.GetSolution(control)
        if np.isnan(self.X_k).any():
            print("failure")
        
        self.ZMP_ref_k = ZMP_ref_k
        return self.U_k[0]
    
    

def generate_zmp_reference(foot_steps, no_samples_per_step):
    """generate the zmp reference given a sequence of footsteps
    """

    #>>>>TODO: use the previously footstep type to build the reference 
    # trajectory for the zmp
    zmp_ref = []

    for foot_step in foot_steps:
        for _ in range(no_samples_per_step):
            zmp_ref.append(foot_step)
    return zmp_ref
