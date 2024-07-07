import numpy as np

from pydrake.all import MathematicalProgram, Solve

################################################################################
# Helper fnc
################################################################################

def continious_LIP_dynamics(g, h):
    """returns the static matrices A,B of the continious LIP dynamics
    """
    #>>>> Compute
    omega = np.sqrt(g / h)

    A =np.array([[0, 1], [omega, 0]])
    B =np.array([[0, -omega]])
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
        #>>>>TODO: Finish
        self.g = conf.g
        self.h = conf.h
        self.A, self.B = continious_LIP_dynamics(conf.g, conf.h)
        
    def integrate(self, u):
        #>>>>TODO: integrate with dt
        self.x_dot = self.A @ self.x.reshape(2, 2) + self.B @ u
        self.x += (self.x_dot * self.dt).reshape(4)
        return self.x
    
    def comState(self):
        #>>>>TODO: return the center of mass state
        # that is position \in R3, velocity \in R3, acceleration \in R3
        c, c_dot, c_ddot = None, None, None
        c = self.x
        c_dot = self.x_dot
        c_ddot = self.A @ self.x.reshape(2, 2)
        return [c, c_dot, c_ddot]
    
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
    return np.array(zmp_ref)
