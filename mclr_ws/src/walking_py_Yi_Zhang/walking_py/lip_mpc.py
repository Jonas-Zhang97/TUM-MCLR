import numpy as np

from pydrake.all import MathematicalProgram, Solve

################################################################################
# Helper fnc
################################################################################


def continious_LIP_dynamics(g, h):
    """returns the static matrices A,B of the continious LIP dynamics"""
    # >>>>TODO: Compute
    w = g / h
    A = np.array([[0, 1, 0, 0], [w, 0, 0, 0], [0, 0, 0, 1], [0, 0, w, 0]])
    B = np.array([[0, 0], [-w, 0], [0, 0], [0, -w]])
    return A, B


def discrete_LIP_dynamics(g, h, dt):
    """returns the matrices static Ad,Bd of the discretized LIP dynamics"""
    # >>>>TODO: Compute
    w = np.sqrt(g / h)
    t = dt
    A_d = np.array(
        [[np.cosh(w * t), 1 / w * np.sinh(w * t)], [w * np.sinh(w * t), np.cosh(w * t)]]
    )
    B_d = np.array([[1 - np.cosh(w * t)], [-w * (np.sinh(w * t))]])
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
        # >>>>TODO: Finish
        self.x_dot = np.array([0, 0, 0, 0])
        self.A, self.B = continious_LIP_dynamics(self.conf.g, self.conf.h)

    def integrate(self, u):
        # >>>>TODO: integrate with dt
        self.x_dot = self.A @ self.x + self.B @ u
        self.x += self.dt * self.x_dot
        return self.x

    def comState(self):
        # >>>>TODO: return the center of mass state
        # that is position \in R3, velocity \in R3, acceleration \in R3
        c = np.array([self.x[0], self.x[2], self.conf.h])
        c_dot = np.array([self.x_dot[0], self.x_dot[2], 0])
        c_ddot = np.array([self.x_dot[1], self.x_dot[3], 0])
        return c, c_dot, c_ddot

    def dcm(self):
        # >>>>TODO: return the computed dcm
        x, x_dot, _ = self.comState()
        dcm = x + x_dot / (self.conf.g / self.conf.h)
        return dcm

    def zmp(self):
        # >>>>TODO: return the zmp
        x, _, _ = self.comState()
        zmp = x - self.h / self.g
        return zmp


################################################################################
# LIPMPC
################################################################################


class LIPMPC:
    def __init__(self, conf):
        self.conf = conf
        self.dt = conf.dt_mpc
        self.no_samples = conf.no_mpc_samples_per_horizon

        # solution and references over the horizon
        self.X_k = None
        self.U_k = None
        self.ZMP_ref_k = None

        self.Ad, self.Bd = discrete_LIP_dynamics(conf.g, conf.h, conf.dt_mpc)

    def buildSolveOCP(self, x_k, ZMP_ref_k, terminal_idx):
        """build and solve ocp

        Args:
            x_k (_type_): inital mpc state
            ZMP_ref_k (_type_): zmp reference over horizon
            terminal_idx (_type_): index within horizon to apply terminal constraint

        Returns:
            _type_: control
        """

        # >>>>TODO: build and solve the ocp
        # >>>>Note: start without terminal constraints
        nx = 4
        nu = 2
        prog = MathematicalProgram()

        state = prog.NewContinuousVariables(self.no_samples, nx, "state")
        control = prog.NewContinuousVariables(self.no_samples, nu, "control")

        # 1. intial constraint
        for i in range(nx):
            prog.AddConstraint(state[0, i] == x_k[i])

        # 2. at each time step: respect the LIP descretized dynamics
        for i in range(self.no_samples - 1):  # tbc

            a = self.Ad @ (state[i].reshape(2, 2).T)
            b = self.Bd @ np.array([control[i]])
            x_next = (a + b).T.reshape(4)

            for j in range(nx):
                prog.AddConstraint(state[i + 1, j] == x_next[j])

        # 3. at each time step: keep the ZMP within the foot sole (use the footprint and planned step position)
        for i in range(self.no_samples):
            xmin = ZMP_ref_k[i][0] - self.conf.lfxn
            xmax = ZMP_ref_k[i][0] + self.conf.lfxp
            ymin = ZMP_ref_k[i][1] - self.conf.lfyn
            ymax = ZMP_ref_k[i][1] + self.conf.lfyp
            prog.AddBoundingBoxConstraint(xmin, xmax, control[i][0])
            prog.AddBoundingBoxConstraint(ymin, ymax, control[i][1])

        # 4. if terminal_idx < self.no_samples than we have the terminal state within
        # the current horizon. In this case create the terminal state (foot step pos + zero vel)
        # and apply the state constraint to all states >= terminal_idx within the horizon

        if terminal_idx < self.no_samples:  # when mpc samples less than horizon 16
            for i in range(terminal_idx, self.no_samples):
                prog.AddConstraint(state[i, 0] == ZMP_ref_k[-1][0])
                prog.AddConstraint(state[i, 1] == 0)
                prog.AddConstraint(state[i, 2] == ZMP_ref_k[-1][1])
                prog.AddConstraint(state[i, 3] == 0)

        # setup our cost: minimize zmp error (tracking), minimize CoM velocity (smoothing)
        # >>>>TODO: add the cost at each timestep, hint: prog.AddCost
        for i in range(self.no_samples - 1):
            prog.AddCost(
                self.conf.alpha
                * (
                    (control[i, 0] - ZMP_ref_k[i][0]) ** 2
                    + (control[i, 1] - ZMP_ref_k[i][1]) ** 2
                )
                + self.conf.gamma * (state[i][1] ** 2 + state[i][3] ** 2)
            )

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
    """generate the zmp reference given a sequence of footsteps"""

    # >>>>TODO: use the previously footstep type to build the reference
    # trajectory for the zmp
    zmp_array = []
    for foot_step in foot_steps:
        for i in range(no_samples_per_step):
            zmp_array.append(foot_step.pose.translation)
    zmp_ref = np.array(zmp_array)
    return zmp_ref
