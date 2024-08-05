import numpy as np
import ndcurves as curves
# Only if like to use this one, otherwise scipy, numpy, etc...
# import pinocchio as pin

import matplotlib.pyplot as plt


class SwingFootTrajectory:
    """SwingFootTrajectory
    Interpolate Foot trajectory between SE3 T0 and T1
    """

    def __init__(self, T0, T1, duration, height=0.05):
        """initialize SwingFootTrajectory

        Args:
            T0 (pin.SE3): Inital foot pose
            T1 (pin.SE3): Final foot pose
            duration (float): step duration
            height (float, optional): setp height. Defaults to 0.05.
        """
        self._height = height
        self._t_elapsed = 0.0
        self._duration = duration
        self.reset(T0, T1)

    def reset(self, T0, T1):
        """reset back to zero, update poses"""
        # >>>>TODO: plan the spline

        waypoint_0 = T0
        waypoint_2 = T1

        waypoint_1 = (waypoint_0 + waypoint_2) / 2 + np.array([0.0, 0.0, self._height])
        # waypoint_2[0] += 0.01

        waypoints = np.matrix([waypoint_0, waypoint_1, waypoint_2]).T

        c = curves.curve_constraints()
        c.init_vel = np.matrix([0, 0, 0]).T
        c.end_vel = np.matrix([0, 0, 0]).T
        c.init_acc = np.matrix([0, 0, 0]).T
        c.end_acc = np.matrix([0, 0, 0]).T

        self._spline = curves.bezier(waypoints, c, 0, self._duration)

        # p0 = T0
        # p2 = T1
        # p1 = p0 + 0.5*(p2-p0)
        # p1[2] = self._height

        # p = np.matrix([p0,p1,p2]).T
        # t = np.matrix([0, self._duration / 2, self._duration]).T
        # self._spline = curves.exact_cubic(p, t, c)

    def isDone(self):
        return self._t_elapsed >= self._duration

    def evaluate(self, t):
        """evaluate at time t"""
        # >>>>TODO: evaluate the spline at time t, return pose, velocity, acceleration
        pos = self._spline(t)
        vel = self._spline.derivate(t, 1)
        acc = self._spline.derivate(t, 2)
        return pos, vel, acc


if __name__ == "__main__":
    T0 = np.array([0, 0, 0])
    T1 = np.array([0.2, 0, 0])

    # >>>>TODO: plot to make sure everything is correct
    duration = 5.0

    traj = SwingFootTrajectory(T0, T1, duration, height=0.2)

    # plot
    t = []
    pos_array = []
    vel_array = []
    acc_array = []

    for i in range(100):
        current_t = duration / 100 * i
        t.append(current_t)
        pos, vel, acc = traj.evaluate(current_t)
        pos_array.append(pos)
        vel_array.append(vel)
        acc_array.append(acc)

    fig, ax = plt.subplots(3, 1)
    ax[0].set_title("Position")
    ax[0].plot(t,pos_array)
    ax[0].legend(["x", "y", "z"])

    ax[1].set_title("Velocity")
    ax[1].plot(t,vel_array)
    ax[1].legend(["x", "y", "z"])

    ax[2].set_title("Acceleration")
    ax[2].plot(t,acc_array)
    ax[2].legend(["x", "y", "z"])

    plt.show()
