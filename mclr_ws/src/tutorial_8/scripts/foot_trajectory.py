import numpy as np
import ndcurves as curves # Only if like to use this one, otherwise scipy, numpy, etc...
import pinocchio as pin
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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
        self.spline = self.reset(T0, T1)

    def reset(self, T0, T1, test=False):
        '''reset back to zero, update poses
        '''
        #>>>>TODO: plan the spline
        
        # desired trajectory:
        # -a \cdot \left( x^2 - 0.2 x \right)
        # a = \frac{4 \cdot \text{step_height}}{\text{step_length}^2}
        # by taking the derivative of the above equation, we have:
        # -2a \cdot x + 0.2a
        # let x be:
        # x = \frac{\text{step_size}}{2}
        # we get the control point at the middle of the trajectory
        # p1 = \left( \frac{\text{step_size}}{2}, ~ 0, ~ 2 \cdot \text{step_height} \right)

        way_pts = [T0.translation.tolist(), 
                   [(T1.translation[0] - T0.translation[0]) / 2, (T1.translation[2] - T0.translation[1]) / 2, self._height * 2], 
                   T1.translation.tolist()]
        way_pts = np.array(way_pts).T
        bezier_ref = curves.bezier(way_pts, 0, self._duration)
        if test:
            numSamples = 10
            fNumSamples = float(numSamples)
            ptsTime = [
                (bezier_ref(float(t) / fNumSamples), float(t) / fNumSamples) for t in range(numSamples + 1)
            ]
            for el in ptsTime:
                print(el)
        return bezier_ref

    def isDone(self):
        return self._t_elapsed >= self._duration 
    
    def evaluate(self, t):
        """evaluate at time t
        """
        #>>>>TODO: evaluate the spline at time t, return pose, velocity, acceleration
        pose = self.spline(t)
        vel = self.spline.derivate(t, 1)
        acc = self.spline.derivate(t, 2)
        return pose, vel, acc

if __name__=="__main__":
    T0 = pin.SE3(np.eye(3), np.array([0, 0, 0]))
    T1 = pin.SE3(np.eye(3), np.array([0.2, 0, 0]))

    sft = SwingFootTrajectory(T0, T1, 0.8)

    #>>>>TODO: plot to make sure everything is correct
    t = np.linspace(0, 0.8, 100)
    poses = []
    vels = []
    accs = []
    for i in t:
        pose, vel, acc = sft.evaluate(i)
        poses.append(pose)
        vels.append(vel)
        accs.append(acc)

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    x_line = [pose[0] for pose in poses]
    y_line = [pose[1] for pose in poses]
    z_line = [pose[2] for pose in poses]
    ax.plot3D(x_line, y_line, z_line, 'gray')
    ax.scatter(x_line, y_line, z_line, c=z_line, cmap='Greens')
    plt.show()