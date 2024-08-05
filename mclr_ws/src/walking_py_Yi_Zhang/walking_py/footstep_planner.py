import numpy as np
import pinocchio as pin
from enum import Enum
import talos_conf as conf 

from pybullet_wrapper import PybulletWrapper

class Side(Enum):
    """Side
    Describes which foot to use
    """
    LEFT=0
    RIGHT=1

def other_foot_id(id):
    if id == Side.LEFT:
        return Side.RIGHT
    else:
        return Side.LEFT

class FootStep:
    """FootStep
    Holds all information describing a single footstep
    """
    def __init__(self, pose, footprint, side=Side.LEFT):
        """inti FootStep

        Args:
            pose (pin.SE3): the pose of the footstep
            footprint (np.array): 3 by n matrix of foot vertices
            side (_type_, optional): Foot identifier. Defaults to Side.LEFT.
        """
        self.pose = pose
        self.footprint = footprint
        self.side = side

    def poseInWorld(self):
        return self.pose

    def plot(self, simulation):

        # >>>>TODO: plot in pybullet footprint, addGlobalDebugRectancle(...)

        # >>>>TODO: display the side of the step, addUserDebugText(...)

        # >>>>TODO: plot step target position addSphereMarker(...)
        x = self.pose.translation

        simulation.addSphereMarker(x, radius=0.02)

        simulation.addGlobalDebugRectancle(
            x - np.array([0.02, 0, 0]), length=0.2, width=0.13
        )
        if self.side == Side.LEFT:
            simulation.addUserDebugText(
                parent_id=-1, link_id=-1, text="LEFT", x=x + np.array([0, 0.05, 0])
            )
        else:
            simulation.addUserDebugText(
                parent_id=-1, link_id=-1, text="RIGHT", x=x + np.array([0, 0.05, 0])
            )
        return None

class FootStepPlanner:
    """FootStepPlanner
    Creates footstep plans (list of right and left steps)
    """

    def __init__(self, conf):
        self.conf = conf
        self.steps = []

    def planLine(self, T_0_w, side, no_steps, T_support_w):
        """plan a sequence of steps in a strait line

        Args:
            T_0_w (pin.SE3): The inital starting position of the plan
            side (Side): The intial foot for starting the plan
            no_steps (_type_): The number of steps to take

        Returns:
            list: sequence of steps
        """

        # the displacement between steps in x and y direction
        dx = self.conf.step_size_x
        dy = 2*self.conf.step_size_y

        # the footprint of the robot
        lfxp, lfxn = self.conf.lfxp, self.conf.lfxn
        lfyp, lfyn = self.conf.lfyp, self.conf.lfyn

        # >>>>TODO: Plan a sequence of steps with T_0_w being the first step pose.
        # >>>>Note: Plan the second step parallel to the first step (robot starts standing on both feet)
        # >>>>Note: Plan the final step parallel to the last-1 step (robot stops standing on both feet)

        steps=[]
        first_step = np.array(
            [T_0_w.translation[0], T_0_w.translation[1], 0.0]
        )  # first step
        footprint = np.array(
            [
                first_step + np.array([lfxn, lfyp, 0]),
                first_step + np.array([-lfxp, lfyp, 0]),
                first_step + np.array([-lfxp, -lfyn, 0]),
                first_step + np.array([lfxn, -lfyn, 0]),
            ]
        )
        steps.append(
            FootStep(pin.SE3(T_0_w.rotation, first_step), footprint, side)
            )

        if side == Side.LEFT:
            side = Side.RIGHT
        else:
            side = Side.LEFT

        second_step = np.array(
            [T_support_w.translation[0], T_support_w.translation[1], 0.0]
        )  # first step
        footprint = np.array(
            [
                second_step + np.array([lfxn, lfyp, 0]),
                second_step + np.array([-lfxp, lfyp, 0]),
                second_step + np.array([-lfxp, -lfyn, 0]),
                second_step + np.array([lfxn, -lfyn, 0]),
            ]
        )
        steps.append(
            FootStep(pin.SE3(T_support_w.rotation, second_step), footprint, side)
        )

        # footprint = np.array(
        #     [
        #         second_step + np.array([lfxp, lfyp, 0]),
        #         second_step + np.array([-lfxn, lfyp, 0]),
        #         second_step + np.array([-lfxn, -lfyn, 0]),
        #         second_step + np.array([lfxp, -lfyn, 0]),
        #     ]
        # )
        # steps.append(
        #     FootStep(pin.SE3(T_0_w.rotation, second_step), footprint, side)
        # )

        current_step = second_step

        for i in range(no_steps-3):
            if side == Side.LEFT:
                current_step = current_step + np.array([dx, -dy, 0])
                side = Side.RIGHT
            else:
                current_step = current_step + np.array([dx, dy, 0])
                side = Side.LEFT
            footprint = np.array(
                [
                    current_step + np.array([lfxp, lfyp, 0]),
                    current_step + np.array([-lfxn, lfyp, 0]),
                    current_step + np.array([-lfxn, -lfyn, 0]),
                    current_step + np.array([lfxp, -lfyn, 0]),
                ]
            )

            steps.append(
                FootStep(pin.SE3(T_0_w.rotation, current_step), footprint, side)
                )

        # last step parallel to last-1 step
        if side == Side.LEFT:
            current_step = current_step + np.array([0, -dy, 0])
            side = Side.RIGHT
        else:
            current_step = current_step + np.array([0, dy, 0])
            side = Side.LEFT
        footprint = np.array(
            [
                current_step + np.array([lfxp, lfyp, 0]),
                current_step + np.array([-lfxn, lfyp, 0]),
                current_step + np.array([-lfxn, -lfyn, 0]),
                current_step + np.array([lfxp, -lfyn, 0]),
            ]
        )
        steps.append(FootStep(pin.SE3(T_0_w.rotation, current_step), footprint, side))

        self.steps = steps
        return steps

    def plot(self, simulation):
        for step in self.steps:
            step.plot(simulation)


if __name__=='__main__':
    """ Test footstep planner
    """

    # >>>>TODO: Generate a plan and plot it in pybullet.
    # >>>>TODO: Check that the plan looks as expected
    step_planner = FootStepPlanner(conf)
    T0 = pin.SE3(np.eye(3), np.array([0.0, 0.096, 0]))

    # plan
    side = Side.LEFT
    steps_line = step_planner.planLine(T0, side, 8)

    simulation = PybulletWrapper()

    step_planner.plot(simulation)

    while True:
        simulation.debug()
        simulation.step()
