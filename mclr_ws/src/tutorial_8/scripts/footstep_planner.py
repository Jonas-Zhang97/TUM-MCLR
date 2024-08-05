import numpy as np
import pinocchio as pin
from enum import Enum
import talos_conf as conf
import pybullet_wrapper as pbw

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
        
        #>>>>TODO: plot in pybullet footprint, addGlobalDebugRectancle(...) 
        
        #>>>>TODO: display the side of the step, addUserDebugText(...)
        
        #>>>>TODO: plot step target position addSphereMarker(...)
        for pose in self.pose:
            simulation.addSphereMarker(pose.translation,radius=0.02)
            simulation.addGlobalDebugRectancle(pose.translation - np.array([0.0,0,0]),length=0.2, width=0.13)
            if pose.translation[1] >0 :
                simulation.addUserDebugText(parent_id=-1, link_id=-1, text = 'LEFT', x = pose.translation + np.array([0,0.05,0]))
            else:
                simulation.addUserDebugText(parent_id=-1, link_id=-1, text = 'RIGHT', x = pose.translation + np.array([0,0.05,0]))
        
        return None

class FootStepPlanner:
    """FootStepPlanner
    Creates footstep plans (list of right and left steps)
    """
    
    def __init__(self, conf):
        self.conf = conf
        self.steps = []
        
    def planLine(self, T_0_w, side, no_steps, simulator, is_plot=False):
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
        dy = 2*0.085
        
        # the footprint of the robot
        lfxp, lfxn = self.conf.lfxp, self.conf.lfxn
        lfyp, lfyn = self.conf.lfyp, self.conf.lfyn
        
        #>>>>TODO: Plan a sequence of steps with T_0_w being the first step pose.
        #>>>>Note: Plan the second step parallel to the first step (robot starts standing on both feet)
        #>>>>Note: Plan the final step parallel to the last-1 step (robot stops standing on both feet)
        print("initial step: \n", T_0_w.translation)
        steps=[T_0_w]
        if side == Side.LEFT:
            for i in range(0,no_steps):
                T_i = steps[i].copy()
                T_i.translation += np.array([dx, (-1)**(i + 1) * dy, 0])
                steps.append(T_i)
                print(i+1, "planed step: \n", T_i.translation)
        else:
            for i in range(0,no_steps):
                T_i = steps[i].copy()
                T_i.translation += np.array([dx, (-1)**i * dy, 0])
                steps.append(T_i)
                print(i+1, "planed step: \n", T_i.translation)
        
        if is_plot:
            fs = FootStep(steps, 0, side)
            #>>>>TODO: Check that the plan looks as expected
            fs.plot(simulator)

        return steps

    
    def plot(self, simulation):
        for step in self.steps:
            step.plot(simulation)

            
if __name__=='__main__':
    """ Test footstep planner
    """
    is_plot = True
    
    #>>>>TODO: Generate a plan and plot it in pybullet.
    fsp = FootStepPlanner(conf)

    T_0_w = pin.SE3.Identity()
    T_0_w.translation = np.array([0, conf.step_size_y, 0])
    
    
    #>>>>TODO: Check that the plan looks as expected
    line_id = -1
    simulator = pbw.PybulletWrapper()

    planed_steps = fsp.planLine(T_0_w, Side.LEFT, 5, simulator)
    
    fs = FootStep(planed_steps, 0, Side.LEFT)
    while is_plot == True:
        fs.plot(simulator)
