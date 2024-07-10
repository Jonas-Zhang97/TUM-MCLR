# 3rd Deliverable

The provided code base is the content of `src` folder of the workspace. To create workspace:

```bash
mkdir -p ~/mclr_ws/src
```

Copy the content in the deliverable into the created `src` folder and then navigate to the workspace:

```bash
cd ~/mclr_ws
```

## Prerequest

Drake is needed for solving MPC problems, it is recommanded to install it globally:

```bash
pip install pip --upgrade
pip install drake
```

For tutorial 7, trajectories are generated via `ndcurves`, run the following command for a system-wide installation:

```bash
sudo apt install robotpkg-py3\*-ndcurves
```

## Tutorial 6: Linear Model Predictive Control for Walking

### a Task 1: Getting familiar

To check the pendulumn simulator, run:

```bash
python3 src/tutorial_7/scripts/example_2_pydrake.py 
```

this will start the simulation without any constraints on the torque. By providing a `--torque limit` argument, an additional torque limit will be performed on the simulation, for example:

```bash
python3 src/tutorial_7/scripts/example_2_pydrake.py --torque_limit=3 
```

### b Task 2: Linear inverted pendulum

In the terminal with the above mentioned virtual environment activated, run:

```bash
python3 src/tutorial_7/scripts/ocp_lipm_2ord.py 
```

### c Task 3: Linear Model Predictive Control

For this task, run

```bash
python3 src/tutorial_7/scripts/mpc_lipm_2ord.py
```

In figure 2 (plots in y-axis), you can notice an obseravable disturbance at 3.2 second in `y acc of CoM`, which corresponds to the 5th step (the 3rd blue step), where you can see that the `traj of ZMP` has a sudden change.

## Tutorial 7: Walking Control

### a Task 1: Implement Modules

To check **a.1 Foot Trajectory**, run:

```bash
python3 src/tutorial_8/scripts/foot_trajectory.py
```

a 3D plot will be showing as the generated trajectory.

For **a.2 Foot Step planner**, run:

```bash
python3 src/tutorial_8/scripts/footstep_planner.py
```

### b Task 2: Implement the Walking Cycle

Becaus I failed to finish the implementation, you can run the code with python command:

```bash
python3 src/tutorial_8/scripts/walking.py
```

You will see the robot spawning in the simulator and take a step, then it would fall, The bug is caused by mpc calculation, which has returned a wrong CoM state, I'm still working on it.