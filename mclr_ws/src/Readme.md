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

Install drake in a python virtual environment as descriped in the official [documentation](https://drake.mit.edu/pip.html#stable-releases), all codes provided here should be ran in the environment.

Assuming the environment is named as `example_env`, activate it with:

```bash
source example_env/bin/activate
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
