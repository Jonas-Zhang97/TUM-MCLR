# 3rd Deliverable

## Prerequest

Install drake in a python virtual environment as descriped in the official [documentation](https://drake.mit.edu/pip.html#stable-releases), all codes provided here should be ran in the environment.

Assuming the environment is named as "example_env", activate it with:

```bash
source example_env/bin/activate
```

## Tutorial 6: Linear Model Predictive Control for Walking

### a Task 1: Getting familiar

To check the pendulumn simulator, run:

```bash
python3 mclr_ws/src/tutorial_7/scripts/example_2_pydrake.py 
```

this will start the simulation without any constraints on the torque. By providing a `--torque limit` argument, an additional torque limit will be performed on the simulation, for example:

```bash
python3 mclr_ws/src/tutorial_7/scripts/example_2_pydrake.py --torque_limit=3 
```