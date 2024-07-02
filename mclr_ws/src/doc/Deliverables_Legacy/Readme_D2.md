# 2nd Deliverable

This folder contains scripts for 2nd assignment of MCLR.

To run the separate homework, follow the instructions below.

## Tutorial 4: Whole-body control

### Exercise 1: Getting the robot to stand

Open a terminal, source the workspace, and then run:

```bash
roslaunch bullet_sims t41.launch
```

You should see the robot spawning in the bullet simulation and rviz and standing steadily.

### Exercise 2: Getting the robot to balance on foot

Run the command:

```bash
roslaunch bullet_sims t42.launch
```

After spawned, the robot will move its center of mass, and after 2 seconds, it will stand just with the right leg.

### Exercise 3: Getting the robot to do squats & Adding arm motions

Run:

```bash
roslaunch bullet_sims t4.launch
```

Along with the one leg standing, the robot will squat for a few seconds and then moves its arm in a circle. You will also see a live plot showing the 3 types of hights of center of mass (only z-axis, the rest are constant values).

## Tutorial 6: Balance Control

Report of this tutorial can be found [here](./bullet_sims/doc/report.md).

All the launch command below will bring up the robot in bullet simulation and rviz, along with a live plotter showing the ZMP, CMP and CP (x- and y-axis).

### Torque Control Modality

For **no strategy** run:

```bash
roslaunch bullet_sims t61.launch balance_strategy:=none
```

For **ankle strategy** run:

```bash
roslaunch bullet_sims t61.launch balance_strategy:=ankle
```

For **hip strategy** run:

```bash
roslaunch bullet_sims t61.launch balance_strategy:=hip
```

For **combined strategy** run:

```bash
roslaunch bullet_sims t61.launch balance_strategy:=combined
```

### Position Control Modality

For **no strategy** run:

```bash
roslaunch bullet_sims t62.launch balance_strategy:=none
```

For **ankle strategy** run:

```bash
roslaunch bullet_sims t62.launch balance_strategy:=ankle
```

For **hip strategy** run:

```bash
roslaunch bullet_sims t62.launch balance_strategy:=hip
```

For **combined strategy** run:

```bash
roslaunch bullet_sims t62.launch balance_strategy:=combined
```

