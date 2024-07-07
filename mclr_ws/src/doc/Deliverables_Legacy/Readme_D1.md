# Assignment 1

Author: Yueyang Zhang

## For Tutorial 1

Launch Files:

### Exercise 1

Launch the following line to check E1. You should see the markers rotating with `o_5` corner

```bash
roslaunch ros_visuals tut11.launch
```

### Exercise 2

Launch the following line to check E2. You should see the twist applied in different frames

```bash
roslaunch ros_visuals tut12.launch
```

### Exercise 3

Launch the following line to check E3. You should see the wrenchs visualized in different frames

```bash
roslaunch ros_visuals tut13.launch
```

## For Tutorial 2

### Exercise 1

Run the command to check the robot in bullet sim

```bash
rosrun bullet_sims t2_temp.py
```

### Exercise 2

Run the command to check the robot in bullet sim with controller

```bash
rosrun bullet_sims t21.py
```

### Exercise3

Run the command to check the robot in bullet sim  and RViz with controller

Open 2 terminals

```bash
rosrun bullet_sims t22.py
```

```bash
roslaunch bullet_sims t23.launch
```

## For Tutorial 3

Run the command to check the robot in bullet sim and RViz with controller and interactive marker

After the first command you should see the robot spawning in the simulator and changing controllers after 5 secs

```bash
rosrun bullet_sims t3_main.py
```

```bash
roslaunch bullet_sims t3.launch
```
