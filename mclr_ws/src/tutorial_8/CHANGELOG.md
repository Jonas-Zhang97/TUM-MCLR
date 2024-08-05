# Debug Log: Tutorial 8

## Bug description

1. Robot spawned on wrong pose, leading to a wrong initial foot SE3 pose
2. Robot does not initialize its pose to stand on one leg

## Solution

1. Change the parameters in talos_conf.py (all params related with q_home)
2. Debug the _solve() functuion in talos.py