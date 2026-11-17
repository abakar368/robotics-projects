# 2D Robot Arm Visualizer

## Overview

A Python-based 2D robotic arm simulation that demonstrates forward kinematics (FK), inverse kinematics (IK), and robot arm visualization.

The project models a planar 2-link robotic arm and calculates the joint configurations required to reach a specified target position.

## Features

- Two-link planar robot arm model
- Analytical inverse kinematics using the cosine rule
- Both elbow-up and elbow-down solutions
- Forward kinematics for end-effector position calculation
- Reachability checking
- Visualization of the robot arm and target position
- FK-based verification of IK solutions

## Project Structure

```text
01_2d_robot_visualizer/
├── ik_2d.py
├── visualizer.py
├── main.py
└── README.md
```

## Requirements

- Python 3
- Matplotlib

Install Matplotlib:

```bash
pip install matplotlib
```

## Run

```bash
python3 main.py
```

Enter the desired X and Y target coordinates when prompted.

## Mathematical Model

For link lengths L1 and L2, the forward kinematics are:

x = L1*cos(theta1) + L2*cos(theta1 + theta2)

y = L1*sin(theta1) + L2*sin(theta1 + theta2)

Inverse kinematics calculates the joint angles from a desired end-effector position.

## Verification

Each IK solution is verified using forward kinematics to confirm that the calculated end-effector position matches the requested target.

## Technologies

- Python
- NumPy-free analytical trigonometry
- Matplotlib
- Robotics kinematics