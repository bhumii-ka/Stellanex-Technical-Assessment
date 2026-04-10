# 2D Waypoint Controller for Differential Drive Robot
------------------------------------------------------------------------
Youtube Video: https://youtu.be/b2inU3ziVZ4?si=RfjLXJML_jAkMSRt

## Problem Definition

The goal of this project is to implement a **2D waypoint controller**
for a differential-drive robot.

The robot operates in a 2D plane with state: 
- Position: *(x, y)* 
- Orientation: *(θ)*

The controller computes: 
- Linear velocity (v) → forward motion
- Angular velocity (ω) → rotational motion

The objective is to: Navigate the robot from a starting pose to a target
waypoint *(x_target, y_target)* and stop within a small tolerance.

------------------------------------------------------------------------

## Setup & Deployment

### 1. Clone the repository

``` bash
git clone <your-repo-link>
cd <repo-name>
```

### 2. Install dependencies

Make sure Python (≥ 3.8) is installed.

``` bash
pip install numpy matplotlib
```

### 3. Run the script

``` bash
python waypoint_controller.py
```

### 4. Provide inputs

Enter: 
- Starting position *(x, y, θ in degrees)* 
- Target position *(x_target, y_target)*

### 5. Output

-   Final robot position in terminal
-   A plot showing:
    -   Robot path
    -   Start and target points
    -   Orientation arrows along trajectory

------------------------------------------------------------------------

## Technical Architecture

### 1. Robot Model

Standard **unicycle (differential drive) kinematics** has been used:

x(t+1) = x(t) + v \* cos(θ) \* Δt\
y(t+1) = y(t) + v \* sin(θ) \* Δt\
θ(t+1) = θ(t) + ω \* Δt

------------------------------------------------------------------------

### 2. Control Strategy

A **Proportional (P) Controller** has been implemented.

Distance error: d = sqrt((x_target - x)\^2 + (y_target - y)\^2)

Target heading: α = atan2(y_target - y, x_target - x)

Heading error: e_θ = α - θ

Normalized using: e_θ = atan2(sin(e_θ), cos(e_θ))

------------------------------------------------------------------------

### 3. Control Laws

v = k_v \* d\
ω = k_θ \* e_θ

------------------------------------------------------------------------

### 4. Visualization

Using matplotlib, the simulation plots: - Robot trajectory
- Start and target points
- Orientation arrows along the path
- Tolerance circle around the goal

------------------------------------------------------------------------

## Critical Reflection

A key design decision was using a **Proportional (P) Controller**
instead of a more advanced controller.

### Trade-offs

| Approach        | Pros                       | Cons                          |
|----------------|----------------------------|-------------------------------|
| P Controller   | Simple, easy to implement  | Less precise, may oscillate   |
| PID Controller | Better accuracy & stability| Requires tuning               |
| Pure Pursuit   | Smooth path following      | Needs predefined path         |
| MPC            | Optimal control            | Computationally expensive     |

### Future Improvements

-   Implement a PID controller
-   Add velocity constraints
-   Extend to multiple waypoints
-   Integrate with Gazebo

------------------------------------------------------------------------

## Summary

This project demonstrates: 
- Robot kinematics
- Feedback control
- Real-time simulation
- Visualization of robot motion
