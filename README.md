# Unified Mobile Manipulator Framework (YouBot)
**Autonomous Kinematics, Control, and Trajectory Suite**

This repository contains a high-performance C++ framework for a **9-DOF Mobile Manipulator** (4-wheel omnidirectional base + 5-DOF serial arm). The system bridges the gap between high-level task planning and low-level actuator execution, utilizing **Screw Theory** and **Modern Robotics** principles.

---

## 🏗 System Architecture

The software is organized into modular components designed for real-time performance:

### 0. Mathematical Core (`mr library`):
* **Mathematical Core**: The low-level mathematical operations, and course algorithm's (Lie Algebra, matrix exponentials, adjoint transforms, kinematics, trajectory, odometry) are isolated in the **`mr/`** folder for modularity and clarity.

### 1. Kinematics Engine (`RobotKinematics`)
The heart of the system that manages the robot's state $q \in \mathbb{R}^{12}$ (chassis, joints, and wheels).
* **Unified Jacobian**: Concatenates the chassis base Jacobian ($J_{base}$) and the arm body Jacobian ($J_{arm}$) into a single $6 \times 9$ matrix.
* **Damped Least Squares (DLS)**: Implements robust matrix inversion to handle kinematic singularities smoothly, avoiding infinite velocity commands.
* **Odometry**: Provides real-time integration of wheel speeds to update the chassis pose in the space frame.

### 2. Task-Space Controller (`RobotControl`)
A sophisticated feedback/feedforward controller that generates the required twists to follow a path.
* **Control Laws**: Supports PID, PI, and Feedforward control strategies.
* **Error Correction**: Uses $X_{err}$ (twist error) integration to eliminate steady-state tracking errors in $SE(3)$.

### 3. Trajectory Generation (`TrajectoryGen`)
Generates smooth motion profiles for the end-effector.
* **Screw Trajectory**: Follows the shortest path in $SE(3)$ using a constant screw axis.
* **Cartesian Trajectory**: Decouples translation and rotation for linear tool paths.
* **Time Scaling**: Uses **Quintic Polynomials** (5th-order) to ensure zero velocity and acceleration at start/end points.

### 4. Mission Execution (`RobotRunner`)
The high-level orchestrator that manages the lifecycle of a mission.
* **Segment Management**: Manages mission segments such as "Standoff," "Grasp," and "Place".
* **Timing Synchronization**: Manages the relationship between control frequency ($dt$), trajectory steps ($k$), and logging frequency.

### 5. Telemetry & Parameters (`CSVLogger` & `RobotParams`)
* **Data Logging**: Captures 13-element state vectors (phi, x, y, 5 arm joints, 4 wheel angles, gripper) for analysis.
* **Centralized Constants**: Defines physical robot dimensions, screw axes ($B_{list}$), and joint limits in one location.

---

## 📐 Kinematic & Physical Constraints

The system is configured with safety limits based on the physical robot parameters:

| Joint | Position Limit (Min/Max) | Velocity Limit |
| :--- | :--- | :--- |
| **Chassis Wheels** | Continuous | $12.3 \text{ rad/s}$ |
| **Arm J1 & J5** | $\pm 3\pi \text{ rad}$ | $1.0 \text{ rad/s}$ |
| **Arm J2** | $\pm 2\pi/3 \text{ rad}$ | $1.0 \text{ rad/s}$ |
| **Arm J3 & J4** | $\pm \pi/3 \text{ rad}$ | $1.0 \text{ rad/s}$ |

**Soft Limits**: The kinematics engine includes a **$0.08 \text{ rad}$ buffer** that forces the robot to slow down automatically as it approaches a physical hard stop.

---

## 🚦 Quick Start

### 1. Configure the Mission
```cpp
RobotRunner runner("mission_results.csv");
runner.configureRobotTiming(0.01, 10, 0.1); // dt=10ms, k=10, saving_dt=100ms
```

### 2. Define a Pick-and-Place Segment
```cpp
runner.addSegment(
    "PickUp", 
    T_se_start, 
    T_se_goal, 
    5.0, // Duration in seconds
    TrajectoryType::ScrewTrajectory, 
    0 // Gripper State: Open
);
```

### 3. Run the Mission
```cpp
runner.runMission(); // Executes trajectory generation -> control -> integration
```

---

## 💻 Technical Requirements
* **Compiler**: C++17 compliant.
* **Linear Algebra**: Eigen 3.x.
* **Platform**: Optimized for high-performance Linux environments (Tested on Acer Nitro i9-13900H / RTX 5060).

*This framework is the result of integrated research in Mechanical Engineering, Earth Sciences, and Autonomous Navigation.*