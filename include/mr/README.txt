# Modern Robotics C++ Library (MR-Lib)

A lightweight, header-only C++ library for robotics kinematics and dynamics, based on the textbook by Kevin Lynch and Frank Park.

## 📁 Library Structure & Functions

### 1. `core.h` - Lie Groups & Mathematical Foundations
Contains essential operations for $SO(3)$ and $SE(3)$.

* **Conversions:** `VecToso3`, `so3ToVec`, `VecTose3`, `se3ToVec`.
* **Transformations:** `RpToTrans`, `TransToRp`, `TransInv`, `Adjoint`.
* **Exponentials & Logarithms:** * `MatrixExp3`, `MatrixLog3` (Rotations).
    * `MatrixExp6`, `MatrixLog6` (Homogeneous Transformations).
* **Screw Theory:** `ScrewToAxis`, `AxisAng3`, `AxisAng6`.

### 2. `kinematics.h` - Forward & Inverse Kinematics
Logic for mapping joint spaces to task spaces.

* **Forward Kinematics:** `FKinBody`, `FKinSpace`.
* **Jacobians:** `JacobianBody`, `JacobianSpace`.
* **Inverse Kinematics:** * `IKinBody` (Numerical Newton-Raphson).
    * `IKinSpace` (Numerical Newton-Raphson).
* **Analysis:** `TestIfObstacle` (if applicable), `EndEffectorForces`.

### 3. `trajectory.h` - Motion Planning
Generating smooth transitions between configurations.

* **Time Scaling:** `CubicTimeScaling`, `QuinticTimeScaling`.
* **Trajectories:** * `JointTrajectory` (Space of joint angles).
    * `ScrewTrajectory` (Interpolation along a screw axis).
    * `CartesianTrajectory` (Decoupled translation and rotation).

### 4. 'dynamics.h' - Robot dynamics

---

## 🛠 Usage

1. **Prerequisites:** Install [Eigen 3.3+](https://eig**Time Scaling:** `CubicTimeScaling`, `QuinticTimeScaling`.
* **Trajectories:** * `JointTrajectory` (Space of joint angles).
    * `ScrewTrajectory` (Interpolation along a screw axis).
    * `CartesianTrajectory` (Decoupled translation and rotation).en.tuxfamily.org/).
2. **Integration:** Since this is header-only, just include the relevant file:

```cpp
#include "mr/core.h"
#include "mr/trajectory.h"

// Your robotic logic here...