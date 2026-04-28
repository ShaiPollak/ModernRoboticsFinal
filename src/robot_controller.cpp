#include "robot_controller.h"
#include "mr/kinematics.h"

/** --- CONTROLLER TUNING GUIDE (Lynch & Park / Modern Robotics) ---
 * * 1. STABILITY & DAMPING:
 * To achieve Critical Damping (fastest response without oscillation):
 * Kd ≈ 2 * sqrt(Kp)
 * - If Kd < 2*sqrt(Kp): Underdamped (overshoot, oscillations)
 * - If Kd > 2*sqrt(Kp): Overdamped (slow, sluggish response)
 * * 2. TASK SPACE GAINS (Typical Starting Values for youBot):
 * Kp: Start with 1.0 - 5.0 (for all 6 DOFs)
 * Ki: Start with 0.0 or very small (0.01) to eliminate steady-state error.
 * Kd: Start with 0.0, add only if the robot is vibrating or overshooting.
 * * 3. TUNING PROCEDURE:
 * a. Set Ki and Kd to 0.
 * b. Increase Kp until the robot oscillates steadily (Critical Gain).
 * c. Set Kp to ~50% of that value.
 * d. Increase Ki slowly to eliminate the final millimeters of error.
 * e. Increase Kd if needed to dampen rapid movements.
 * * Note: High gains with a large dt (sampling time) can cause numerical instability.
 * Current dt: 0.01s (100Hz) is generally safe for Kp < 20.
 */

// --- Setters ---

// Constructor
RobotLogic::RobotControl::RobotControl(const double dt) 
    : dt_(dt) {
    // Set up 6 rows vectors (by default, it may change for other controllers)
    Kp_ = Eigen::Matrix<double, 6, 6>::Zero();
    Ki_ = Eigen::Matrix<double, 6, 6>::Zero();
    Kd_ = Eigen::Matrix<double, 6, 6>::Zero(); //For YouBot this usually stays Zero
}

// --- Setters ---

void RobotLogic::RobotControl::setDt(const double dt) {
    if (dt <= 0) {
        throw std::invalid_argument("dt must be positive");
    }
    dt_ = dt;
}
void RobotLogic::RobotControl::setKp(const Eigen::Matrix<double, 6, 6>& Kp) {
    Kp_ = Kp; 
}
void RobotLogic::RobotControl::setKi(const Eigen::Matrix<double, 6, 6>& Ki) {
    Ki_ = Ki;
}
void RobotLogic::RobotControl::setKd(const Eigen::Matrix<double, 6, 6>& Kd) {
    Kd_ = Kd;
}
void RobotLogic::RobotControl::setGains(
    const Eigen::Matrix<double, 6, 6>& Kp, 
    const Eigen::Matrix<double, 6, 6>& Ki, 
    const Eigen::Matrix<double, 6, 6>& Kd)
{     
    setKp(Kp);
    setKi(Ki);
    setKd(Kd);
}

// --- Getters ---
double RobotLogic::RobotControl::getDt() const noexcept {
    return dt_;
}
Eigen::Matrix<double, 6, 6> RobotLogic::RobotControl::getKp() const noexcept {
    return Kp_;
}
Eigen::Matrix<double, 6, 6> RobotLogic::RobotControl::getKi() const noexcept {
    return Ki_;
}
Eigen::Matrix<double, 6, 6> RobotLogic::RobotControl::getKd() const noexcept {
    return Kd_;
}

Eigen::Vector<double, 6> RobotLogic::RobotControl::feedForwardControl(
    const Eigen::Matrix4d& X, 
    const Eigen::Matrix4d& Xd, 
    const Eigen::Vector<double, 6>& V_d)
{
    
    return (mr::Adjoint(mr::TransInv(X)*Xd)*V_d);
}

Eigen::Vector<double, 6> RobotLogic::RobotControl::feedbackControl(
    const Eigen::Matrix4d& X, 
    const Eigen::Matrix4d& Xd)
{
    // X_err = [se3toVec(log(inv(X) * Xd))]
    Eigen::Matrix4d T_diff = mr::TransInv(X) * Xd;
    Eigen::Matrix4d error_matrix = mr::MatrixLog6(T_diff);
    
    // Log to extract the Error Twist
    last_X_err_ = mr::Se3ToVec(mr::MatrixLog6(T_diff));

    // Integration
    X_err_integration_ += last_X_err_ * dt_;
    

    // Kp*X_err + Ki*(integral(X_err) from 0 to t)
    return Kp_*last_X_err_ + Ki_*X_err_integration_;
}

Eigen::Vector<double, 6> RobotLogic::RobotControl::calNextTwistInTaskSpace(
    const Eigen::Matrix4d& X, 
    const Eigen::Matrix4d& Xd, 
    const Eigen::Vector<double, 6>& V_d)
{
    return feedForwardControl(X, Xd, V_d) + feedbackControl(X, Xd);
}

Eigen::Vector<double, 6> RobotLogic::RobotControl::getXErrIntegration() const noexcept 
{
    return X_err_integration_;
}

Eigen::Vector<double, 6> RobotLogic::RobotControl::getCurrentXErr() const noexcept 
{
    return last_X_err_;
}