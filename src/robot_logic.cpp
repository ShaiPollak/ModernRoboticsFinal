#include "robot_logic.h"
#include "robot_params.h"
#include <iostream>
#include <Eigen/Dense>
#include <algorithm>
#include <cassert>
#include <iomanip>


RobotLogic::RobotKinematics::RobotKinematics(const Eigen::MatrixXd &F, const Eigen::MatrixXd &H_0, 
                                            const Eigen::MatrixXd &T_sb, const Eigen::MatrixXd &T_b0, 
                                            const Eigen::MatrixXd &M0_e, const Eigen::MatrixXd &B_list)
    : F_(F), H_0_(H_0), T_sb_(T_sb), T_b0_(T_b0), M0_e_(M0_e), B_list_(B_list),
      num_of_joints_(static_cast<int>(B_list.cols())),
      num_of_controlable_wheels_(static_cast<int>(F.cols())) 
{
    // Initialize the logger and open the file
    logger = std::make_unique<CSVLogger>("robot_log.csv", true);
    
    // Initialize q_state with zeros (12 elements: 3 chassis, 5 joints, 4 wheels)
    q_state_ = Eigen::VectorXd::Zero(3 + num_of_joints_ + num_of_controlable_wheels_);

    u_limited_aux_.resize(num_of_joints_ + num_of_controlable_wheels_);
    q_next_aux_.resize(3 + num_of_joints_ + num_of_controlable_wheels_);
}
   
void RobotLogic::RobotKinematics::set_dt(double dt)
{
    dt_ = dt;
}

double RobotLogic::RobotKinematics::get_dt() const noexcept
{
    return dt_;
}

void RobotLogic::RobotKinematics::set_max_wheels_velocity(double vel)
{
    max_wheels_velocity_ = vel;
}


double RobotLogic::RobotKinematics::get_max_wheels_velocity() const noexcept
{
    return max_wheels_velocity_;
}

void RobotLogic::RobotKinematics::set_max_joints_velocity(double vel)
{
    max_joints_velocity_ = vel;
}

double RobotLogic::RobotKinematics::get_max_joints_velocity() const noexcept
{
    return max_joints_velocity_;
}

void RobotLogic::RobotKinematics::write_configuration_to_csv_file()
{
    logger->logState(this->q_state_, this->get_grip_state());
}

void RobotLogic::RobotKinematics::print_configuration(const Eigen::VectorXd& q, bool gripper_open) {
    // Set standard formatting for the output
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "\n================  ROBOT STATE  ================" << std::endl;

    // 1. Chassis State (phi, x, y)
    std::cout << " [CHASSIS] " 
              << "Phi: " << std::setw(8) << q(0) << " rad | "
              << "X: "   << std::setw(8) << q(1) << " m | "
              << "Y: "   << std::setw(8) << q(2) << " m" << std::endl;

    // 2. Arm Joints (J1 - J5)
    std::cout << " [ARM]     ";
    for (int i = 0; i < num_of_joints_; ++i) {
        std::cout << "J" << i + 1 << ": " << std::setw(7) << q(3 + i) << " ";
    }
    std::cout << std::endl;

    // 3. Wheel Angles (W1 - W4)
    std::cout << " [WHEELS]  ";
    for (int i = 0; i < num_of_controlable_wheels_; ++i) {
        std::cout << "W" << i + 1 << ": " << std::setw(7) << q(3 + num_of_joints_ + i) << " ";
    }
    std::cout << std::endl;

    // 4. Gripper Status
    std::cout << " [GRIPPER] Status: " << (gripper_open ? "OPEN" : "CLOSED") << std::endl;
    
    std::cout << "===============================================" << std::endl;
}

void RobotLogic::RobotKinematics::openGrip()
{
    grip_state_ = YouBot::Gripper::GripperState::Open;
}
                
void RobotLogic::RobotKinematics::closeGrip()
{
    grip_state_ = YouBot::Gripper::GripperState::Closed;
}

int RobotLogic::RobotKinematics::get_grip_state() const noexcept {
    // Return 1 for Closed, 0 for Open
    return (grip_state_ == YouBot::Gripper::GripperState::Closed) ? 1 : 0;
}

void RobotLogic::RobotKinematics::setInitialQState(const Eigen::VectorXd& q_start)
{
    q_state_ = q_start;
}

Eigen::VectorXd RobotLogic::RobotKinematics::computeNextState(const Eigen::VectorXd &q_current, 
                                                              const Eigen::VectorXd &u_controls) 
{
    // --- 1. Safety Checks (Assertions) ---
    // Ensure inputs match the expected robot dimensions to prevent memory crashes
    assert(q_current.size() == q_state_.size() && "Input configuration size mismatch!");
    assert(u_controls.size() == (num_of_joints_ + num_of_controlable_wheels_) && "Control vector size mismatch!");

    // --- 2. Velocity Limiting (Clamping) ---
    // We update u_limited_aux
    u_limited_aux_ = u_controls;

    // Limit Arm Joint Velocities
    for (int i = 0; i < num_of_joints_; ++i) {
        u_limited_aux_[i] = std::clamp(u_limited_aux_[i], -max_joints_velocity_, max_joints_velocity_);
    }

    // Limit Wheel Velocities
    for (int i = num_of_joints_; i < u_limited_aux_.size(); ++i) {
        u_limited_aux_[i] = std::clamp(u_limited_aux_[i], -max_wheels_velocity_, max_wheels_velocity_);
    }

    // --- 3. Joint and Wheel Integration ---
    // Pre-fill q_next_aux_ with current state
    q_next_aux_ = q_current;
    
    // Update angles: theta_new = theta_old + (velocity * dt)
    // We update joints (index 3-7) and wheels (index 8-11) in one efficient operation
    q_next_aux_.segment(3, num_of_joints_ + num_of_controlable_wheels_) += u_limited_aux_ * dt_;

    // --- 4. Chassis Odometry ---
    // Calculate Body Twist: Vb = F * wheel_speeds * dt
    // delta_Vb_aux_ is a Vector3d (fixed size), which is extremely fast in Eigen
    delta_Vb_aux_ = F_ * u_limited_aux_.tail(num_of_controlable_wheels_) * dt_;

    const double phi = q_current(0);
    const double d_phi = delta_Vb_aux_(0);
    double dx, dy;

    if (std::abs(d_phi) < 1e-6) {
        // Case A: Straight line motion (limit of the arc formula as d_phi -> 0)
        dx = delta_Vb_aux_(1) * std::cos(phi) - delta_Vb_aux_(2) * std::sin(phi);
        dy = delta_Vb_aux_(1) * std::sin(phi) + delta_Vb_aux_(2) * std::cos(phi);
    } else {
        // Case B: Exact integration of motion along a constant curvature arc
        const double phi_next = phi + d_phi;
        dx = (delta_Vb_aux_(1) * (std::sin(phi_next) - std::sin(phi)) + 
              delta_Vb_aux_(2) * (std::cos(phi_next) - std::cos(phi))) / d_phi;
        dy = (delta_Vb_aux_(1) * (-std::cos(phi_next) + std::cos(phi)) + 
              delta_Vb_aux_(2) * (std::sin(phi_next) - std::sin(phi))) / d_phi;
    }

    // Update chassis state in our temp buffer
    q_next_aux_(0) += d_phi;
    q_next_aux_(1) += dx;
    q_next_aux_(2) += dy;

    // --- 5. Finalize and Log ---
    // Update the official internal state
    q_state_ = q_next_aux_;

    // Record the step in CSV and Print to terminal
    write_configuration_to_csv_file();
    bool is_currently_open = (this->get_grip_state() == 0); 
    print_configuration(q_state_, is_currently_open);

    return q_state_;
}
