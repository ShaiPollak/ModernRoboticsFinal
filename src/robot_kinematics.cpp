#include "robot_kinematics.h"
//#include "csv_logger.h"
#include "mr/kinematics.h"
#include "mr/odometry.h"
#include "mr/core.h"
#include <iostream>
#include <Eigen/Dense>
#include <iomanip>

/** @brief Constructor for the RobotKinematics class
 * @param F Transmission matrix that maps control inputs to body twist (H(0) dagger)
 * @param VbMappingIndices A vector of indices that specify how the control inputs map to the 
 * body twist components in F_ext
 * @param H_0 Home configuration of the robot in the space frame
 * @param T_sb Transformation from the space frame to the robot base frame
 * @param T_b0 Transformation from the robot base frame to the arm base frame
 * @param M0_e Home configuration of the end-effector in the arm base frame
 * @param B_list Screw axes of the joints in the end-effector frame when at home position
 */
RobotLogic::RobotKinematics::RobotKinematics(
    const Eigen::MatrixXd &F, 
    const std::vector<int>& VbMappingIndices, 
    const Eigen::MatrixXd &H_0, 
    const Eigen::MatrixXd &T_sb_init, 
    const Eigen::MatrixXd &T_b0, 
    const Eigen::MatrixXd &M0_e, 
    const Eigen::MatrixXd &B_list):
    F_(F), 
    VbMappingIndices_(VbMappingIndices),
    H_0_(H_0), T_sb_(T_sb_init), 
    T_b0_(T_b0), M0_e_(M0_e), 
    B_list_(B_list),
    num_of_joints_(static_cast<int>(B_list.cols())),
    num_of_controlable_wheels_(static_cast<int>(F.cols())) 
{
    
    // Initialize q_state with zeros (12 elements: 3 chassis, 5 joints, 4 wheels)
    this->q_state_ = Eigen::VectorXd::Zero(3 + this->num_of_joints_ + num_of_controlable_wheels_);
    
    // Initialize F_ext as a 6xN matrix of zeros, where N is the number of controlable wheels 
    // (columns in F)
    F_ext_ = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, num_of_controlable_wheels_);

    /* Map the relevant rows of F to F_ext based on the VbMappingIndices.
    It will look like this;
    F_ext_ = [ 0 0 0 0 ]
             [ F(0,0) F(0,1) F(0,2) F(0,3) ]
             [ F(1,0) F(1,1) F(1,2) F(1,3) ]
             [ F(2,0) F(2,1) F(2,2) F(2,3) ]
             [ 0 0 0 0 ]
             [ 0 0 0 0 ] 
    */
    for (size_t i = 0; i < VbMappingIndices_.size(); ++i) {
        int row_index = VbMappingIndices_[i];
        if (row_index >= 0 && row_index < 6) {
            F_ext_.row(row_index) = F_.row(i);
        } else {
            throw std::out_of_range("Row index in VbMappingIndices is out of valid range (0-5)");
        }
    }

    // J_base is the Jacobian of the robot base in the end-effector frame, calculated as Adjoint(TransInv(T_b0 * M0_e)) * F_ext
    this->J_base_ = mr::Adjoint(mr::TransInv(T_b0_ * M0_e_)) * F_ext_;

    // J_arm is just the B_list of the robot arm expressed in the end-effector frame 
    this->J_arm_ = B_list_;

    // The overall Jacobian J_e that maps the combined joint and wheel velocities 
    // to the end-effector twist is the horizontal concatenation of J_base and J_arm
    this->J_e_ = mr::kinematics::StackJacobians(J_base_, J_arm_);
}

//--------------------------------------------Setters and Getters for Robot Parameters--------------------------------------

// Setters and getters for robot parameters
void RobotLogic::RobotKinematics::setF(const Eigen::MatrixXd& F) 
{
    this->F_ = F;
    num_of_controlable_wheels_ = static_cast<int>(F.cols());
}
Eigen::MatrixXd RobotLogic::RobotKinematics::getF() const noexcept
{
    return this->F_;
}
void RobotLogic::RobotKinematics::setFExtWithVbMappingIndices(const std::vector<int>& VbMappingIndices) 
{
    this->VbMappingIndices_ = VbMappingIndices;
    F_ext_ = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, num_of_controlable_wheels_);
    /* Map the relevant rows of F to F_ext based on the VbMappingIndices.
    It will look like this;
    F_ext_ = [ 0 0 0 0 ]
             [ F(0,0) F(0,1) F(0,2) F(0,3) ]
             [ F(1,0) F(1,1) F(1,2) F(1,3) ]
             [ F(2,0) F(2,1) F(2,2) F(2,3) ]
             [ 0 0 0 0 ]
             [ 0 0 0 0 ] 
    */
    for (size_t i = 0; i < VbMappingIndices_.size(); ++i) {
        int row_index = VbMappingIndices_[i];
        if (row_index >= 0 && row_index < 6) {
            F_ext_.row(row_index) = F_.row(i);
        } else {
            throw std::out_of_range("Row index in VbMappingIndices is out of valid range (0-5)");
        }
    }
}
std::pair<std::vector<int>, Eigen::MatrixXd> RobotLogic::RobotKinematics::getFExt() const noexcept
{
    return {this->VbMappingIndices_, this->F_ext_};
}
void RobotLogic::RobotKinematics::setH0(const Eigen::MatrixXd& H_0) 
{
    this->H_0_ = H_0;
}
Eigen::MatrixXd RobotLogic::RobotKinematics::getH0() const noexcept
{
    return this->H_0_;
}
void RobotLogic::RobotKinematics::setTsb_init(const Eigen::MatrixXd& T_sb_init) {
    this->T_sb_ = T_sb_init;
}
Eigen::MatrixXd RobotLogic::RobotKinematics::getRobotLocation_T_sb() const noexcept {
    return this->T_sb_;
}
void RobotLogic::RobotKinematics::setTb0(const Eigen::MatrixXd& T_b0) {
    this->T_b0_ = T_b0;
}
Eigen::MatrixXd RobotLogic::RobotKinematics::getTb0() const noexcept {
    return this->T_b0_;
}
void RobotLogic::RobotKinematics::setM0e(const Eigen::MatrixXd& M0_e) {
    this->M0_e_ = M0_e;
}
Eigen::MatrixXd RobotLogic::RobotKinematics::getM0e() const noexcept {
    return this->M0_e_;
}
void RobotLogic::RobotKinematics::setBList(const Eigen::MatrixXd& B_list) {
    this->B_list_ = B_list;
    this->num_of_joints_ = static_cast<int>(B_list.cols());
}
 
/** @brief Set the time step for integration
  * @param dt Time step in seconds (e.g., 0.01 for 10ms)
 */
void RobotLogic::RobotKinematics::setDt(double dt)
{
    this->dt_ = dt;
}
double RobotLogic::RobotKinematics::getDt() const noexcept
{
    return this->dt_;
}

/** @brief Set the maximum velocity for the wheels
  * @param vel Maximum wheel velocity in appropriate units (e.g., radians per second)
 */
void RobotLogic::RobotKinematics::setMaxWheelsVelocity(double vel)
{
    this->max_wheels_velocity_ = vel;
}
double RobotLogic::RobotKinematics::getMaxWheelsVelocity() const noexcept
{
    return this->max_wheels_velocity_;
}

/** @brief Set the maximum velocity for the arm joints
  * @param vel Maximum joint velocity in appropriate units (e.g., radians per second)
 */
void RobotLogic::RobotKinematics::setMaxJointsVelocity(double vel)
{
    max_joints_velocity_ = vel;
}
double RobotLogic::RobotKinematics::getMaxJointsVelocity() const noexcept
{
    return max_joints_velocity_;
}


// --------------------------------------Configuration and State Update Methods--------------------------------------

/** @brief Update the robot's location based on the current and next chassis configurations
  * @param T_sb Current transformation matrix from the robot frame to the space frame
  * @param q_current Current configuration vector (For our case: size 12: 3 chassis, 5 arm joints, 4 wheels)
  * @param q_next Next configuration vector (For our case: size 12: 3 chassis, 5 arm joints, 4 wheels)
  */
void RobotLogic::RobotKinematics::updateRobotLocation(
    const Eigen::Matrix4d& T_sb,
    const Eigen::VectorXd& q_current,
    const Eigen::VectorXd& q_next) {
    
    T_sb_ = mr::Odometry::nextOmniLocationInT(T_sb, q_current.head(3), q_next.head(3));
}

void RobotLogic::RobotKinematics::updateEndEffectorConfiguration(const Eigen::VectorXd& q)
{
    // Update the end-effector pose based on the current arm joint angles
    Eigen::VectorXd arm_thetas = q.segment(3, num_of_joints_);
    // Compute the forward kinematics (FK) for the arm to determine the end-effector pose relative to the base
    T_0e_ = mr::kinematics::FKinBody(M0_e_, B_list_, arm_thetas);

    // Update the overall end-effector pose in the space frame by combining the transformations
    T_se_ = T_sb_ * T_b0_ * T_0e_;
}

/** @brief Update the Jacobian matrices based on the current configuration of the robot
  * param q_current Current configuration vector (For our case: size 12: 3 chassis, 5 arm joints, 4 wheels)
  */
void RobotLogic::RobotKinematics::updateJacobian(const Eigen::VectorXd& q) {
    // 1. Update Arm FK first so T_be is current!
    Eigen::VectorXd arm_thetas = q.segment(3, num_of_joints_); // joints 3,4,5,6,7
    this->T_0e_ = mr::kinematics::FKinBody(M0_e_, B_list_, arm_thetas);
    
    // 2. Compute the current Base-to-End-Effector Transform
    Eigen::Matrix4d T_be = T_b0_ * T_0e_;

    // 3. Compute Arm Jacobian (6 x num_of_joints)
    this->J_arm_ = mr::kinematics::JacobianBody(B_list_, arm_thetas);

    // 4. Compute Base Jacobian
    // Ensure F_ext_ is 6 x 4 (for 4 wheels)
    if (F_ext_.rows() != 6) {
        throw std::runtime_error("F_ext_ must have 6 rows to multiply by Adjoint!");
    }
    this->J_base_ = mr::Adjoint(mr::TransInv(T_be)) * F_ext_;
    
    // 5. Stack (Result is 6 x (4 wheels + 5 joints) = 6x9)
    J_e_ = mr::kinematics::StackJacobians(J_base_, J_arm_);
    
    // DEBUG PRINT: This will tell you exactly what the dimensions are before the crash
    std::cout << "Jacobian Updated. Size: " << J_e_.rows() << "x" << J_e_.cols() << std::endl;
}

/** @brief Update the current configuration of the robot to the next state
  * @param q Next configuration vector (For our case: size 12: 3 chassis, 5 arm joints, 4 wheels)
  */
void RobotLogic::RobotKinematics::updateQState(const Eigen::VectorXd& q)
{
    q_state_ = q;
}

/** @brief Get the current configuration of the robot
  * @return Current configuration vector (For our case: size 12: 3 chassis, 5 arm joints, 4 wheels)
  */
Eigen::VectorXd RobotLogic::RobotKinematics::getCurrentQState() const noexcept
{
    return q_state_;
}

/** @brief Get the current end-effector pose in the space frame
  * @return Current end-effector transformation matrix in the space frame
  */
Eigen::Matrix4d RobotLogic::RobotKinematics::getCurrentEndEffectorPose() const noexcept
{
    return T_se_;
}

/** @brief Update the current configuration of the robot to the next state
  * @param q_next Next configuration vector (For our case: size 12: 3 chassis, 5 arm joints, 4 wheels)
  */
void RobotLogic::RobotKinematics::updateToNextState(const Eigen::VectorXd &q_next)
{
    // Validate the size of q_next
    if (q_next.size() != q_state_.size()) {
        throw std::invalid_argument("q_next size mismatch!");
    }

    try {
        // Update the robot's location based on the new chassis configuration
        updateRobotLocation(T_sb_, q_state_, q_next);

        // After updating the configuration, we need to recalculate the Jacobian matrices to reflect the new state of the robot
        updateEndEffectorConfiguration(q_next);

        // Recompute the Jacobian matrices based on the updated configuration
        updateJacobian(q_next);

        // Update the internal state to the new configuration
        updateQState(q_next);
        
        // Check for singularity with a defined threshold (default is 1e-3 in mr::kinematics)
        double condition_number; 
        if (mr::kinematics::isSingular(J_e_)) {
            //  Flag the robot as being in a singular configuration (you can define what this means for your application, e.g., setting a boolean flag or logging a warning)
            this->near_singularity_ = true; 
        } else {
            this->near_singularity_ = false;
        }

    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("Error updating to next state: ") + e.what());
    }
}

/** @brief Get the current configuration of the robot
  * @return Current configuration vector (For our case: size 12: 3 chassis, 5 arm joints, 4 wheels)
  */
Eigen::VectorXd RobotLogic::RobotKinematics::computeNextState(const Eigen::VectorXd &u_controls) 
{
    Eigen::VectorXd q_next(12); // 3 chassis + 5 arm joints + 4 wheels = 12
    int wheel_start_idx = 3 + num_of_joints_;

    // Compute the next chassis state using the odometry function
    q_next.head(3) = mr::Odometry::nextQState(q_state_.head(3), u_controls.segment(0, 4), dt_, F_);
    
    // Compute the next arm joint states using simple Euler integration
    q_next.segment(3, num_of_joints_) = mr::eulerIntegration(q_state_.segment(3, num_of_joints_), 
        u_controls.tail(num_of_joints_), dt_);
    
    // Compute the next wheel states using simple Euler integration
    q_next.segment(wheel_start_idx, num_of_controlable_wheels_) = 
        mr::eulerIntegration(q_state_.segment(wheel_start_idx, num_of_controlable_wheels_), 
        u_controls.segment(0, num_of_controlable_wheels_), dt_);

    return q_next;  
}

/** @brief Compute the control inputs required to achieve a desired end-effector twist
  * @param V_t End-effector twist in the space frame (size 6: [vx, vy, vz, wx, wy, wz])
  * @return Control input vector (size 9: 4 wheel speeds, 5 arm joints)
  */
Eigen::VectorXd RobotLogic::RobotKinematics::computeControlsFromEndEffectorTwist(
    const Eigen::Vector<double, 6>& V_t)
{
    // Safety check: The pseudoinverse of J (6xN) will be (Nx6). 
    // It must be multiplied by a 6x1 vector.
    if (J_e_.rows() != 6) {
        std::cerr << "CRITICAL: J_e_ rows (" << J_e_.rows() 
                  << ") != V_t size (6). Jacobian update failed!" << std::endl;
    }

    // Compute the 9x6 pseudoinverse
    Eigen::MatrixXd J_pinv = mr::pseudoInverse(J_e_);

    // Check inner dimension: J_pinv.cols() must equal V_t.rows() (which is 6)
    if (J_pinv.cols() != V_t.size()) {
        std::cerr << "Dimension Mismatch: J_pinv cols: " << J_pinv.cols() 
                  << ", V_t size: " << V_t.size() << std::endl;
    }

    return J_pinv * V_t; // This returns a Vector of size 9
}


// --------------------------------------Utility Methods--------------------------------------

/** @brief Print the current configuration of the robot in a human-readable format
  * @param q Current configuration vector (For our case: size 12: 3 chassis, 5 arm joints, 4 wheels)
  */
void RobotLogic::RobotKinematics::printConfiguration(const Eigen::VectorXd& q) const noexcept {
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
    
    std::cout << "===============================================" << std::endl;
}

/** @brief Print the current and desired end-effector transformation matrices
  * @param T_current Current end-effector transformation matrix (4x4)
  * @param T_desired Desired end-effector transformation matrix (4x4)
  */
void RobotLogic::RobotKinematics::printTransformationMatrix(const Eigen::MatrixXd& T_current, 
    const Eigen::MatrixXd& T_desired) const noexcept {
    std::cout << "\n[Transformation Matrix]" << std::endl;
    std::cout << "Current:\n" << T_current << std::endl;
    std::cout << "Desired:\n" << T_desired << std::endl;
}
