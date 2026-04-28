#include "robot_runner.h"
#include "trajectory_gen.h"
#include "robot_kinematics.h"
#include "robot_controller.h"
#include "mr/core.h"
#include <vector>
#include <iostream>


RobotLogic::RobotRunner::RobotRunner(const std::string& log_path)
{
    kinematics_ = std::make_shared<RobotKinematics>();
    controller_ = std::make_shared<RobotControl>(0.01);
    logger_ = std::make_shared<RobotLogger::CSVLogger>(log_path, true);

    // Derive results directory from log_path for graph output
    size_t sep = log_path.find_last_of("/\\");
    std::string results_dir = (sep != std::string::npos) ? log_path.substr(0, sep) : ".";
    graph_logger_ = std::make_shared<RobotLogger::GraphLogger>(results_dir);
}

// ---------------------------------------------------------------------------------------------
// --- Configuration Logic ---

void RobotLogic::RobotRunner::configureRobotTiming(const double dt, const int k, const double saving_dt) 
{
    dt_ = dt;
    k_ = k;
    saving_dt_ = saving_dt; //defaults to 0.1s (100ms) if not provided
    if (kinematics_) {
        kinematics_->setDt(dt_);
    } else {
        std::cerr << "Error: RobotKinematics instance is not initialized." << std::endl;
    }
    if (controller_) {
        controller_->setDt(dt_);
    } else {
        std::cerr << "Error: RobotControl instance is not initialized." << std::endl;
    }
}

void RobotLogic::RobotRunner::configureRobotKinematics(
    const std::vector<int>& VbMappingIndices,
    const Eigen::MatrixXd& F, 
    const Eigen::MatrixXd& H_0, 
    const Eigen::MatrixXd& T_sb_init, 
    const Eigen::MatrixXd& T_b0, 
    const Eigen::MatrixXd& M0_e, 
    const Eigen::MatrixXd& B_list,
    const double max_wheels_vel, 
    const double max_joints_vel,
    const std::vector<double>& joint_min_pos,
    const std::vector<double>& joint_max_pos) 
{
    if (kinematics_) {
        kinematics_->updateQState(Eigen::VectorXd::Zero(12)); // Assuming 12 DOF: 3 chassis, 5 arm joints, 4 wheels
        kinematics_->setDt(dt_);
        kinematics_->setF(F);
        kinematics_->setFExtWithVbMappingIndices(VbMappingIndices);
        kinematics_->setH0(H_0);
        kinematics_->setTsb_init(T_sb_init);
        kinematics_->setTb0(T_b0);
        kinematics_->setM0e(M0_e);
        kinematics_->setBList(B_list);
        kinematics_->updateEndEffectorConfiguration(Eigen::VectorXd::Zero(12)); // Assuming 12 DOF: 3 chassis, 5 arm joints, 4 wheels
        kinematics_->updateJacobian(Eigen::VectorXd::Zero(12)); // Assuming 12 DOF: 3 chassis, 5 arm joints, 4 wheels
        kinematics_->setMaxWheelsVelocity(max_wheels_vel, -1);
        kinematics_->setMaxJointsVelocity(max_joints_vel, -1);
        kinematics_->setJointPositionLimits(joint_min_pos, joint_max_pos);
    }
    else {
        std::cerr << "Error: RobotKinematics instance is not initialized." << std::endl;
    }
}

void RobotLogic::RobotRunner::configureRobotControl(
    const Eigen::Matrix<double, 6, 6>& Kp, 
    const Eigen::Matrix<double, 6, 6>& Ki, 
    const Eigen::Matrix<double, 6, 6>& Kd) 
{
    if (controller_) {
        controller_->setGains(Kp, Ki, Kd);
        controller_->setDt(dt_);
    } else {
        std::cerr << "Error: RobotControl instance is not initialized." << std::endl;
    }
}   

// ---------------------------------------------------------------------------------------------
// --- Data Logging and Graphing Logic ---


void RobotLogic::RobotRunner::saveQStateToCSV(const Eigen::VectorXd& q_state, const int gripper_state) 
{
    if (logger_) {
        logger_->writeConfigurationToCSVFile(q_state, gripper_state);
    }
}

void RobotLogic::RobotRunner::saveSegmentQDataToCSV(
    const std::vector<Eigen::VectorXd>& q_data, 
    const int gripper_state) 
{
    if (logger_) {
        for (const Eigen::VectorXd& q : q_data) {
            logger_->writeConfigurationToCSVFile(q, gripper_state);
        }
    }
}

void RobotLogic::RobotRunner::graphErrorAndControlData(
    double t, 
    const std::string& seg_name, 
    const Eigen::VectorXd& q, 
    const Eigen::Vector<double, 6>& X_err, 
    const Eigen::VectorXd& controls, 
    const Eigen::Matrix4d& T_desired, 
    const Eigen::Matrix4d& T_actual) 
{
    if (graph_logger_) { // Log data at intervals of saving_dt_
        graph_logger_->logStep(t, seg_name, q, X_err, controls, T_desired, T_actual);
    }
}

void RobotLogic::RobotRunner::logData(
    const double current_time_,
    const std::string& seg_name,
    const Eigen::VectorXd& q_current, 
    const Eigen::Matrix4d& T_desired, 
    const Eigen::Matrix4d& T_actual, 
    const Eigen::Vector<double, 6>& X_err, 
    const Eigen::VectorXd& controls,
    const int gripper_state) 
{
    // Log data at intervals of saving_dt_ to balance detail with performance

    if (last_csv_save_time_ < 0.0 || (current_time_ - last_csv_save_time_) >= saving_dt_ - dt_ * 0.5) {
        last_csv_save_time_ = current_time_;
        saveQStateToCSV(q_current, gripper_state);
        graphErrorAndControlData(current_time_, seg_name, q_current, X_err, controls, T_desired, T_actual);
    }
}

// ---------------------------------------------------------------------------------------------
// --- Control and State Update Logic ---

Eigen::VectorXd RobotLogic::RobotRunner::computeNextControl(
    const Eigen::Matrix4d& T_desired_prev,
    const Eigen::Matrix4d& T_desired_curr) 
{
    // --- Calculate Desired Twist (Vd) ---
    // Vd = [Xd^-1 * Xd_dot]
    // In discrete time: log(T_prev.inv * T_curr) / dt
    // std::cout<< "Calculating desired twist Vd..." << std::endl;
    Eigen::Vector<double, 6> V_d = mr::Se3ToVec(mr::MatrixLog6(mr::TransInv(T_desired_prev) * T_desired_curr)) / dt_;

    // --- Feedback Control ---
    Eigen::VectorXd q_current = kinematics_->getCurrentQState();
    Eigen::Matrix4d T_current = kinematics_->getCurrentEndEffectorPose();
    
    // Pass the desired twist Vd into your controller for feedforward + PI feedback
    // std::cout << "Controller: Calculating Task-Space Twist (V_t)..." << q_current.transpose() << std::endl;
    Eigen::Vector<double, 6> V_t = controller_->calNextTwistInTaskSpace(T_current, T_desired_curr, V_d);
    
    // --- Compute next control input and update state ---
    // std::cout << "Computing next control input..." << std::endl;
    Eigen::VectorXd u_9_controls = kinematics_->computeControlsFromEndEffectorTwist(V_t);

    return u_9_controls;
}

Eigen::VectorXd RobotLogic::RobotRunner::computeNextState(const Eigen::VectorXd& u_9_controls) 
{
    // std::cout << "Computing next state from control input..." << std::endl;
    return kinematics_->computeNextState(u_9_controls);
}


void RobotLogic::RobotRunner::executeControl(const Eigen::VectorXd& u_9_controls) 
{

}

void RobotLogic::RobotRunner::checkForSingularity() 
{
    // Check if the robot is near a singularity and alert if necessary
    kinematics_->alertIfNearSingularity();
}

void RobotLogic::RobotRunner::updateToNextState(const Eigen::VectorXd& q_next) 
{
    // Update the robot's internal state to the next configuration
    kinematics_->updateToNextState(q_next);
    current_time_ += dt_;
}

// ---------------------------------------------------------------------------------------------
// --- Mission Execution Logic ---

void RobotLogic::RobotRunner::addSegment(
    const std::string& name, 
    const Eigen::Matrix4d& T_start, 
    const Eigen::Matrix4d& T_end, 
    double duration, 
    TrajectoryType type, 
    const int gripper_state) 
{
    segments_.emplace_back(std::make_unique<TrajectoryGen>(name, T_start, T_end));
    segments_.back()->setDt(dt_);
    segments_.back()->setTrjTime(duration);
    segments_.back()->setTrjType(type);
    segments_.back()->setGripperState(gripper_state);
}

void RobotLogic::RobotRunner::runMission()
{
    current_time_ = 0.0;
    for (const auto& segment : segments_) {
        executeSegment(*segment);
    }
    if (graph_logger_) {
        std::cout << "--- Generating performance graphs ---" << std::endl;
        graph_logger_->generatePlots();
    }
}

void RobotLogic::RobotRunner::executeSegment(TrajectoryGen& segment) 
{
    
    std::cout << "- - - Executing segment: - - - " << segment.getSegmentName() << std::endl;
    
    // Generate the trajectory based on the segment's parameters
    segment.generateTrajectory(segment.getTrjType(), segment.getTrjTime());

    // Get the gripper state for this segment (e.g., 0 open or 1 closed)
    int gripper_state = segment.getGripperState();
    
    // Get the generated trajectory (a vector of SE(3) transformation matrices)
    const std::vector<Eigen::Matrix4d>& trajectory = segment.getTrajectory();

    // Iterate through the trajectory points and execute control for each step
    for (size_t i = 1; i < trajectory.size(); ++i) {       
        
        if (i % 100 == 0 || i == trajectory.size() - 1) { // Log progress every 100 steps and at the end
            std::cout << "Step " << i << "/" << trajectory.size() - 1 << " at time " << current_time_ << "s" << std::endl;
        }
        
        // 1. Check for singularities before computing control
        checkForSingularity();

        // 2. Compute the control input for the current step
        Eigen::VectorXd u_9_controls = computeNextControl(trajectory[i-1], trajectory[i]);
       
        // 3. Compute the next state based on the control input
        Eigen::VectorXd q_next = computeNextState(u_9_controls);
        
        // 4. Execute the control input to move the robot to the next state
        executeControl(u_9_controls);

        // 5. Update the robot's internal state to the next configuration
        updateToNextState(q_next);

        // 6. Log data for graphing (error, control inputs, etc.)
        logData(current_time_, segment.getSegmentName(), q_next, trajectory[i], 
            kinematics_->getCurrentEndEffectorPose(), controller_->getCurrentXErr(), 
            u_9_controls, gripper_state);
    }

}
