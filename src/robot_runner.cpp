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
    controller_ = std::make_shared<RobotControl>(0.01); // Default dt, can be updated later
    logger_ = std::make_shared<CSVLogger>(log_path, true); // Write headers by default
}

void RobotLogic::RobotRunner::configureRobotTiming(const double dt, const int k, const double saving_dt) 
{
    dt_ = dt;
    k_ = k;
    saving_dt_ = saving_dt;
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
    const double max_wheels_vel, 
    const double max_joints_vel, 
    const std::vector<int>& VbMappingIndices,
    const Eigen::MatrixXd& F, 
    const Eigen::MatrixXd& H_0, 
    const Eigen::MatrixXd& T_sb_init, 
    const Eigen::MatrixXd& T_b0, 
    const Eigen::MatrixXd& M0_e, 
    const Eigen::MatrixXd& B_list) 
{
    if (kinematics_) {
        kinematics_->setMaxWheelsVelocity(max_wheels_vel);
        kinematics_->setMaxJointsVelocity(max_joints_vel);
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

void RobotLogic::RobotRunner::addSegment(
    const std::string& name, 
    const Eigen::Matrix4d& T_start, 
    const Eigen::Matrix4d& T_end, 
    double duration, 
    TrajectoryType type, 
    const int gripper_state) 
{
    segments_.emplace_back(std::make_unique<TrajectoryGen>(name, T_start, T_end));
    segments_.back()->setTrjTime(duration);
    segments_.back()->setTrjType(type);
    segments_.back()->setGripperState(gripper_state);
}

void RobotLogic::RobotRunner::runMission() 
{
    for (const auto& segment : segments_) {
        executeSegment(*segment);
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

void RobotLogic::RobotRunner::executeSegment(TrajectoryGen& segment) 
{
    
    std::cout << "- - - Executing segment: - - - " << segment.getSegmentName() << std::endl;

    // This will hold the configurations along the trajectory for this segment
    std::vector<Eigen::VectorXd> trajectory_q_lists; // This will hold the configurations along the trajectory
    Eigen::Matrix4d T_current;
    Eigen::VectorXd q_current;

    // Generate the trajectory based on the segment's parameters
    segment.generateTrajectory(segment.getTrjType(), segment.getTrjTime());

    // Get the gripper state for this segment (e.g., 0 open or 1 closed)
    int gripper_state = segment.getGripperState();
    
    // 1. Get the trajectory and time step
    const std::vector<Eigen::Matrix4d>& trajectory = segment.getTrajectory();
    double dt = segment.getTrjTime() / trajectory.size(); // Assuming uniform time steps

    // 2. Iterate through the trajectory (starting from index 1 to compute velocity)
    for (size_t k = 1; k < trajectory.size(); ++k) {
        std::cout << "STEP " << k << "/" << trajectory.size() << std::endl;

        const Eigen::Matrix4d& T_desired_curr = trajectory[k];
        const Eigen::Matrix4d& T_desired_prev = trajectory[k-1];

        // --- Calculate Desired Twist (Vd) ---
        // Vd = [Xd^-1 * Xd_dot]
        // In discrete time: log(T_prev.inv * T_curr) / dt
        std::cout<< "Calculating desired twist Vd..." << std::endl;
        Eigen::Vector<double, 6> V_d = mr::Se3ToVec(mr::MatrixLog6(mr::TransInv(T_desired_prev) * T_desired_curr)) / dt;

        // --- Feedback Control ---
        q_current = kinematics_->getCurrentQState();
        T_current = kinematics_->getCurrentEndEffectorPose();

        //Debugging: Print current and desired transformation matrices
        kinematics_->printTransformationMatrix(T_current, T_desired_curr);
        
        // Pass the desired twist Vd into your controller for feedforward + PI feedback
        std::cout << "Controller: Calculating Task-Space Twist (V_t)..." << q_current.transpose() << std::endl;
        Eigen::Vector<double, 6> V_t = controller_->calNextTwistInTaskSpace(T_current, T_desired_curr, V_d);
        
        // --- Compute next control input and update state ---
        std::cout << "Computing next control input..." << std::endl;
        Eigen::VectorXd u_9_controls = kinematics_->computeControlsFromEndEffectorTwist(V_t);

        std::cout << "Computing next state..." << std::endl;
        Eigen::VectorXd q_next = kinematics_->computeNextState(u_9_controls);
        
        // Update the robot's state to the next configuration (this will also update the internal 
        // state of the kinematics)
        std::cout << "Updating robot state to next configuration..." << std::endl;
        kinematics_->updateToNextState(q_next);

        trajectory_q_lists.push_back(q_next);
    }

    // Save the trajectory configurations to CSV for this segment
    std::cout << "- - - Saving segment data to CSV - - -" << std::endl;
    saveSegmentQDataToCSV(trajectory_q_lists, gripper_state);
}
