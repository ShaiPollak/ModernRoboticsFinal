#include <iostream>
#include "robot_runner.h"
#include "robot_params.h"
#include <Eigen/Dense>

int main() {
    // Write log to CSV file in the results directory:
    std::string prints = "/home/shaypk0/dev/ModernRobotics/results/prints.csv";
    std::ofstream prints_file(prints);
    if (!prints_file.is_open()) {
        std::cerr << "Failed to open prints file: " << prints << std::endl;
        return -1;
    }

    prints_file << "Starting Robot Mission..." << std::endl;
    
    std::string log_path = "/home/shaypk0/dev/ModernRobotics/results/robot_log.csv";
    RobotLogic::RobotRunner runner(log_path);

    // Configure robot timing parameters
    double dt = 0.01; // Control loop time step in seconds
    int k = 10; // Steps per dt for trajectory generation
    double saving_dt = 0.01; // Time interval for saving data to CSV in seconds
    runner.configureRobotTiming(dt, k, saving_dt);

    // Configure robot kinematics parameters
    double max_wheels_vel = 10.0; // Max wheel velocity in radians per second
    double max_joints_vel = 1.0; // Max joint velocity in radians per second
    runner.configureRobotKinematics(
         YouBot::Frame::VbMappingIndices(), // F_ext with wheel control mapping
         YouBot::Frame::F(), 
         YouBot::Frame::H_0(), 
         YouBot::Frame::T_sb(0, 0 ,0), // Assuming starting at origin with no rotation
         YouBot::Frame::T_b0(), 
         YouBot::Arm::M_0e(), 
         YouBot::Arm::Blist(),
         YouBot::Wheel::max_velocity,
         YouBot::Arm::max_vel,
         YouBot::Arm::joint_limits.min_pos,
         YouBot::Arm::joint_limits.max_pos
    );

    // Configure robot control parameters
    Eigen::Matrix<double, 6, 6> Kp = Eigen::Matrix<double, 6, 6>::Identity() * 100; // Proportional gain
    Eigen::Matrix<double, 6, 6> Ki = Eigen::Matrix<double, 6, 6>::Identity() * 0.1; // Integral gain
    Eigen::Matrix<double, 6, 6> Kd = Eigen::Matrix<double, 6, 6>::Zero(); // Derivative gain (0 for pure PI control)
    runner.configureRobotControl(Kp, Ki, Kd);

    prints_file << "Robot Mission Configuration Complete. Setting up mission segments..." << std::endl;

    prints_file << "Segment 1: Home to Standoff" << std::endl;
    runner.addSegment(
        "Home to Standoff",
        YouBot::Task::T_se_start(0, 0, 0), // Starting pose at home position
        YouBot::Task::T_se_StandoffPickUp(), // Desired end-effector pose at standoff position
        10.0, // Duration of the segment in seconds
        RobotLogic::TrajectoryType::ScrewTrajectory, // Trajectory type
        0 // Gripper state (0 for open)
    );

    prints_file << "Segment 2: Standoff to Grasp" << std::endl;
    runner.addSegment(
        "Standoff to Grasp",
        YouBot::Task::T_se_StandoffPickUp(), // Starting pose at standoff position
        YouBot::Task::T_se_GraspPickUp(), // Desired end-effector pose at grasp position
        10.0, // Duration of the segment in seconds
        RobotLogic::TrajectoryType::ScrewTrajectory, // Trajectory type
        0 // Gripper state (0 for open)
    );

    prints_file << "Segment 3: Close Gripper" << std::endl;
    runner.addSegment(
        "Close Gripper",
        YouBot::Task::T_se_GraspPickUp(), // Starting pose at grasp position
        YouBot::Task::T_se_GraspPickUp(), // End pose is the same since we're just closing the gripper
        10.0, // Duration of the segment in seconds
        RobotLogic::TrajectoryType::JointTrajectory, // Trajectory type (no movement, just gripper action)
        1 // Gripper state (1 for closed)
    );

    prints_file << "Segment 4: Grasp to Standoff (Cube Pick Up)" << std::endl;
    runner.addSegment(
        "Grasp to Standoff",
        YouBot::Task::T_se_GraspPickUp(), // Starting pose at grasp position
        YouBot::Task::T_se_StandoffPickUp(), // Desired end-effector pose at standoff position
        10.0, // Duration of the segment in seconds
        RobotLogic::TrajectoryType::ScrewTrajectory, // Trajectory type
        1 // Gripper state (1 for closed)
    );

    prints_file << "Segment 5: Standoff to Layoff (Driving to Layoff Location)" << std::endl;
    runner.addSegment(
        "Standoff to Layoff Locations",
        YouBot::Task::T_se_StandoffPickUp(), // Starting pose at standoff position
        YouBot::Task::T_se_StandoffLayoff(), // Desired end-effector pose at standoff position above layoff location
        10.0, // Duration of the segment in seconds
        RobotLogic::TrajectoryType::ScrewTrajectory, // Trajectory type
        1 // Gripper state (1 for closed)
    );

    prints_file << "Segment 6: Lower the cube" << std::endl;
    runner.addSegment(
        "Lower the cube",
        YouBot::Task::T_se_StandoffLayoff(), // Starting pose at standoff position above layoff location
        YouBot::Task::T_se_GraspLayoff(), // Desired end-effector pose at layoff position (lowered to place the cube)
        10.0, // Duration of the segment in seconds
        RobotLogic::TrajectoryType::ScrewTrajectory, // Trajectory type
        1 // Gripper state (1 for closed)
    );
    
    prints_file << "Segment 7: Open Gripper" << std::endl;
    runner.addSegment(
        "Open Gripper",
        YouBot::Task::T_se_GraspLayoff(), // Starting pose at layoff position
        YouBot::Task::T_se_GraspLayoff(), // End pose is the same since we're just opening the gripper
        10.0, // Duration of the segment in seconds
        RobotLogic::TrajectoryType::JointTrajectory, // Trajectory type (no movement, just gripper action)
        0 // Gripper state (0 for open)
    );

    prints_file << "Segment 8: Retreat to Standoff" << std::endl;
    runner.addSegment(
        "Retreat to Standoff",
        YouBot::Task::T_se_GraspLayoff(), // Starting pose at layoff position
        YouBot::Task::T_se_StandoffLayoff(), // Desired end-effector pose at standoff position above layoff location
        10.0, // Duration of the segment in seconds
        RobotLogic::TrajectoryType::ScrewTrajectory, // Trajectory type
        0 // Gripper state (0 for open)
    );
    
    prints_file << "All mission segments added. Starting execution..." << std::endl;
    runner.runMission();


    prints_file << "Robot Mission Completed." << std::endl;
    prints_file << "Writing results to file..." << std::endl;
    prints_file.close();
    return 0;
}