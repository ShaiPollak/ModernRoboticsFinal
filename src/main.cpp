#include <iostream>
#include "robot_runner.h"
#include "robot_params.h"
#include <Eigen/Dense>

int main() {
    std::cout << "Starting Robot Mission..." << std::endl;
    
    std::string log_path = "/home/shaypk0/dev/ModernRobotics/results/robot_log.csv";
    RobotLogic::RobotRunner runner(log_path);

    // Configure robot timing parameters
    double dt = 0.01; // Control loop time step in seconds
    int k = 10; // Steps per dt for trajectory generation
    double saving_dt = 0.1; // Time interval for saving data to CSV in seconds
    runner.configureRobotTiming(dt, k, saving_dt);

    // Configure robot kinematics parameters
    double max_wheels_vel = 10.0; // Max wheel velocity in radians per second
    double max_joints_vel = 1.0; // Max joint velocity in radians per second
    runner.configureRobotKinematics(
        max_wheels_vel, 
        max_joints_vel, 
         YouBot::Frame::VbMappingIndices(), // F_ext with wheel control mapping
         YouBot::Frame::F(), 
         YouBot::Frame::H_0(), 
         YouBot::Frame::T_sb(0, 0 ,0), // Assuming starting at origin with no rotation
         YouBot::Frame::T_b0(), 
         YouBot::Arm::M_0e(), 
         YouBot::Arm::Blist());

    // Configure robot control parameters
    Eigen::Matrix<double, 6, 6> Kp = Eigen::Matrix<double, 6, 6>::Identity() * 100; // Proportional gain
    Eigen::Matrix<double, 6, 6> Ki = Eigen::Matrix<double, 6, 6>::Identity() * 0.1; // Integral gain
    Eigen::Matrix<double, 6, 6> Kd = Eigen::Matrix<double, 6, 6>::Zero(); // Derivative gain (0 for pure PI control)
    runner.configureRobotControl(Kp, Ki, Kd);

    std::cout << "Robot Mission Configuration Complete. Setting up mission segments..." << std::endl;

    std::cout << "Segment 1: Home to Standoff" << std::endl;
    runner.addSegment(
        "Home to Standoff",
        YouBot::Task::T_se_start(0, 0, 0), // Starting pose at home position
        YouBot::Task::T_se_StandoffPickUp(), // Desired end-effector pose at standoff position
        10.0, // Duration of the segment in seconds
        RobotLogic::TrajectoryType::ScrewTrajectory, // Trajectory type
        0 // Gripper state (0 for open)
    );

    std::cout << "Segment 2: Standoff to Grasp" << std::endl;
    runner.addSegment(
        "Standoff to Grasp",
        YouBot::Task::T_se_StandoffPickUp(), // Starting pose at standoff position
        YouBot::Task::T_se_GraspPickUp(), // Desired end-effector pose at grasp position
        10.0, // Duration of the segment in seconds
        RobotLogic::TrajectoryType::CartesianTrajectory, // Trajectory type
        0 // Gripper state (0 for open)
    );

    std::cout << "Segment 3: Close Gripper" << std::endl;
    runner.addSegment(
        "Close Gripper",
        YouBot::Task::T_se_GraspPickUp(), // Starting pose at grasp position
        YouBot::Task::T_se_GraspPickUp(), // End pose is the same since we're just closing the gripper
        10.0, // Duration of the segment in seconds
        RobotLogic::TrajectoryType::JointTrajectory, // Trajectory type (no movement, just gripper action)
        1 // Gripper state (1 for closed)
    );

    std::cout << "Segment 4: Grasp to Standoff (Cube Pick Up)" << std::endl;
    runner.addSegment(
        "Grasp to Standoff",
        YouBot::Task::T_se_GraspPickUp(), // Starting pose at grasp position
        YouBot::Task::T_se_StandoffPickUp(), // Desired end-effector pose at standoff position
        10.0, // Duration of the segment in seconds
        RobotLogic::TrajectoryType::CartesianTrajectory, // Trajectory type
        1 // Gripper state (1 for closed)
    );

    std::cout << "Segment 5: Standoff to Layoff (Driving to Layoff Location)" << std::endl;
    runner.addSegment(
        "Standoff to Layoff Locations",
        YouBot::Task::T_se_StandoffPickUp(), // Starting pose at standoff position
        YouBot::Task::T_se_StandoffLayoff(), // Desired end-effector pose at standoff position above layoff location
        10.0, // Duration of the segment in seconds
        RobotLogic::TrajectoryType::ScrewTrajectory, // Trajectory type
        1 // Gripper state (1 for closed)
    );

    std::cout << "Segment 6: Lower the cube" << std::endl;
    runner.addSegment(
        "Lower the cube",
        YouBot::Task::T_se_StandoffLayoff(), // Starting pose at standoff position above layoff location
        YouBot::Task::T_se_GraspLayoff(), // Desired end-effector pose at layoff position (lowered to place the cube)
        10.0, // Duration of the segment in seconds
        RobotLogic::TrajectoryType::ScrewTrajectory, // Trajectory type
        1 // Gripper state (1 for closed)
    );
    
    std::cout << "Segment 7: Open Gripper" << std::endl;
    runner.addSegment(
        "Open Gripper",
        YouBot::Task::T_se_GraspLayoff(), // Starting pose at layoff position
        YouBot::Task::T_se_GraspLayoff(), // End pose is the same since we're just opening the gripper
        10.0, // Duration of the segment in seconds
        RobotLogic::TrajectoryType::JointTrajectory, // Trajectory type (no movement, just gripper action)
        0 // Gripper state (0 for open)
    );

    std::cout << "Segment 8: Retreat to Standoff" << std::endl;
    runner.addSegment(
        "Retreat to Standoff",
        YouBot::Task::T_se_GraspLayoff(), // Starting pose at layoff position
        YouBot::Task::T_se_StandoffLayoff(), // Desired end-effector pose at standoff position above layoff location
        10.0, // Duration of the segment in seconds
        RobotLogic::TrajectoryType::CartesianTrajectory, // Trajectory type
        0 // Gripper state (0 for open)
    );
    
    std::cout << "All mission segments added. Starting execution..." << std::endl;
    runner.runMission();


    std::cout << "Robot Mission Completed." << std::endl;
    return 0;
}