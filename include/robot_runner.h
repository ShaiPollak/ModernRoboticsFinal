#ifndef ROBOT_RUNNER_H
#define ROBOT_RUNNER_H

#include <vector>
#include <memory>
#include "trajectory_gen.h"
#include "robot_kinematics.h"
#include "robot_controller.h"
#include "csv_logger.h"

namespace RobotLogic {

class RobotRunner {
public:
    /** @brief Constructor for the RobotRunner class.
     * Initializes the RobotRunner with shared pointers to the RobotKinematics and RobotControl instances, as well as an optional log filename for the CSV logger. The constructor sets up the internal state of the RobotRunner and prepares it for executing the mission segments.
     * @param kinematics A shared pointer to an instance of the RobotKinematics class, which handles the robot's kinematic calculations and state updates.
     * @param controller A shared pointer to an instance of the RobotControl class, which manages the control algorithms for generating control inputs based on desired trajectories.
     * @param log_filename An optional string parameter specifying the filename for logging mission data to a CSV file. If not provided, a default filename "robot_mission_log.csv" will be used.
     */
    RobotRunner(const std::string& log_filename = "robot_mission_log.csv");

    /** @brief Configures the robot's sample time and trajectory generation parameters
     * This function sets the time step for the control loop, the number of steps per time step 
     * for trajectory generation, and the interval for saving data to CSV. 
     * It should be called before configuring the robot's kinematics and control parameters to 
     * ensure that all components are properly synchronized for mission execution.
     * @param dt Time step for control loop in seconds (e.g., 0.01 for 10ms)
     * @param k Number of steps per dt (e.g., 10)
     * @param saving_dt Time interval for saving data to CSV in seconds (e.g., 0.1 for 100ms)
     */
    void configureRobotTiming(const double dt, const int k, const double saving_dt);

     /** @brief Configures the robot's kinematics parameters
      * This function sets the kinematic parameters for the robot, including the maximum velocities 
      * for the wheels and arm joints, as well as the transformation matrices for the robot's base 
      * and end-effector.
      * @param max_wheels_vel Maximum velocity for the wheels in radians per second
      * @param max_joints_vel Maximum velocity for the arm joints in radians per second
      * @param VbMappingIndices A vector of indices for mapping Vb to F_ext
      * @param F The F matrix representing the robot's base kinematics
      * @param H_0 The H matrix representing the robot's initial configuration
      * @param T_sb The transformation matrix from the robot frame to the space frame
      * @param T_b0 The transformation matrix from the arm base frame to the robot frame
      * @param M0_e The initial transformation matrix of the end-effector in its home position
      * @param B_list The list of screw axes for the arm joints in the end-effector frame
     */
    void configureRobotKinematics(
        const double max_wheels_vel, 
        const double max_joints_vel, 
        const std::vector<int>& VbMappingIndices, 
        const Eigen::MatrixXd& F,
        const Eigen::MatrixXd& H_0, 
        const Eigen::MatrixXd& T_sb, 
        const Eigen::MatrixXd& T_b0, 
        const Eigen::MatrixXd& M0_e, 
        const Eigen::MatrixXd& B_list);
    
    /** @brief Configures the robot's control parameters
      * This function sets the control parameters for the robot, including the time step and the PID gain matrices. 
      * It should be called after configuring the kinematics and before running the mission to ensure that the 
      * control algorithms are properly set up for trajectory execution.
      * @param Kp The proportional gain matrix (6x6) for the PID controller
      * @param Ki The integral gain matrix (6x6) for the PID controller 
      * @param Kd The derivative gain matrix (6x6) for the PID controller (0 for pure PI control)
     */
    void configureRobotControl(
        const Eigen::Matrix<double, 6, 6>& Kp, 
        const Eigen::Matrix<double, 6, 6>& Ki, 
        const Eigen::Matrix<double, 6, 6>& Kd);

    void configureRobotDynamics(); // Placeholder for future dynamics configuration (e.g., mass, inertia)

    /** @brief Adds a segment to the overall mission */
    void addSegment(
        const std::string& name, 
        const Eigen::Matrix4d& T_start, 
        const Eigen::Matrix4d& T_end, 
        double duration, 
        TrajectoryType type, 
        const int gripper_state);

    /** @brief function to run the entire mission by executing each segment in sequence */
    void runMission();

    /** @brief Saves the configuration data (q) for a given segment to a CSV file using the CSVLogger class. 
     * This function iterates through the provided vector of configuration vectors and logs each one to the CSV file, along with the corresponding gripper state.
     * @param q_data A vector of Eigen::VectorXd, where each VectorXd represents a configuration of the robot at a specific time step during the segment execution.
     * @param gripper_state An integer representing the state of the gripper (e.g., 0 for open, 1 for closed) to be logged alongside each configuration.
     */
    void saveSegmentQDataToCSV(
        const std::vector<Eigen::VectorXd>& q_data, 
        const int gripper_state);

private:
    std::shared_ptr<RobotKinematics> kinematics_ = nullptr;
    std::shared_ptr<RobotControl> controller_ = nullptr;
    std::shared_ptr<CSVLogger> logger_ = nullptr;

    bool is_configured_ = false; // Flag to check if the robot has been configured before running the mission
    double dt_ = 0.01;           // Default time step for control loop
    int k_ = 10;                 // Default number of steps per dt for trajectory generation
    double saving_dt_ = 0.1;     // Default time interval for saving data to CSV
    
    // List of generator objects, each representing a segment of the mission
    std::vector<std::unique_ptr<TrajectoryGen>> segments_;
    
    /** @brief Executes a single trajectory segment
      * This function takes a TrajectoryGen object representing a segment of the mission and executes it.
      * It generates the trajectory, computes the control inputs, updates the robot state, and optionally logs the data.
      * @param segment A reference to the TrajectoryGen object representing the segment to be executed.
      * @return A vector of Eigen::VectorXd representing the configurations of the robot along the trajectory.
     */
    void executeSegment(TrajectoryGen& segment);

    bool checkConfiguration() const; // Helper function to check if the robot is properly configured before running the mission
};

} // namespace RobotLogic

#endif