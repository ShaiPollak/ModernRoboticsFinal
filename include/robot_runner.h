#ifndef ROBOT_RUNNER_H
#define ROBOT_RUNNER_H

#include <vector>
#include <memory>
#include "trajectory_gen.h"
#include "robot_kinematics.h"
#include "robot_controller.h"
#include "robot_logger.h"

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
      * @param VbMappingIndices A vector of indices for mapping Vb to F_ext
      * @param F The F matrix representing the robot's base kinematics
      * @param H_0 The H matrix representing the robot's initial configuration
      * @param T_sb The transformation matrix from the robot frame to the space frame
      * @param T_b0 The transformation matrix from the arm base frame to the robot frame
      * @param M0_e The initial transformation matrix of the end-effector in its home position
      * @param B_list The list of screw axes for the arm joints in the end-effector frame
      * @param max_wheels_vel Maximum velocity for the wheels in radians per second
      * @param max_joints_vel Maximum velocity for the arm joints in radians per second
      * @param joint_min_pos A vector of minimum position limits for each arm joint
      * @param joint_max_pos A vector of maximum position limits for each arm joint
     */
    void configureRobotKinematics(
        const std::vector<int>& VbMappingIndices, 
        const Eigen::MatrixXd& F,
        const Eigen::MatrixXd& H_0, 
        const Eigen::MatrixXd& T_sb, 
        const Eigen::MatrixXd& T_b0, 
        const Eigen::MatrixXd& M0_e, 
        const Eigen::MatrixXd& B_list,
        const double max_wheels_vel, 
        const double max_joints_vel, 
        const std::vector<double>& joint_min_pos,
        const std::vector<double>& joint_max_pos);
    
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
    std::shared_ptr<RobotLogger::CSVLogger> logger_ = nullptr;
    std::shared_ptr<RobotLogger::GraphLogger> graph_logger_ = nullptr;

    bool is_configured_ = false;
    double dt_ = 0.01;
    int k_ = 10;
    double saving_dt_ = 0.01;
    double current_time_ = 0.0;       // Running mission clock used by GraphLogger
    double last_csv_save_time_ = -1.0; // Tracks last CSV log time to enforce saving_dt_ interval
    
    // List of generator objects, each representing a segment of the mission
    std::vector<std::unique_ptr<TrajectoryGen>> segments_;
    
    /** @brief Executes a single trajectory segment
      * This function takes a TrajectoryGen object representing a segment of the mission and executes it.
      * It generates the trajectory, computes the control inputs, updates the robot state, and optionally logs the data.
      * @param segment A reference to the TrajectoryGen object representing the segment to be executed.
      * @return A vector of Eigen::VectorXd representing the configurations of the robot along the trajectory.
     */
    void executeSegment(TrajectoryGen& segment);

    /** @brief Computes the next control input for the robot based on the current and desired end-effector configurations
      * This function calculates the desired end-effector twist (V_d) based on the difference between the current and desired configurations, and then uses the RobotControl instance to compute the feedforward and feedback control twists. The resulting control input is returned as a vector of control commands for the wheels and joints.
      * @param T_desired_prev The previous desired end-effector configuration (4x4 homogeneous transformation matrix)
      * @param T_desired_curr The current desired end-effector configuration (4x4 homogeneous transformation matrix)
      * @return A vector of control commands (9 elements: w1..w4 for wheels, j1..j5 for joints) to be applied to the robot.
     */
    Eigen::VectorXd computeNextControl(
        const Eigen::Matrix4d& T_desired_prev,
        const Eigen::Matrix4d& T_desired_curr);

    /** @brief Computes the next state of the robot based on the current state and the control inputs
      * This function takes the control commands computed by computeNextControl and applies them to the robot's kinematics to calculate the next configuration (q_next) of the robot. It also checks for singularities and updates the internal state of the robot accordingly.
      * @param u_9_controls A vector of control commands (9 elements: w1..w4 for wheels, j1..j5 for joints) to be applied to the robot.
      * @return The next configuration of the robot as an Eigen::VectorXd.
     * 
     */
    Eigen::VectorXd computeNextState(const Eigen::VectorXd& u_9_controls);

    /** @brief Executes the control input to move the robot to the next state
      * This function takes the control commands and applies them to the robot's kinematics to physically move the robot. It should be called after computing the next state to ensure that the robot's configuration is updated in the real world (or simulation).
      * @param u_9_controls A vector of control commands (9 elements: w1..w4 for wheels, j1..j5 for joints) to be applied to the robot.
     */
    void executeControl(const Eigen::VectorXd& u_9_controls);

    /** @brief Checks if the robot is near a singularity and alerts if necessary
      * This function uses the RobotKinematics instance to check if the current configuration of the robot is close to a singularity. If it detects that the robot is near a singularity, it can log a warning message or take appropriate action to avoid potential issues during control execution.
     */
    void checkForSingularity();

    /** @brief Updates the robot's internal state to the next configuration
      * This function takes the next configuration (q_next) computed by computeNextState and updates the robot's internal state representation. This is important for ensuring that subsequent control computations are based on the most current state of the robot.
      * @param q_next The next configuration of the robot as an Eigen::VectorXd, which should be used to update the internal state of the robot.
     */
    void updateToNextState(const Eigen::VectorXd& q_next);

    /** @brief Checks if the robot is properly configured before running the mission
      * This function verifies that all necessary components and parameters are correctly set up.
      * @return True if the robot is properly configured, false otherwise.
     */
    bool checkConfiguration() const; // Helper function to check if the robot is properly configured before running the mission

    /** @brief Logs the current data for analysis and graphing
      * This function takes the current time, segment name, robot configuration, desired and actual end-effector poses, task-space error, control commands, and gripper state, and logs this data using the appropriate logger instances.
      * @param current_time_ The current time in seconds during mission execution.
      * @param seg_name The name of the active trajectory segment being executed.
      * @param q_current The current configuration of the robot as an Eigen::VectorXd.
      * @param T_desired The desired end-effector pose (4x4 homogeneous transformation matrix) at this time step.
      * @param T_actual The actual end-effector pose (4x4 homogeneous transformation matrix) at this time step.
      * @param X_err The task-space error twist (6x1 vector) representing the difference between the desired and actual end-effector configurations.
      * @param controls The control commands (9-element vector) being applied to the robot at this time step.
      * @param gripper_state An integer representing the state of the gripper (e.g., 0 for open, 1 for closed) to be logged alongside the configuration data.
     */
    void logData(
        const double current_time_,
        const std::string& seg_name,
        const Eigen::VectorXd& q_current, 
        const Eigen::Matrix4d& T_desired, 
        const Eigen::Matrix4d& T_actual, 
        const Eigen::Vector<double, 6>& X_err, 
        const Eigen::VectorXd& controls,
        const int gripper_state);

    /** @brief Logs the current configuration (q) and gripper state to a CSV file using the CSVLogger instance.
      * This function is called at regular intervals during mission execution to save the robot's state for later analysis. It takes the current configuration vector and the gripper state, and uses the CSVLogger to write this data to a CSV file.
      * @param q_current The current configuration of the robot as an Eigen::VectorXd, which should be logged to the CSV file.
      * @param gripper_state An integer representing the state of the gripper (e.g., 0 for open, 1 for closed) to be logged alongside the configuration data.
     */
    void saveQStateToCSV(const Eigen::VectorXd& q_current, const int gripper_state);

    /** @brief Logs the error and control data for graphing and analysis
      * This function takes the current time, segment name, robot configuration, error twist, control commands, and desired/actual end-effector poses, and logs this data using the GraphLogger instance for later analysis and plotting.
      * @param t The current time in seconds during mission execution.
      * @param seg_name The name of the active trajectory segment being executed.
      * @param q The current configuration of the robot as an Eigen::VectorXd.
      * @param X_err The task-space error twist (6x1 vector) representing the difference between the desired and actual end-effector configurations.
      * @param controls The control commands (9-element vector) being applied to the robot at this time step.
      * @param T_desired The desired end-effector pose (4x4 homogeneous transformation matrix) at this time step.
      * @param T_actual The actual end-effector pose (4x4 homogeneous transformation matrix) at this time step.
     */    
    void graphErrorAndControlData(
        double t,
        const std::string& seg_name,
        const Eigen::VectorXd& q,
        const Eigen::Vector<double, 6>& X_err,
        const Eigen::VectorXd& controls,
        const Eigen::Matrix4d& T_desired,
        const Eigen::Matrix4d& T_actual);
    };
} // namespace RobotLogic

#endif