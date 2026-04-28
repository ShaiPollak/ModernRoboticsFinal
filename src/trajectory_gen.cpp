#include "trajectory_gen.h"
#include "mr/core.h"
#include "mr/trajectory.h"
#include <Eigen/Dense>

int RobotLogic::TrajectoryGen::k_ = 10;
double RobotLogic::TrajectoryGen::dt_ = 0.01;
double RobotLogic::TrajectoryGen::saving_dt_ = 0.1;
double RobotLogic::TrajectoryGen::control_frq_ = k_ / dt_;

/** @brief Constructor for the TrajectoryGen class.
 * * Initializes a single trajectory segment for the youBot pick-and-place mission.
 * Sets the boundary conditions (start/end poses) and defines the timing resolution
 * for the trajectory generation process.
 * * @param segment_name A descriptive name for the segment (e.g., "HomeToStandoff").
 * @param T_init The initial 4x4 transformation matrix of the end-effector {e} in {s} frame.
 * @param T_end The target 4x4 transformation matrix of the end-effector {e} in {s} frame.
 * @param Tf Total time duration for the segment execution in seconds.
 * * @note Default simulation parameters:
 * - k = 10 (Steps per dt)
 * - dt = 0.01s (Control loop cycle)
 * - gripper_state = 0 (Open)
 */
RobotLogic::TrajectoryGen::TrajectoryGen(
    const std::string& segment_name, 
    const Eigen::Matrix4d& T_init, 
    const Eigen::Matrix4d& T_end):
    segment_name_(segment_name), 
    T_init_(T_init), 
    T_end_(T_end)
{
    // Initialize static members with default values
    tf_ = 10;                    // Default, in sec
    gripper_state_ = 0;
}

// Setters and getters for class members

void RobotLogic::TrajectoryGen::setSegmentName(const std::string& name) {
    segment_name_ = name;
}
std::string RobotLogic::TrajectoryGen::getSegmentName() const noexcept {
    return segment_name_;
}
void RobotLogic::TrajectoryGen::setTrjTime(double Tf){
    if (Tf <= saving_dt_) {
        throw std::invalid_argument("Time for trajectory segment is too short.");
    }
    tf_ = Tf;
    total_steps_ = static_cast<int>((tf_ / dt_) * k_) + 1;
}
double RobotLogic::TrajectoryGen::getTrjTime() const noexcept {
    return tf_;
}
void RobotLogic::TrajectoryGen::setGripperState(int state){
    if (state != 0 && state != 1){
        throw std::invalid_argument("Gripper State is only 0 (open) or 1 (closed).");
    }
    gripper_state_ = state;
}
int RobotLogic::TrajectoryGen::getGripperState() const noexcept{
    return gripper_state_;
}
void RobotLogic::TrajectoryGen::setTrjType(TrajectoryType type){
    traj_type_ = type;
}
RobotLogic::TrajectoryType RobotLogic::TrajectoryGen::getTrjType() const noexcept{
    return traj_type_;
}
int RobotLogic::TrajectoryGen::getTrjSteps() const noexcept {
    // Calculate total steps based on tf and dt and k (k steps per dt)
    return total_steps_;
}
// Setters and getters for static members

void RobotLogic::TrajectoryGen::setK(int k) {
    k_ = k;
    control_frq_ = k_ / dt_; // Update control frequency based on new k and dt
}
int RobotLogic::TrajectoryGen::getK() noexcept {
    return k_;
}
void RobotLogic::TrajectoryGen::setSavingDt(double saving_dt) {
    if (saving_dt <= dt_) {
        throw std::invalid_argument("Saving time step must be greater than control time step.");
    }
    saving_dt_ = saving_dt;
}
double RobotLogic::TrajectoryGen::getSavingDt() noexcept {
    return saving_dt_;
}
void RobotLogic::TrajectoryGen::setDt(double dt) {
    if (dt >= saving_dt_) {
        throw std::invalid_argument("Control time step must be less than saving time step.");
    }
    dt_ = dt;
    control_frq_ = k_ / dt_; // Update control frequency based on new dt
}
double RobotLogic::TrajectoryGen::getDt() noexcept {
    return dt_;
}
void RobotLogic::TrajectoryGen::setControlFrq(double control_frq) {
    control_frq_ = control_frq;
    setK(static_cast<int>(control_frq * dt_)); // Update k based on new control frequency and dt
}
double RobotLogic::TrajectoryGen::getControlFrq() noexcept {
    return control_frq_;
}


// Set and get Trajectory

/** @brief Generates a discrete trajectory between the initial and final configurations.
 * * This function calculates the total number of steps based on the provided time (Tf) 
 * and the fixed control time-step (dt_). It then populates the internal trajectory 
 * vector using Modern Robotics algorithms.
 * * @param type The interpolation method to use:
 * - CartesianTrajectory: Decouples rotation and translation (linear path for p).
 * - ScrewTrajectory: Follows a constant screw axis (the shortest distance in SE(3)).
 * @param Tf   The total duration of the movement in seconds.
 * * @details 
 * - The number of points (N) is calculated as: floor(Tf / dt) + 1.
 * - Uses a Quintic Polynomial (5th-order) time scaling to ensure zero velocity and zero 
 * acceleration at both the start and end of the segment (smooth motion).
 */
void RobotLogic::TrajectoryGen::generateTrajectory(TrajectoryType type, double Tf){  
    setTrjTime(Tf);
    setTrjType(type);
    int N = static_cast<int>(tf_ / dt_) + 1;
    if (type == TrajectoryType::CartesianTrajectory){
        trajectory_ = mr::trajectory::CartesianTrajectory(T_init_, T_end_, tf_, N ,5);
    } 
    else {
        trajectory_ = mr::trajectory::ScrewTrajectory(T_init_, T_end_, tf_, N, 5);
    } 
}

/** @brief Returns the generated strajectory. Throws an error if empty. */
const std::vector<Eigen::Matrix4d>& RobotLogic::TrajectoryGen::getTrajectory() const {
    if (this->trajectory_.empty()) { 
        throw std::runtime_error("Trajectory is empty!");
    }
    return this->trajectory_;
}


