#ifndef TRAJECTORY_GEN_H
#define TRAJECTORY_GEN_H

#include <Eigen/Dense>
#include <vector>

namespace RobotLogic{

enum class TrajectoryType{
ScrewTrajectory,
CartesianTrajectory,
JointTrajectory
};


class TrajectoryGen {
public:

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
    TrajectoryGen(const std::string& segment_name, const Eigen::Matrix4d& T_init, const Eigen::Matrix4d& T_end);

    void setSegmentName(const std::string& name);
    std::string getSegmentName() const noexcept;
    void setTrjTime(double Tf);
    double getTrjTime() const noexcept;
    void setGripperState(int state);
    int getGripperState() const noexcept;
    TrajectoryType getTrjType() const noexcept;
    void setTrjType(TrajectoryType type);

    static void setK(int k);
    static int getK() noexcept;
    static void setSavingDt(double saving_dt);
    static double getSavingDt() noexcept;
    static void setDt(double dt);
    static double getDt() noexcept;
    static void setControlFrq(double control_frq);
    static double getControlFrq() noexcept;

    //static void setTceGrasp(const Eigen::MatrixXd& T_ce_grasp);
    //static Eigen::MatrixXd getTceGrasp() noexcept;
    //static void setTceStandoff(const Eigen::MatrixXd& T_ce_standoff);
    //static Eigen::MatrixXd getTceStandoff() noexcept;
    
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
    void generateTrajectory(TrajectoryType traj_type, double Tf);
    
    /** @brief Returns the generated strajectory. Throws an error if empty. */
    const std::vector<Eigen::Matrix4d>& getTrajectory() const;


private:
    std::string segment_name_;              // Name of the trajectory segment (e.g., "pick", "place")
    double tf_;                             // Total time of trajectory segment
    int gripper_state_;                     // 0 is open 1 is closed
    TrajectoryType traj_type_;              // Type of trajectory interpolation (Cartesian or Screw)
    
    static int k_;                          // Number of steps to take within each time step dt
    static double saving_dt_;               // Time step for the trajectory
    static double dt_;                      // Time step for the robot control loop
    static double control_frq_;             // Control frequency (Hz) for the robot control loop

    // Trajectory of end-effector frame {e} relative to space frame {s} and gripper state (0 for open, 1 for closed)
    std::vector<Eigen::Matrix4d> trajectory_;
        
    Eigen::Matrix4d T_init_;             // Initial configuration of endeffector for this segment in {s}
    Eigen::Matrix4d T_end_;              // Final configuration of endeffector for this segment in {s}

    //static Eigen::MatrixXd T_ce_grasp_;     // Configuration of the end-effector frame {e} relative to cube frame {c} when grasping
    //static Eigen::MatrixXd T_ce_standoff_;  // Configuration of the end-effector frame {e} relative to cube frame {c} when standoff
};

}

#endif