#ifndef ROBOT_LOGGER_H
#define ROBOT_LOGGER_H

#include <fstream>
#include <string>
#include <vector>
#include <utility>
#include <Eigen/Dense>

namespace RobotLogger {

class CSVLogger {
public:
    CSVLogger(const std::string& filepath, bool write_headers = false);
    ~CSVLogger();

    void logState(const Eigen::VectorXd& state, int gripper_state);
    void logError(const std::string& error_message);
    void writeConfigurationToCSVFile(const Eigen::VectorXd& q, int gripper_state);

private:
    std::ofstream file_;
};

class GraphLogger {
public:
    explicit GraphLogger(const std::string& output_dir);

    /**
     * @brief Record one control step for later plotting.
     * @param t         Absolute mission time (s)
     * @param seg_name  Name of the active trajectory segment
     * @param q         12-element robot state [phi, x, y, j1..j5, w1..w4]
     * @param X_err     6-element task-space error twist [wx,wy,wz, vx,vy,vz]
     * @param controls  9-element control commands [w1..w4, j1..j5]
     * @param T_desired Desired end-effector pose (4x4)
     * @param T_actual  Actual end-effector pose  (4x4)
     */
    void logStep(
        double t,
        const std::string& seg_name,
        const Eigen::VectorXd& q,
        const Eigen::Vector<double, 6>& X_err,
        const Eigen::VectorXd& controls,
        const Eigen::Matrix4d& T_desired,
        const Eigen::Matrix4d& T_actual);

    /** @brief Write generate_plots.py to output_dir and execute it with python3. */
    void generatePlots() const;

private:
    std::string output_dir_;
    std::string current_segment_;

    std::vector<double>                       times_;
    std::vector<Eigen::VectorXd>              q_history_;
    std::vector<Eigen::Vector<double, 6>>     error_history_;
    std::vector<Eigen::VectorXd>              controls_history_;
    std::vector<Eigen::Vector3d>              ee_actual_positions_;
    std::vector<Eigen::Vector3d>              ee_desired_positions_;
    std::vector<std::pair<double, std::string>> segment_starts_;

    std::string buildPythonScript() const;

    static std::string serialize(const std::vector<double>& v);
    static std::string extractChannel(const std::vector<Eigen::VectorXd>& h, int idx);
    static std::string extractErrChannel(const std::vector<Eigen::Vector<double, 6>>& h, int idx);
    static std::string extractVec3Channel(const std::vector<Eigen::Vector3d>& h, int idx);
};

} // namespace RobotLogger

#endif
