#ifndef CSV_LOGGER_H
#define CSV_LOGGER_H

#include <fstream>
#include <string>
#include <Eigen/Dense>

class CSVLogger {
public:

    CSVLogger(const std::string& filename, bool write_headers = false);
    ~CSVLogger();

    void logState(const Eigen::VectorXd& state, const int gripper_state);
    void logError(const std::string& error_message);
    void writeConfigurationToCSVFile(const Eigen::VectorXd& q, const int gripper_state);
    void printConfiguration(const Eigen::VectorXd& q, const bool gripper_open);

private:
    std::ofstream file_;
};

#endif