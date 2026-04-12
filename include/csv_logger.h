#ifndef CSV_LOGGER_H
#define CSV_LOGGER_H

#include <fstream>
#include <string>
#include <Eigen/Dense>

class CSVLogger {
public:

    CSVLogger(const std::string& filename, bool write_headers = false);
    ~CSVLogger();

    void logState(const Eigen::VectorXd& state, int gripper_state);

private:
    std::ofstream file_;
};

#endif