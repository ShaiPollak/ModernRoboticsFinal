#include "csv_logger.h"
#include <iostream>
#include <iomanip>
#include <cassert>
#include <ctime>

CSVLogger::CSVLogger(const std::string& filepath, const bool write_headers) {
    
    // Save file in /results directory: /results/YYYYMMDD_HHMMSS_filename.csv

    std::time_t now = std::time(0);
    std::tm* local_time = std::localtime(&now);
    char timestamp[20];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", local_time);

    file_.open(filepath);
    if (!file_.is_open()) {
        std::cerr << "Error: Could not open file " << filepath << std::endl;
    }
    
    file_ << std::fixed << std::setprecision(6);

    if (write_headers && file_.is_open()) {
        file_ << "# chassis_phi,chassis_x,chassis_y,j1,j2,j3,j4,j5,w1,w2,w3,w4,gripper\n";
    }
}

CSVLogger::~CSVLogger() {
    if (file_.is_open()) {
        file_.close();
    }
}

void CSVLogger::logState(const Eigen::VectorXd& q, const int gripper_state) {
    if (!file_.is_open()) return;

    assert(q.size() == 12 && "Logger expected a 12-element state vector!");

    for (int i = 0; i < q.size(); ++i) {
        file_ << q[i] << ", ";
    }
    file_ << gripper_state << "\n";
}

void CSVLogger::logError(const std::string& error_message) {
    if (!file_.is_open()) return;

    file_ << "# ERROR: " << error_message << "\n";
}

// LOGGING AND PRINTING METHODS
void CSVLogger::writeConfigurationToCSVFile(const Eigen::VectorXd& q, const int gripper_state)
{
    this->logState(q, gripper_state);
}