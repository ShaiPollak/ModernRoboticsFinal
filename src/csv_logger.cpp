#include "csv_logger.h"
#include <iostream>
#include <iomanip>
#include <cassert>

CSVLogger::CSVLogger(const std::string& filename, bool write_headers) {
    file_.open(filename);
    if (!file_.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
    }
    
    file_ << std::fixed << std::setprecision(6);

    if (write_headers && file_.is_open()) {
        file_ << "chassis_phi,chassis_x,chassis_y,j1,j2,j3,j4,j5,w1,w2,w3,w4,gripper\n";
    }
}

CSVLogger::~CSVLogger() {
    if (file_.is_open()) {
        file_.close();
    }
}

void CSVLogger::logState(const Eigen::VectorXd& state, int gripper_state) {
    if (!file_.is_open()) return;

    assert(state.size() == 12 && "Logger expected a 12-element state vector!");

    for (int i = 0; i < state.size(); ++i) {
        file_ << state[i] << ", ";
    }
    file_ << gripper_state << "\n";
}