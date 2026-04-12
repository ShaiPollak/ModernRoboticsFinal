#include <iostream>
#include <Eigen/Dense>
#include "robot_logic.h"
#include "robot_params.h"

int main() {
    try {
        // 1. אתחול הקינמטיקה עם הפרמטרים מה-Namespace של YouBot
        RobotLogic::RobotKinematics robot(
            YouBot::Frame::F(),
            YouBot::Frame::H_0(),
            YouBot::Frame::T_sb(0, 0, 0), // התחלה בראשית הצירים
            YouBot::Frame::T_b0(),
            YouBot::Arm::M_0e(),
            YouBot::Arm::Blist()
        );

        // 2. הגדרת מצב התחלתי (12 אלמנטים)
        // [phi, x, y, J1, J2, J3, J4, J5, W1, W2, W3, W4]
        Eigen::VectorXd q_start = Eigen::VectorXd::Zero(12);
        robot.setInitialQState(q_start);

        // 3. הגדרת פקודות מהירות (9 אלמנטים)
        // [J1, J2, J3, J4, J5, W1, W2, W3, W4]
        Eigen::VectorXd u_controls = Eigen::VectorXd::Zero(9);

        // טסט: נסיעה קדימה (כל הגלגלים ב-10 רדיאנים לשנייה) וסיבוב מפרק 1
        u_controls << 0.2, 0.0, 0.0, 0.0, 0.0,  // מפרקי הזרוע
                      10.0, 10.0, 10.0, 10.0;   // גלגלים

        std::cout << "--- Starting Test Simulation (1 second) ---" << std::endl;

        // 4. לולאת סימולציה (100 צעדים = 1 שניה ב-dt=0.01)
        for (int i = 0; i < 100; ++i) {
            q_start = robot.computeNextState(q_start, u_controls);
        }

        std::cout << "\nTest Complete!" << std::endl;
        std::cout << "Check 'robot_log.csv' for the generated trajectory." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error during simulation: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}