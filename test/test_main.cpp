#include <iostream>
#include <Eigen/Dense>
#include "robot_logic.h"
#include "robot_params.h"
#include "trajectory_gen.h"

int main() {

    // Basic Controls and CSV File Print Test:

    try {
        std::cout << "---------Testing Basic CSV output and Odometry-----------------" << std::endl;
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

        std::cout << "-----------------Completed Test-----------------" << std::endl;
        std::cout << "------------------------------------------------" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error during simulation: " << e.what() << std::endl;
        return 1;
    }

    //----------------------------------------------------------------------

    // Trajectory Test

    try {
        std::cout << "--------- Deep Comparison: Screw vs Cartesian ---------" << std::endl;

        // נקודות קצה
        Eigen::Matrix4d T_start = YouBot::Arm::T_se_start(0, 0, 0);
        Eigen::Matrix4d T_end = YouBot::Arm::T_se_StandoffPickUp();

        // הדפסה מלאה של מטריצות המטרה
        std::cout << "[1] FULL START MATRIX (T_start):\n" << T_start << "\n" << std::endl;
        std::cout << "[2] FULL END MATRIX (T_end):\n" << T_end << "\n" << std::endl;

        // גנרציה
        RobotLogic::TrajectoryGen screw_gen("Screw", T_start, T_end);
        screw_gen.generateTrajectory(RobotLogic::TrajectoryType::ScrewTrajectory, 10.0);
        const auto& screw_traj = screw_gen.getTrajectory();

        RobotLogic::TrajectoryGen cart_gen("Cartesian", T_start, T_end);
        cart_gen.generateTrajectory(RobotLogic::TrajectoryType::CartesianTrajectory, 10.0);
        const auto& cart_traj = cart_gen.getTrajectory();

        // דגימות בדרך
        std::vector<int> sample_indices = {250, 500, 750}; 
        
        std::cout << "--- Intermediate Waypoints (T-Matrix) ---" << std::endl;
        for (int idx : sample_indices) {
            double t = idx * 0.01;
            std::cout << "\n>>> At t = " << t << "s (Index " << idx << "):" << std::endl;
            std::cout << "  [SCREW]:\n" << screw_traj[idx] << std::endl;
            std::cout << "  [CARTESIAN]:\n" << cart_traj[idx] << std::endl;
            
            double pos_diff = (screw_traj[idx].block<3,1>(0,3) - cart_traj[idx].block<3,1>(0,3)).norm();
            std::cout << "  --> Difference: " << pos_diff << " meters." << std::endl;
        }

        // בדיקה סופית של המטריצה האחרונה בוקטור (לוודא שהיא אכן T_end)
        std::cout << "\n[3] FINAL TRAJECTORY POINT (Index 1000):" << std::endl;
        std::cout << "  Screw Last Point:\n" << screw_traj.back() << std::endl;
        std::cout << "  Cartesian Last Point:\n" << cart_traj.back() << std::endl;

        // Continuity Check
        auto run_continuity = [](const std::vector<Eigen::Matrix4d>& traj, std::string label) {
            double max_step = 0;
            for(size_t i = 1; i < traj.size(); ++i) {
                double step = (traj[i].block<3,1>(0,3) - traj[i-1].block<3,1>(0,3)).norm();
                if(step > max_step) max_step = step;
            }
            std::cout << "   Max translation step [" << label << "]: " << max_step << " meters." << std::endl;
        };

        std::cout << "\n--- Continuity Check ---" << std::endl;
        run_continuity(screw_traj, "Screw");
        run_continuity(cart_traj, "Cartesian");

        std::cout << "\n--------- End of Deep Comparison ---------" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Simulation Error: " << e.what() << std::endl;
    }





    return 0;
}