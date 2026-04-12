#ifndef ROBOT_LOGIC_H
#define ROBOT_LOGIC_H

#include <Eigen/Dense>
#include <vector>
#include <array>
#include <memory>
#include "robot_params.h"
#include "csv_logger.h"

namespace RobotLogic{
    
    class RobotKinematics{
    public:
        RobotKinematics(const Eigen::MatrixXd &F, const Eigen::MatrixXd &H_0, 
                        const Eigen::MatrixXd &T_sb, const Eigen::MatrixXd &T_b0, 
                        const Eigen::MatrixXd &M0_e, const Eigen::MatrixXd &B_list);

        void set_dt(double dt);
        double get_dt() const noexcept;

        void set_max_wheels_velocity(double vel);
        double get_max_wheels_velocity() const noexcept;

        void set_max_joints_velocity(double vel);
        double get_max_joints_velocity() const noexcept;

        void write_configuration_to_csv_file();
        void print_configuration(const Eigen::VectorXd& q, bool gripper_open);

        void openGrip();
        void closeGrip();
        int get_grip_state() const noexcept;

        void setInitialQState(const Eigen::VectorXd& q_start);
        
        Eigen::VectorXd computeNextState(const Eigen::VectorXd &q_current,  //12 vec conf (3 chassis, 5 arm, 4 wheels)
                                        const Eigen::VectorXd &u_controls); //9 vec velocity (4 wheels, 5 joints)                                    


    private:

        // Robot Transformations Mats:
        Eigen::MatrixXd F_;         // F Mat (for omnidirectional robot)
        Eigen::MatrixXd H_0_;       // H Mat (for omnidirectional robot)
        Eigen::MatrixXd T_b0_;      // Arm base T.M in {b} (frame) 
        Eigen::MatrixXd T_sb_;      // Robot frame T.M in {s}
        Eigen::MatrixXd M0_e_;      // Initial Arm Transformation Matrix
        Eigen::MatrixXd B_list_;    // Initial Joints Screw Axises Lists in {e}

        Eigen::MatrixXd T_eb_;      // 
        Eigen::MatrixXd J_e;        // Jacobian of the robot in the eyes of {e}

        Eigen::VectorXd u_limited_aux_; // Input buffer for saturated/clamped controls
        Eigen::VectorXd q_next_aux_;    // Workspace for intergration step
        Eigen::Vector3d delta_Vb_aux_;  // Intermediate Body Twist Vector dphi, dx, dy
        
        Eigen::VectorXd q_state_;   // Current Configuration of the robot

        int num_of_joints_;
        int num_of_controlable_wheels_;

        double max_wheels_velocity_ = 12.3;
        double max_joints_velocity_ = 1.5;

        double dt_ = 0.01; //In sec

        YouBot::Gripper::GripperState grip_state_ = YouBot::Gripper::GripperState::Open; //0 is closed and 1 is open
        std::unique_ptr<CSVLogger> logger = nullptr;


    };

    class RobotDynamics{
    public:

    private:
    };


    class RobotControl{
    public:



    private:

        
    };

    class Odometry{

    };
}


#endif