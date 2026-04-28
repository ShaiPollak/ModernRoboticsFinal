#ifndef ROBOT_LOGIC_H
#define ROBOT_LOGIC_H

#include <Eigen/Dense>
#include <vector>
#include <array>
#include <memory>

namespace RobotLogic{
    
    class RobotKinematics{
    public:

        /** @brief Constructor for the RobotKinematics class
          * This constructor initializes the RobotKinematics object with default parameters. It sets up the internal state of the robot, including transformation matrices, Jacobians, and configuration vectors. The constructor prepares the object for subsequent configuration and state updates during mission execution.
        */
        RobotKinematics() = default;

        /** @brief Parameterized constructor for the RobotKinematics class
          * This constructor initializes the RobotKinematics object with specific parameters provided as arguments. It sets up the internal state of the robot based on the given transformation matrices, screw axes, and kinematic parameters. This allows for immediate use of the object with a defined robot configuration.
          * @param F The F matrix representing the robot's base kinematics
          * @param VbMappingIndices A vector of indices for mapping Vb to F_ext
          * @param H_0 The H matrix representing the robot's initial configuration
          * @param T_sb The transformation matrix from the robot frame to the space frame
          * @param T_b0 The transformation matrix from the arm base frame to the robot frame
          * @param M0_e The initial transformation matrix of the end-effector in its home position
          * @param B_list The list of screw axes for the arm joints in the end-effector frame
         */
        RobotKinematics(
          const Eigen::MatrixXd &F, 
          const std::vector<int>& VbMappingIndices, 
          const Eigen::MatrixXd &H_0, 
          const Eigen::MatrixXd &T_sb_init, 
          const Eigen::MatrixXd &T_b0, 
          const Eigen::MatrixXd &M0_e, 
          const Eigen::MatrixXd &B_list);

        
        // Setters and Getters for robot parameters

        void setF(const Eigen::MatrixXd& F);
        Eigen::MatrixXd getF() const noexcept;

        /** @brief Set the extended F matrix based on the provided VbMappingIndices
          * @param VbMappingIndices A vector of indices for mapping Vb to F_ext
          */
        void setFExtWithVbMappingIndices(const std::vector<int>& VbMappingIndices);
        std::pair<std::vector<int>, Eigen::MatrixXd> getFExt() const noexcept;

        /** @brief Set the initial configuration matrix H0
          * @param H_0 The initial configuration matrix
          */
        void setH0(const Eigen::MatrixXd& H_0);
        Eigen::MatrixXd getH0() const noexcept;

        /** @brief Set the transformation matrix from the robot frame to the space frame
          * @param T_sb_init The transformation matrix from the robot frame to the space frame 
          * at the initial configuration
          */
        void setTsb_init(const Eigen::MatrixXd& T_sb_init);
        Eigen::MatrixXd getRobotLocation_T_sb() const noexcept;

        /** @brief Set the transformation matrix from the arm base frame to the robot frame
          * @param T_b0 The transformation matrix from the arm base frame to the robot frame
          */
        void setTb0(const Eigen::MatrixXd& T_b0);
        Eigen::MatrixXd getTb0() const noexcept;

        /** @brief Set the initial transformation matrix of the end-effector in its home position
          * @param M0_e The initial transformation matrix of the end-effector
          */
        void setM0e(const Eigen::MatrixXd& M0_e);
        Eigen::MatrixXd getM0e() const noexcept;

        /** @brief Set the list of screw axes for the arm joints in the end-effector frame
          * @param B_list The list of screw axes
          */
        void setBList(const Eigen::MatrixXd& B_list);
        Eigen::MatrixXd getBList() const noexcept;

        void setDt(double dt);
        double getDt() const noexcept;

        /** @brief Set the maximum velocity for the wheels
          * @param vel Maximum wheel velocity in appropriate units (e.g., radians per second)
          * @param wheel_id Optional parameter to set max velocity for a specific wheel (if -1, sets for all wheels)
          */
        void setMaxWheelsVelocity(double vel, int wheel_id = -1);
        /** @brief Get the maximum velocity for the wheels
          * @param wheel_id Optional parameter to get max velocity for a specific wheel (if -1, returns max velocity for all wheels)
          * @return Maximum wheel velocity in appropriate units (e.g., radians per second)
          */
        double getMaxWheelsVelocity(int wheel_id = -1) const;

        /** @brief Set the maximum velocity for the arm joints
          * @param vel Maximum joint velocity in appropriate units (e.g., radians per second)
          * @param joint_id Optional parameter to set max velocity for a specific joint (if -1, sets for all joints)
          */
        void setMaxJointsVelocity(double vel, int joint_id = -1);
        /** @brief Get the maximum velocity for the arm joints
          * @param joint_id Optional parameter to get max velocity for a specific joint (if -1, returns max velocity for all joints)
          * @return Maximum joint velocity in appropriate units (e.g., radians per second)
          */
        double getMaxJointsVelocity(int joint_id = -1) const;
      
        /** @brief Set the position limits for the arm joints
          * @param min_pos A vector of minimum position limits for each joint
          * @param max_pos A vector of maximum position limits for each joint
          */
        void setJointPositionLimits(const std::vector<double>& min_pos, const std::vector<double>& max_pos);
        double getJointMinPosition(int joint_id) const;
        double getJointMaxPosition(int joint_id) const;

        
        // Kinematics and State Update Methods

        /** @brief Compute the next state of the robot given the current configuration and control inputs
          * @param q_current Current configuration vector (For our case: size 12: 3 chassis, 5 arm joints, 4 wheels)
          * @param u_controls Control input vector (For our case: size 9: 4 wheel speeds, 5 arm joints)
          * @return The next configuration vector (size 12: 3 chassis, 5 arm joints, 4 wheels)
          */
        Eigen::VectorXd computeNextState(const Eigen::VectorXd &u_controls); //9 vec velocity (4 wheels, 5 joints)
         /** @brief Update the current configuration of the robot to the next state
          * @param q_next Next configuration vector (For our case: size 12: 3 chassis, 5 arm joints, 4 wheels)
          */
        void updateToNextState(const Eigen::VectorXd &q_next);

        /** @brief Update the robot's location based on the current configuration and the next configuration
          * @param T_sb Current transformation matrix from the robot frame to the space frame
          * @param q_current Current configuration vector (For our case: size 12: 3 chassis, 5 arm joints, 4 wheels)
          * @param q_next Next configuration vector (For our case: size 12: 3 chassis, 5 arm joints, 4 wheels)
          */
        void updateRobotLocation(
          const Eigen::Matrix4d& T_sb, 
          const Eigen::VectorXd& q_current, 
          const Eigen::VectorXd& q_next);
        /** @brief Update the robot's configuration to the next state based on the current configuration and control inputs
          * @param q_next Next configuration vector (For our case: size 12: 3 chassis, 5 arm joints, 4 wheels)
          */
        void updateEndEffectorConfiguration(const Eigen::VectorXd& q);
        /** @brief Set the initial configuration of the robot
          * @param q_start Initial configuration vector (For our case: size 12: 3 chassis, 5 arm joints, 4 wheels)
          */
        void updateQState(const Eigen::VectorXd& q);
        /** @brief Update the Jacobian matrices based on the current configuration of the robot
          * @param q_current Current configuration vector (For our case: size 12: 3 chassis, 5 arm joints, 4 wheels)
          */
        void updateJacobian(const Eigen::VectorXd& q);

        /** @brief Get the current configuration of the robot
          * @return Current configuration vector (For our case: size 12: 3 chassis, 5 arm joints, 4 wheels)
          */
        Eigen::VectorXd getCurrentQState() const noexcept;
        /** @brief Get the current end-effector pose in the space frame
          * @return Current end-effector transformation matrix in the space frame
          */
        Eigen::Matrix4d getCurrentEndEffectorPose() const noexcept;
        
        /** @brief Compute the control inputs required to achieve a desired end-effector twist,
          * taking into account the current Jacobian and applying velocity limits to ensure safe operation.
          * @param V_t End-effector twist in the space frame (size 6: [vx, vy, vz, wx, wy, wz])
          * @return Control input vector (size 9: 4 wheel speeds, 5 arm joints)
          */
        Eigen::VectorXd computeControlsFromEndEffectorTwist(const Eigen::Vector<double, 6>& V_t);

        /** @brief Print the current configuration of the robot in a human-readable format
          * @param q Current configuration vector (For our case: size 12: 3 chassis, 5 arm joints, 4 wheels)
          */
        void printConfiguration(const Eigen::VectorXd& q) const noexcept;

        /** @brief Print the current end-effector pose of the robot in a human-readable format
          * @param T_current Current end-effector transformation matrix (4x4)
          * @param T_desired Desired end-effector transformation matrix (4x4)
          */
        void printTransformationMatrix(const Eigen::MatrixXd& T_current, const Eigen::MatrixXd& T_desired) const noexcept;
        /** @brief Print the current Jacobian matrix of the robot
          */
        void printJacobian() const noexcept;
        /** @brief Alert if the robot is near a singular configuration
          */
        void alertIfNearSingularity() const noexcept;

    private:

        // Robot Transformations Mats:
        Eigen::MatrixXd F_;                              // F Mat (for omnidirectional robot)
        Eigen::Matrix<double, 6, Eigen::Dynamic> F_ext_; // Extended F Mat (for omnidirectional robot)
        std::vector<int> VbMappingIndices_;              // Indices for mapping Vb to F_ext
        Eigen::MatrixXd H_0_;       // H Mat (for omnidirectional robot)
        Eigen::MatrixXd T_b0_;      // Arm base T.M in {b} (frame) 
        Eigen::MatrixXd T_sb_init_; // Robot frame T.M in {s} at the initial configuration
        Eigen::MatrixXd M0_e_;      // Initial Arm Transformation Matrix
        Eigen::MatrixXd B_list_;    // Initial Joints Screw Axises Lists in {e}

        Eigen::MatrixXd J_base_;    // Current Configuration of robot base Jacobain (F_ in Endeffector frame) in Endeffector frame     
        Eigen::MatrixXd J_arm_;     // Current Configuration of robot arm Jacobain (B_list_ in Endeffector frame) in Endeffector frame
        Eigen::MatrixXd J_e_;       // Current Configuration of robot's Jacobain (J_base_ stacked on J_arm_) in Endeffector frame
        
        // Current state of the robot
        Eigen::Matrix4d T_sb_;      // Current Robot location matrix in space frame
        Eigen::MatrixXd T_0e_;      // Current End-effector transformation matrix in arm base frame
        Eigen::Matrix4d T_se_;      // Current End-effector transformation matrix in space frame
        Eigen::VectorXd q_state_;   // Current Configuration of the robot

        int num_of_joints_;
        int num_of_controlable_wheels_;
        
        struct JointLimits {
            // Arm Position Limits (size: num_of_joints)
            std::vector<double> min_pos;
            std::vector<double> max_pos;

            // Velocity Limits (size: num_of_joints + num_of_wheels)
            std::vector<double> max_vel;

            // Soft limit buffer (e.g., 0.08 rad) 
            // Used to start slowing down BEFORE hitting the hard stop
            double buffer = 0.08; 
        } joint_limits_;

        double dt_ = 0.01; //In sec, default value, can be updated by setter

        bool near_singularity_ = false;

        // std::unique_ptr<CSVLogger> logger = nullptr;
    };
}

#endif