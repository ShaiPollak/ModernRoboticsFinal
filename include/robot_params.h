#ifndef ROBOT_PARAMS_H
#define ROBOT_PARAMS_H

#include <Eigen/Dense>
#include <vector>
#include <array>

namespace YouBot{
    // ENUM class for each joint and his ID
    // ENUM is a STATE class, not float

    /**
     * @brief Index mapping for the 13-element robot state vector.
     * phi, x, y (chassis), 5 arm joints, 4 wheel angles, 1 gripper state.
     */
    enum class StateIdx{
        ChassisPhi  = 0, 
        ChassisX    = 1, 
        ChassisY    = 2,
        J1          = 3, 
        J2          = 4, 
        J3          = 5, 
        J4          = 6, 
        J5          = 7,
        W1          = 8, 
        W2          = 9, 
        W3          = 10, 
        W4          = 11,
        Gripper     = 12
    };

    inline constexpr int num_of_joints = 5;
    inline constexpr int num_of_wheel = 4;

    namespace Gripper{
        
        inline constexpr double min_opening_distance = 0.02;
        inline constexpr double max_opening_distance = 0.07;
        inline constexpr double interior_length = 0.035;
        inline constexpr double distance_from_end_eff = 0.043;

        /** @enum Logical state of the end-effector gripper */
        enum class GripperState{
            Open = 0,
            Closed = 1
        };
    }


    namespace Wheel{
    inline constexpr std::array<double ,4> gamma = {
         M_PI /  4.0,
        -M_PI /  4.0,
         M_PI /  4.0,
        -M_PI /  4.0
        };
    }

    namespace Frame{
        // -------Robot Values-------
        // INLINE - Tells the compiler the other cpp referencing to this const will be allowed
        // Constexpr - The compiler compiles with the value itself (not need to load it in running time)
    
        inline constexpr double wheel_radius = 0.0475;
        inline constexpr double frame_wheel_w = 0.15;
        inline constexpr double frame_wheel_l = 0.235;
        inline constexpr double b_frame_height_z = 0.0963;
        
        /**
         * @brief H(0) matrix: Maps chassis Twist (v) to wheel angular velocities (u).
         * Equation: u = H(0) * v.
         * @return 4x3 matrix where each row corresponds to a wheel (W1-W4).
         */
        inline Eigen::Matrix<double, 4 ,3> H_0(){
            double k = frame_wheel_l + frame_wheel_w;
            Eigen::Matrix<double, 4 ,3> H_mat;

            H_mat << -k, 1, -1,
                      k, 1,  1,
                      k, 1, -1,
                     -k, 1,  1;
            return (1.0 / wheel_radius) * H_mat;
        }

        /**
         * @brief F matrix (Pseudo-inverse of H): Maps wheel speeds (u) to chassis Twist (Vb).
         * Used for Odometry calculations in NextState.
         * Calculated once and stored as a static constant for performance.
         */
        inline Eigen::Matrix<double, 3 ,4> F(){
            static const Eigen::Matrix<double, 3, 4> F_constant = 
                H_0().completeOrthogonalDecomposition().pseudoInverse();
            return F_constant;
}

        /**
         * @brief Computes the transformation matrix from space frame {s} to chassis {b}.
         * @param x Chassis X position (m).
         * @param y Chassis Y position (m).
         * @param phi Chassis rotation (rad).
         */
        inline Eigen::Matrix4d T_sb(double x, double y, double phi) {
            Eigen::Matrix4d T;
            T << std::cos(phi), -std::sin(phi), 0, x,
                 std::sin(phi),  std::cos(phi), 0, y,
                 0,              0,             1, b_frame_height_z,
                 0,              0,             0, 1;
            return T;
        }

        /** @brief Fixed offset from chassis frame {b} to arm base {0} */
        inline Eigen::Matrix<double, 4 ,4> T_b0(){
            Eigen::Matrix<double, 4, 4> T_b0_mat;
            
            T_b0_mat << 1, 0, 0, 0.1662,
                        0, 1, 0, 0,
                        0, 0, 1, 0.0026,
                        0, 0, 0, 1;
            return T_b0_mat;
        }
    }
        
    namespace Arm{
        /** @brief End-effector frame {e} relative to arm base {0} at home position. */
        inline Eigen::Matrix<double, 4, 4> M_0e(){
            Eigen::Matrix<double, 4, 4> M_0e_mat;

            M_0e_mat << 1, 0, 0, 0.033,
                        0, 1, 0, 0,
                        0, 0, 1, 0.6546,
                        0, 0, 0, 1;
            return M_0e_mat;       
        };

        /** @brief Screw axes B for the five joints in the end-effector frame {e}. */
        inline Eigen::Matrix<double, 6, 5> Blist() {
        Eigen::Matrix<double, 6, 5> Blist_mat;

        //          (S1,    S2,      S3,      S4,     S5)

        Blist_mat << 0,      0,       0,       0,      0,   // omega_x
                     0,     -1,      -1,      -1,      0,   // omega_y
                     1,      0,       0,       0,      1,   // omega_z
                     0,     -0.5076, -0.3526, -0.2176, 0,   // v_x
                     0.033,  0,       0,       0,      0,   // v_y
                     0,      0,       0,       0,      0;   // v_z

        return Blist_mat;
        }
    }
}

namespace Obj{
    inline constexpr double l = 0.05;
    inline constexpr double w = 0.05;
    inline constexpr double h = 0.05;

    inline Eigen::Vector3d start_pos(){
        // Starting position of Obj relative to S frame
        Eigen::Vector3d st_pos;
        st_pos << 1, 0, 0.025;  //x y z
        return st_pos;
    }

    inline Eigen::Vector3d end_pos(){
        // Ending position of Obj relative to S frame
        Eigen::Vector3d end_pos;
        end_pos << 0, -1, 0.025; //x y z
        return end_pos;
    }

    /** @brief Initial T_sc matrix (Space to Cube) */
    inline Eigen::Matrix4d T_sc_initial(){
        Eigen::Vector3d st_pos = start_pos();
        Eigen::Matrix4d T_mat = Eigen::Matrix4d::Identity();
        T_mat.block<3,1>(0,3) = st_pos; 
        return T_mat;
    }

    /** @brief Target T_sc matrix (Space to Cube) */
    inline Eigen::Matrix4d T_sc_end() {
        Eigen::Vector3d e_pos = end_pos();
        Eigen::Matrix4d T_sc_e_mat;
        T_sc_e_mat <<  0, 1, 0, e_pos[0],
                      -1, 0, 0, e_pos[1],
                       0, 0, 1, e_pos[2],
                       0, 0, 0, 1;
        return T_sc_e_mat;
    }
}


#endif