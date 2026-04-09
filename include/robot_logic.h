#ifndef ROBOT_PARAMS_H
#define ROBOT_PARAMS_H

#include <Eigen/Dense>
#include <vector>

namespace YouBot{

    // ENUM is a STATE class, not float
    enum class GripperState{
        Open = 0,
        Closed = 1
    };

    // ENUM class for each joint and his ID
    enum class StateIdx{
        ChassisPhi = 0, ChassisX = 1, ChassisY = 2,
        J1 = 3, J2 = 4, J3 = 5, J4 = 6, J5 = 7,
        W1 = 8, W2 = 9, W3 = 10, W4 = 11,
        Gripper = 12
    };

    namespace Frame{
        // -------Robot Values-------
        // INLINE - Tells the compiler the other cpp referencing to this const will be allowed
        // Constexpr - The compiler compiles with the value itself (not need to load it in running time)
    
        inline constexpr double wheel_radius = 0.0475;
        inline constexpr double frame_wheel_w = 0.15;
        inline constexpr double frame_wheel_l = 0.235;
        inline constexpr double b_frame_height_z = 0.0963;


    }
    


};


#endif