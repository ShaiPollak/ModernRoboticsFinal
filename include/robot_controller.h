#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H
#include <Eigen/Dense>
  
namespace RobotLogic { 

class RobotControl{
public:

        enum ControllerType{
            PID,
            PI,
            P,
            AI
        };
        
        /** @brief Constructor for the RobotControl class
          * This constructor initializes the RobotControl object with default parameters. It sets up the internal state of the controller, including default gains for the PID controller and a default time step. The constructor prepares the object for subsequent control computations during mission execution.
        */
        RobotControl(const double dt);
        
        void setDt(const double dt);
        void setKp(const Eigen::Matrix<double, 6, 6>& Kp);
        void setKi(const Eigen::Matrix<double, 6, 6>& Ki);
        void setKd(const Eigen::Matrix<double, 6, 6>& Kd);
        void setGains(
            const Eigen::Matrix<double, 6, 6>& Kp, 
            const Eigen::Matrix<double, 6, 6>& Ki, 
            const Eigen::Matrix<double, 6, 6>& Kd);

        double getDt() const noexcept;
        Eigen::Matrix<double, 6, 6> getKp() const noexcept;
        Eigen::Matrix<double, 6, 6> getKi() const noexcept;
        Eigen::Matrix<double, 6, 6> getKd() const noexcept;

        /** @brief Compute the feedforward control twist in task space based on the desired end-effector configuration and twist
          * @param X Current end-effector configuration (4x4 homogeneous transformation matrix)
          * @param Xd Desired end-effector configuration (4x4 homogeneous transformation matrix)
          * @param V_d Desired end-effector twist (6x1 vector)
          * @return Feedforward control twist in task space (6x1 vector)
          */
        Eigen::Vector<double, 6> feedForwardControl(const Eigen::Matrix4d& X, const Eigen::Matrix4d& Xd, const Eigen::Vector<double, 6>& V_d);
        /** @brief Compute the feedback control twist in task space based on the current and desired end-effector configurations
          * @param X Current end-effector configuration (4x4 homogeneous transformation matrix)
          * @param Xd Desired end-effector configuration (4x4 homogeneous transformation matrix)
          * @return Feedback control twist in task space (6x1 vector)
          */
        Eigen::Vector<double, 6> feedbackControl(const Eigen::Matrix4d& X, const Eigen::Matrix4d& Xd);

        /** @brief Compute the total control twist in task space by combining feedforward and feedback control
          * @param X Current end-effector configuration (4x4 homogeneous transformation matrix)
          * @param Xd Desired end-effector configuration (4x4 homogeneous transformation matrix)
          * @param V_d Desired end-effector twist (6x1 vector)
          * @return Total control twist in task space (6x1 vector)
          */
        Eigen::Vector<double, 6> calNextTwistInTaskSpace(const Eigen::Matrix4d& X, const Eigen::Matrix4d& Xd, const Eigen::Vector<double, 6>& V_d);
        
        /** @brief Get the integrated error twist in task space over time
          * @return Integrated error twist in task space (6x1 vector)
          */
        Eigen::Vector<double, 6> getXErrIntegration() const noexcept;
        /** @brief Get the last computed error twist in task space
          * @return Last computed error twist in task space (6x1 vector)
          */
        Eigen::Vector<double, 6> getCurrentXErr() const noexcept;
    private:

        double dt_;
        Eigen::Matrix<double, 6, 6> Kp_;
        Eigen::Matrix<double, 6, 6> Ki_;
        Eigen::Matrix<double, 6, 6> Kd_;

        Eigen::Vector<double, 6> last_X_err_;
        Eigen::Vector<double, 6> X_err_integration_;
            
        };
}
#endif