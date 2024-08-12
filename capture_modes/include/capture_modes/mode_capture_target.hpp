#pragma once

#include <autopilot/mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <capture_modes/mpc_controller.hpp>

namespace autopilot {

class CaptureTargetMode : public autopilot::Mode {

public:

    enum OperationMode {MPC_OFF, MPC_ON};

    ~CaptureTargetMode();

    void initialize() override;
    virtual bool enter();
    virtual bool exit() override;
    virtual void update(double dt);

    void mode_mpc_off();
    void mode_mpc_on();

    void update_vehicle_state();
    void target_state_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

protected:

    // Susbcriber for the target position
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_sub_;

    // MPC controller variables
    Eigen::Vector3d P{Eigen::Vector3d::Zero()};     // Position
    Eigen::Vector3d V{Eigen::Vector3d::Zero()};     // Velocity
    double yaw{0.0};                                // Yaw angle
    Eigen::Vector3d Pd{Eigen::Vector3d::Zero()};    // Desired position (of the target)
    Eigen::Vector3d Vd{Eigen::Vector3d::Zero()};    // Desired velocity (of the target)
    double yawd{0.0};                               // Desired yaw angle (of the target)

    // MPC gains
    double Kp{1.0};
    double Kv{1.0};
    double Kpz{1.0};
    double Kvz{1.0};

    // MPC controller function
    casadi::Function mpc_controller_;

    // Casadi variables for the MPC controller
    casadi::DM x0;
    casadi::DM xx = DM::zeros(14,26);
    casadi::DM uu = DM::zeros(4,25);

    // Control inputs to apply to the vehicle
    Eigen::Vector3d velocity_;

    // Operation Mode
    OperationMode operation_mode_{MPC_OFF};
};
}