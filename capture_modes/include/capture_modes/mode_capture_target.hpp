#pragma once

#include "gen.h"
#include <casadi/casadi.hpp>
#include <autopilot/mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <capture_msgs/msg/capture.hpp>


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
    void target_state_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

    void compile_mpc_controller();

    void print_vector(const std::string& label, const Eigen::Vector3d& vec);
    void lla_to_ecef(const Eigen::Vector3d &lla, Eigen::Vector3d &ecef);
    void ecef_to_ned(const Eigen::Vector3d &ecef, const Eigen::Vector3d &ecef_ref, const Eigen::Vector3d &lla_ref, Eigen::Vector3d &ned);
    void rpy_to_rotation_matrix(double roll, double pitch, double yaw, Eigen::Matrix3d &R);
    void apply_rotation(const Eigen::Vector3d &ned, const Eigen::Matrix3d &R, Eigen::Vector3d &rotated_ned);
    void translate_to_global(const Eigen::Vector3d &ned, const Eigen::Vector3d &global_translation, Eigen::Vector3d &global_ned);
    void multiply_matrices(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B, Eigen::Matrix3d& C);
    void rotation_matrix_to_rpy(const Eigen::Matrix3d& R, double& roll, double& pitch, double& yaw);
    void quaternion_to_rotation_matrix(double q_w, double q_x, double q_y, double q_z, Eigen::Matrix3d &R);
    void inverse_rotation_matrix(const double, double R[3][3], double R_inv[3][3]);
    void multiply_matrix_vector(const double R[3][3], const double vec[3], double result[3]);

    Eigen::Vector3d Pd{Eigen::Vector3d::Zero()};    // Desired position (of the target)
    Eigen::Vector3d Pd2{Eigen::Vector3d::Zero()};    // Desired position (of the target)

    const double a = 6378137.0;               // WGS-84 Earth semimajor axis (m)
    const double f = 1.0 / 298.257223563;     // WGS-84 flattening
    const double b = a * (1 - f);             // Semi-minor axis
    const double e_sq = (a * a - b * b) / (a * a); // First eccentricity squared
    const double DEG2RAD = M_PI / 180.0;
    const double RAD2DEG = 180.0 / M_PI;
    const int SIZE = 3;
    Eigen::Vector3d final_global_drone1_ned{Eigen::Vector3d::Zero()};

    Eigen::Vector3d final_global_drone2_ned{Eigen::Vector3d::Zero()};

    //Eigen::Vector3d drone1_lla{Eigen::Vector3d::Zero()};

    //Eigen::Vector3d drone2_lla{Eigen::Vector3d::Zero()};

    Eigen::Vector3d drone1_ned{Eigen::Vector3d::Zero()};

    Eigen::Vector3d drone2_ned{Eigen::Vector3d::Zero()};

    Eigen::Vector3d ref_ecef{Eigen::Vector3d::Zero()};

    Eigen::Vector3d drone1_ecef{Eigen::Vector3d::Zero()};

    Eigen::Vector3d drone2_ecef{Eigen::Vector3d::Zero()};

    Eigen::Matrix3d R1_local{Eigen::Matrix3d::Zero()};

    Eigen::Matrix3d R2_local{Eigen::Matrix3d::Zero()};

    Eigen::Vector3d rotated_drone1_ned{Eigen::Vector3d::Zero()};

    Eigen::Vector3d rotated_drone2_ned{Eigen::Vector3d::Zero()};

    double global_roll = 0.0, global_pitch = 0.0, global_yaw = -90.0;
    Eigen::Matrix3d R_global{Eigen::Matrix3d::Zero()};

    Eigen::Vector3d global_rotated_drone1_ned{Eigen::Vector3d::Zero()};

    Eigen::Vector3d global_rotated_drone2_ned{Eigen::Vector3d::Zero()};

    Eigen::Vector3d global_translation{Eigen::Vector3d::Zero()};

    Eigen::Matrix3d R1_combined{Eigen::Matrix3d::Zero()};

    Eigen::Matrix3d R2_combined{Eigen::Matrix3d::Zero()};

    Eigen::Quaterniond q;
    Eigen::Quaterniond q_global;

    double yawd{0.0};

    double global_roll1_final = 0, global_pitch1_final = 0, global_yaw1_final = 0;
    double global_roll2_final = 0, global_pitch2_final = 0, global_yaw2_final = 0;

    Eigen::Vector3d drone1_lla{47.397742, 8.545634, 488.05}; //(3,0)
    Eigen::Vector3d drone2_lla{47.397742, 8.545594, 488.05}; //(0,0)

    Eigen::Vector3d ref_lla{47.397742, 8.545594, 488.05};

    float roll = 0.0;
    float pitch = 0.0;
    float yaw = 0.0; 

    float roll2 = 0.0;
    float pitch2 = 0.0;
    float yaw2 = 0.0;

    capture_msgs::msg::Capture capture_msg;
    Eigen::Vector3d acel_;
    float uu_mpc_[26];

    bool catched = false;


protected:

    bool check_finished();


    // Susbcriber for the target position
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_sub_;

    // MPC controller variables
    Eigen::Vector3d P{Eigen::Vector3d::Zero()};     // Position
    Eigen::Vector3d V{Eigen::Vector3d::Zero()};     // Velocity


    Eigen::Vector3d Vd{Eigen::Vector3d::Zero()};    // Desired velocity (of the target)

 
                                 // Desired yaw angle (of the target)

    // MPC gains
    double Kp{1.0};
    double Kv{1.0};
    double Kpz{1.0};
    double Kvz{1.0};
    int counter{0};

    // MPC controller function
    casadi::Function mpc_controller_;

    // Casadi variables for the MPC controller
    casadi::DM x0;
    casadi::DM xx = casadi::DM::zeros(14,26);
    casadi::DM uu = casadi::DM::zeros(4,25);
    casadi::DM uu_mpc = casadi::DM::zeros(4,25);


    // Control inputs to apply to the vehicle
    Eigen::Vector3d velocity_;
    // Operation Mode
    OperationMode operation_mode_{MPC_OFF};

    rclcpp::Publisher<capture_msgs::msg::Capture>::SharedPtr publisher_;


};
}