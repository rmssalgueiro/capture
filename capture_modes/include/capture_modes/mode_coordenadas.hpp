#pragma once

#include <autopilot/mode.hpp>
#include "pegasus_msgs/srv/waypoint.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <pegasus_msgs/msg/sensor_gps.hpp>
#include <pegasus_msgs/msg/rpy.hpp>
#include <capture_msgs/msg/capture.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>


namespace autopilot {

class CoordenadasMode : public autopilot::Mode {

public:

    ~CoordenadasMode();

    void initialize() override;
    bool enter() override;
    bool exit() override;
    void update(double dt) override;

    void print_vector(const std::string& label, const Eigen::Vector3d& vec);
    void lla_to_ecef(const Eigen::Vector3d &lla, Eigen::Vector3d &ecef);
    void ecef_to_ned(const Eigen::Vector3d &ecef, const Eigen::Vector3d &ecef_ref, const Eigen::Vector3d &lla_ref, Eigen::Vector3d &ned);
    void rpy_to_rotation_matrix(double roll, double pitch, double yaw, Eigen::Matrix3d &R);
    void apply_rotation(const Eigen::Vector3d &ned, const Eigen::Matrix3d &R, Eigen::Vector3d &rotated_ned);
    void translate_to_global(const Eigen::Vector3d &ned, const Eigen::Vector3d &global_translation, Eigen::Vector3d &global_ned);
    void multiply_matrices(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B, Eigen::Matrix3d& C);
    void rotation_matrix_to_rpy(const Eigen::Matrix3d& R, double& roll, double& pitch, double& yaw);
    void quaternion_to_rotation_matrix(double q_w, double q_x, double q_y, double q_z, Eigen::Matrix3d &R);

    void update_vehicle_state();
    void inverse_rotation_matrix(const double, double R[3][3], double R_inv[3][3]);
    void multiply_matrix_vector(const double R[3][3], const double vec[3], double result[3]);


    int Kp = 0;
	int Kv = 0;
	int Kpz = 0;
	int Kvz = 0;
    double Ki = 0.1;  // Choose an appropriate gain for Ki

    Eigen::Vector3d integral_error{Eigen::Vector3d::Zero()};

    Eigen::Vector3d Pd{Eigen::Vector3d::Zero()};    // Desired position (of the target)

    float latitude = 0.0;

    //Eigen::Vector3d pos_ned{Eigen::Vector3d::Zero()};
    Eigen::Vector3d final1_ned{Eigen::Vector3d::Zero()};
    Eigen::Vector3d final2_ned{Eigen::Vector3d::Zero()};

    Eigen::Vector3d u{Eigen::Vector3d::Zero()};
    Eigen::Vector3d target_pos{Eigen::Vector3d::Zero()};
    
    // Constants
    const double a = 6378137.0;               // WGS-84 Earth semimajor axis (m)
    const double f = 1.0 / 298.257223563;     // WGS-84 flattening
    const double b = a * (1 - f);             // Semi-minor axis
    const double e_sq = (a * a - b * b) / (a * a); // First eccentricity squared
    const double DEG2RAD = M_PI / 180.0;
    const double RAD2DEG = 180.0 / M_PI;
    const int SIZE = 3;
    Eigen::Vector3d final_global_drone1_ned{Eigen::Vector3d::Zero()};

    Eigen::Vector3d final_global_drone2_ned{Eigen::Vector3d::Zero()};

    Eigen::Vector3d drone1_lla{Eigen::Vector3d::Zero()};

    Eigen::Vector3d drone2_lla{Eigen::Vector3d::Zero()};

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

    float roll = 0.0;
    float pitch = 0.0;
    float yaw = 0.0; 

    float roll2 = 0.0;
    float pitch2 = 0.0;
    float yaw2 = 0.0;

    double counter = 0.0;

    void target_state_callback(const pegasus_msgs::msg::RPY::ConstSharedPtr msg);
    void target_state_callback2(const pegasus_msgs::msg::RPY::ConstSharedPtr msg);
    void target_gps_callback(const pegasus_msgs::msg::SensorGps::ConstSharedPtr msg);
    void target_gps_callback2(const pegasus_msgs::msg::SensorGps::ConstSharedPtr msg);
    void target_pos_callback2(const nav_msgs::msg::Odometry::ConstSharedPtr msg);


    Eigen::Vector3d P{Eigen::Vector3d::Zero()};     // Shuttle Position
    Eigen::Vector3d V{Eigen::Vector3d::Zero()};     // Shuttle Velocity
    Eigen::Vector3d P2{Eigen::Vector3d::Zero()};     // Target Position 

    
    Eigen::Vector3d ref_lla{47.397742, 8.545594, 488.05};
    
    //px4 default 454671160 -737578370

    capture_msgs::msg::Capture capture_msg;

protected:

    // The waypoint service callback
    void waypoint_callback(const pegasus_msgs::srv::Waypoint::Request::SharedPtr request, const pegasus_msgs::srv::Waypoint::Response::SharedPtr response);
    
    // Check if the waypoint is already set
    bool waypoint_set_{false};

    // The target position and attitude waypoint to be at
    float target_yaw{0.0f};

    // The waypoint service server that sets the position and attitude waypoints at a given target
    rclcpp::Service<pegasus_msgs::srv::Waypoint>::SharedPtr waypoint_service_{nullptr};

    rclcpp::Subscription<pegasus_msgs::msg::RPY>::SharedPtr target_state_sub_;
    rclcpp::Subscription<pegasus_msgs::msg::RPY>::SharedPtr target_state_sub2_;


    rclcpp::Subscription<pegasus_msgs::msg::SensorGps>::SharedPtr target_gps_sub_;
    rclcpp::Subscription<pegasus_msgs::msg::SensorGps>::SharedPtr target_gps_sub2_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_pos_sub2_;

    rclcpp::Publisher<capture_msgs::msg::Capture>::SharedPtr publisher_;

};


}