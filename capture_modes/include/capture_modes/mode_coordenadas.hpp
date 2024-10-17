#pragma once

#include <autopilot/mode.hpp>
#include "pegasus_msgs/srv/waypoint.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <pegasus_msgs/msg/sensor_gps.hpp>
#include <pegasus_msgs/msg/rpy.hpp>

namespace autopilot {

class CoordenadasMode : public autopilot::Mode {

public:

    ~CoordenadasMode();

    void initialize() override;
    bool enter() override;
    bool exit() override;
    void update(double dt) override;

    void print_vector(const std::string& label, const double vec[3]);

    void lla_to_ecef(const double lla[3], double ecef[3]);
    void ecef_to_ned(const double ecef[3], const double ecef_ref[3], const double lla_ref[3], double ned[3]);
    void rpy_to_rotation_matrix(double roll, double pitch, double yaw, double R[3][3]);
    void apply_rotation(const double ned[3], const double R[3][3], double rotated_ned[3]);
    void translate_to_global(const double ned[3], const double global_translation[3], double global_ned[3]);
    void multiply_matrices(const double A[3][3], const double B[3][3], double C[3][3]);
    void rotation_matrix_to_rpy(const double R[3][3], double& roll, double& pitch, double& yaw);
    void update_vehicle_state();
    void inverse_rotation_matrix(const double, double R[3][3], double R_inv[3][3]);
    void multiply_matrix_vector(const double R[3][3], const double vec[3], double result[3]);
    //void inverse_apply_rotation(const double R[3][3], const double vec[3], double result[3]);
    //void global_to_local(const double global_pos[3], const double global_translation[3], const double R_global[3][3], const double R_local[3][3], double local_pos[3]);


    int Kp = 1;
	int Kv = 2;
	int Kpz = 1;
	int Kvz = 3;
    
    Eigen::Vector3d Pd{Eigen::Vector3d::Zero()};    // Desired position (of the target)

    float latitude = 0.0;

    Eigen::Vector3d pos_ned{Eigen::Vector3d::Zero()};

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
    double final_global_drone1_ned[3]={0.0,0.0,0.0};
    double final_global_drone2_ned[3]={0.0,0.0,0.0};

    double drone1_lla[3] = {0.0, 0.0, 0.0};
    double drone2_lla[3] = {0.0, 0.0, 0.0};

    float roll = 0.0;
    float pitch = 0.0;
    float yaw = 0.0; 

    void target_state_callback(const pegasus_msgs::msg::RPY::ConstSharedPtr msg);

    void target_gps_callback(const pegasus_msgs::msg::SensorGps::ConstSharedPtr msg);

    Eigen::Vector3d P{Eigen::Vector3d::Zero()};     // Position
    
    double ref_lla[3] = {47.397742, 8.545594, 488.05}; 
    //double ref_lla[3] = {37.7749, -122.4194, 10};  // Example reference point (San Francisco)    LLA drone_lla = {0.0,0.0,0.0};

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

    rclcpp::Subscription<pegasus_msgs::msg::SensorGps>::SharedPtr target_gps_sub_;

};


}