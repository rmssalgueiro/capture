#include "capture_modes/mode_coordenadas.hpp"
#include "pegasus_utils/rotations.hpp"
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include "pegasus_utils/rotations.hpp"


namespace autopilot {

CoordenadasMode::~CoordenadasMode() {
    // Terminate the waypoint service
    this->waypoint_service_.reset();
}

void CoordenadasMode::initialize() {
    
    // Create the waypoint service server
    node_->declare_parameter<std::string>("autopilot.CoordenadasMode.set_waypoint_service", "set_waypoint"); 
    this->waypoint_service_ = this->node_->create_service<pegasus_msgs::srv::Waypoint>(node_->get_parameter("autopilot.CoordenadasMode.set_waypoint_service").as_string(), std::bind(&CoordenadasMode::waypoint_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    target_gps_sub_ = node_->create_subscription<pegasus_msgs::msg::SensorGps>("/drone1/fmu/sensors/gps", rclcpp::SensorDataQoS(), 
    std::bind(&CoordenadasMode::target_gps_callback, this, std::placeholders::_1));

    target_gps_sub2_ = node_->create_subscription<pegasus_msgs::msg::SensorGps>("/drone2/fmu/sensors/gps", rclcpp::SensorDataQoS(), 
    std::bind(&CoordenadasMode::target_gps_callback2, this, std::placeholders::_1));

    target_state_sub_ = node_->create_subscription<pegasus_msgs::msg::RPY>("/drone1/fmu/filter/rpy", rclcpp::SensorDataQoS(), 
    std::bind(&CoordenadasMode::target_state_callback, this, std::placeholders::_1));

    target_state_sub2_ = node_->create_subscription<pegasus_msgs::msg::RPY>("/drone2/fmu/filter/rpy", rclcpp::SensorDataQoS(), 
    std::bind(&CoordenadasMode::target_state_callback2, this, std::placeholders::_1));

    target_pos_sub2_ = node_->create_subscription<nav_msgs::msg::Odometry>("/drone1/fmu/filter/state", rclcpp::SensorDataQoS(),
    std::bind(&CoordenadasMode::target_pos_callback2, this, std::placeholders::_1));

    // Initialize the publisher for the global position of the vehicle    
    //publisher_ = node_->create_publisher<capture_msgs::msg::Capture>("Capture", 10);
    node_->declare_parameter<std::string>("capture.publishers.status", "capture/status");
    publisher_ = this->node_->create_publisher<capture_msgs::msg::Capture>(
        this->node_->get_parameter("capture.publishers.status").as_string(), rclcpp::SensorDataQoS());
        

    RCLCPP_INFO(this->node_->get_logger(), "CoordenadasMode initialized");


}


void CoordenadasMode::print_vector(const std::string& label, const Eigen::Vector3d& vec) {
    std::cout << label << ": [" << vec[0] << ", " << vec[1] << ", " << vec[2] << "]" << std::endl;
}

// Function to convert LLA to ECEF

void CoordenadasMode::lla_to_ecef(const Eigen::Vector3d &lla, Eigen::Vector3d &ecef) {
    const double lat_rad = lla[0] * DEG2RAD;
    const double lon_rad = lla[1] * DEG2RAD;
    const double alt = lla[2];

    const double N = a / std::sqrt(1 - e_sq * std::sin(lat_rad) * std::sin(lat_rad));

    ecef[0] = (N + alt) * std::cos(lat_rad) * std::cos(lon_rad);
    ecef[1] = (N + alt) * std::cos(lat_rad) * std::sin(lon_rad);
    ecef[2] = (N * (1 - e_sq) + alt) * std::sin(lat_rad);
}

// Function to convert ECEF to NED relative to a reference poin
void CoordenadasMode::ecef_to_ned(const Eigen::Vector3d &ecef, const Eigen::Vector3d &ecef_ref, const Eigen::Vector3d &lla_ref, Eigen::Vector3d &ned) {
    // Convert reference latitude and longitude to radians
    const double lat_ref_rad = lla_ref[0] * DEG2RAD;
    const double lon_ref_rad = lla_ref[1] * DEG2RAD;

    // Calculate the difference between ECEF coordinates and reference ECEF
    Eigen::Vector3d delta_ecef = ecef - ecef_ref;

    // Precompute trigonometric values
    const double sin_lat = std::sin(lat_ref_rad);
    const double cos_lat = std::cos(lat_ref_rad);
    const double sin_lon = std::sin(lon_ref_rad);
    const double cos_lon = std::cos(lon_ref_rad);

    // Calculate NED coordinates using the rotation matrix from ECEF to NED
    ned[0] = -sin_lat * cos_lon * delta_ecef[0] - sin_lat * sin_lon * delta_ecef[1] + cos_lat * delta_ecef[2];  // North
    ned[1] = -sin_lon * delta_ecef[0] + cos_lon * delta_ecef[1];                                                 // East
    ned[2] = -cos_lat * cos_lon * delta_ecef[0] - cos_lat * sin_lon * delta_ecef[1] - sin_lat * delta_ecef[2];    // Down
}

//Function to generate the rotation matrix from RPY
void CoordenadasMode::rpy_to_rotation_matrix(double roll, double pitch, double yaw, Eigen::Matrix3d &R) {
    // Convert angles from degrees to radians
    roll = roll * DEG2RAD;
    pitch = pitch * DEG2RAD;
    yaw = yaw * DEG2RAD;

    // Precompute sine and cosine of the angles
    double cos_roll = std::cos(roll);
    double sin_roll = std::sin(roll);
    double cos_pitch = std::cos(pitch);
    double sin_pitch = std::sin(pitch);
    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);

    // Build the rotation matrix
    R(0, 0) = cos_yaw * cos_pitch;
    R(0, 1) = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
    R(0, 2) = cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll;
    
    R(1, 0) = sin_yaw * cos_pitch;
    R(1, 1) = sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;
    R(1, 2) = sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll;

    R(2, 0) = -sin_pitch;
    R(2, 1) = cos_pitch * sin_roll;
    R(2, 2) = cos_pitch * cos_roll;
}

void CoordenadasMode::quaternion_to_rotation_matrix(double q_w, double q_x, double q_y, double q_z, Eigen::Matrix3d &R) {
    // Precompute repeated terms
    double q_x2 = q_x * q_x;
    double q_y2 = q_y * q_y;
    double q_z2 = q_z * q_z;
    double q_wx = q_w * q_x;
    double q_wy = q_w * q_y;
    double q_wz = q_w * q_z;
    double q_xy = q_x * q_y;
    double q_xz = q_x * q_z;
    double q_yz = q_y * q_z;

    // Set the rotation matrix elements
    R(0, 0) = 1 - 2 * (q_y2 + q_z2);
    R(0, 1) = 2 * (q_xy - q_wz);
    R(0, 2) = 2 * (q_xz + q_wy);
    
    R(1, 0) = 2 * (q_xy + q_wz);
    R(1, 1) = 1 - 2 * (q_x2 + q_z2);
    R(1, 2) = 2 * (q_yz - q_wx);
    
    R(2, 0) = 2 * (q_xz - q_wy);
    R(2, 1) = 2 * (q_yz + q_wx);
    R(2, 2) = 1 - 2 * (q_x2 + q_y2);
}

// Function to apply the rotation matrix to the NED vector
void CoordenadasMode::apply_rotation(const Eigen::Vector3d &ned, const Eigen::Matrix3d &R, Eigen::Vector3d &rotated_ned) {
    rotated_ned = R * ned;  // Eigen handles the matrix-vector multiplication internally
}

void CoordenadasMode::multiply_matrices(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B, Eigen::Matrix3d& C) {
    C = A * B;  // Eigen handles matrix multiplication natively
}

void CoordenadasMode::multiply_matrix_vector(const double R[3][3], const double vec[3], double result[3]) {
    for (int i = 0; i < 3; ++i) {
        result[i] = 0;
        for (int j = 0; j < 3; ++j) {
            result[i] += R[i][j] * vec[j];
        }
    }
}

// Function to compute the inverse of a 3x3 rotation matrix (for orthogonal matrices, it's simply the transpose)
void CoordenadasMode::inverse_rotation_matrix(const double, double R[3][3], double R_inv[3][3]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R_inv[i][j] = R[j][i];  // Transpose of R
        }
    }
}


void CoordenadasMode::rotation_matrix_to_rpy(const Eigen::Matrix3d& R, double& roll, double& pitch, double& yaw) {
    pitch = -std::asin(R(2, 0));  // Same as R[2][0]

    if (std::abs(R(2, 0)) < 0.99999) {
        roll = std::atan2(R(2, 1), R(2, 2));  // R[2][1], R[2][2]
        yaw = std::atan2(R(1, 0), R(0, 0));   // R[1][0], R[0][0]
    } else {
        roll = std::atan2(-R(1, 2), R(1, 1));  // R[1][2], R[1][1]
        yaw = 0.0;
    }

    // Convert back to degrees
    roll *= RAD2DEG;
    pitch *= RAD2DEG;
    yaw *= RAD2DEG;
}

// Function to translate NED coordinates to a global reference frame
void CoordenadasMode::translate_to_global(const Eigen::Vector3d &ned, const Eigen::Vector3d &global_translation, Eigen::Vector3d &global_ned) {
    global_ned = ned + global_translation;
}


void CoordenadasMode::target_gps_callback2(const pegasus_msgs::msg::SensorGps::ConstSharedPtr msg) {
    
    if (drone2_lla[0] == 0.0 && drone2_lla[1] == 0.0 && drone2_lla[2] == 0.0) {
        //drone2_lla[0] = msg->latitude_deg;
        //drone2_lla[1] = msg->longitude_deg;
        //drone2_lla[2] = msg->altitude_msl;
        //double drone1_lla[3] = {47.397769, 8.545594, 488.05};
        //double drone2_lla[3] = {47.397742, 8.545634, 488.05};

    }
}

void CoordenadasMode::target_gps_callback(const pegasus_msgs::msg::SensorGps::ConstSharedPtr msg) {

    //posição inicial LLA drone 1
    if (drone1_lla[0] == 0.0 && drone1_lla[1] == 0.0 && drone1_lla[2] == 0.0) {
        //drone1_lla[0] = msg->latitude_deg;
        //drone1_lla[1] = msg->longitude_deg;
        //drone1_lla[2] = msg->altitude_msl;
        //double drone1_lla[3] = {47.397769, 8.545594, 488.05};
        //double drone2_lla[3] = {47.397742, 8.545634, 488.05};

    }     
}

void CoordenadasMode::target_state_callback2(const pegasus_msgs::msg::RPY::ConstSharedPtr msg) {

    roll2 = msg->roll;
    pitch2 = msg->pitch;
    yaw2 = msg->yaw;
    //RCLCPP_WARN(this->node_->get_logger(), "RPY (%f, %f, %f)", roll, pitch, yaw);
}

void CoordenadasMode::target_state_callback(const pegasus_msgs::msg::RPY::ConstSharedPtr msg) {

    roll = msg->roll;
    pitch = msg->pitch;
    yaw = msg->yaw;
    //RCLCPP_WARN(this->node_->get_logger(), "RPY (%f, %f, %f)", roll, pitch, yaw);
}

bool CoordenadasMode::enter() {

    Kp = 4;
	Kv = 4;
	Kpz = 1;
	Kvz = 3;
    // Return true to indicate that the mode has been entered successfully
    return true;
}

bool CoordenadasMode::exit() {
    
    // Nothing to do here
    return true;   // Return true to indicate that the mode has been exited successfully
}

void CoordenadasMode::update(double dt) {

    // Get the current state of the vehicle
    State state = get_vehicle_state();

    /*
    if(target_yaw == 2.0){
        //drone1_lla[0] = 47.397769;
        //drone1_lla[1] = 8.545594;
        drone1_lla[0] = 38.621955;
        drone1_lla[1] = -9.153695;
        drone1_lla[2] = 47.0;
    }
    
    else if(target_yaw == 1.0){
        drone1_lla[0] = 47.397742;
        drone1_lla[1] = 8.545634;
        drone1_lla[2] = 488.05;
    }
    */
    drone1_lla[0] = 38.621955;
    drone1_lla[1] = -9.153695;
    drone1_lla[2] = 47.0;

    q_global.x() = 0.0;
    q_global.y() = 0.0;
    q_global.z() = 0.0;//- sqrt(2.0) / 2.0;
    q_global.w() = 1.0;//sqrt(2.0) / 2.0;
    
    //double drone1_local_position[3] = {state.position[0], state.position[1], state.position[2]};  // Drone has moved in local frame
    //double drone1_local_position[3] = {0.0,0.0,0.0};  // Drone has moved in local frame
    //double drone2_local_position[3] = {P2[0], P2[1], P2[2]};  // Drone has moved in local frame

    // Step 1: Convert reference point and both drones' LLA positions to ECEF
    lla_to_ecef(ref_lla, ref_ecef);
    lla_to_ecef(drone1_lla, drone1_ecef);
    //lla_to_ecef(drone2_lla, drone2_ecef);


    // Step 2: Convert both drones' positions from ECEF to NED
    drone1_ned[0] = 0.0;  // Assign X value
    drone1_ned[1] = 0.0;  // Assign Y value
    drone1_ned[2] = 0.0;  // Assign Z value
    


    ecef_to_ned(drone1_ecef, ref_ecef, ref_lla, drone1_ned);

    //ecef_to_ned(drone2_ecef, ref_ecef, ref_lla, drone2_ned);

    //if(counter==0.0){
    //    quaternion_to_rotation_matrix(state.attitude.w(), state.attitude.x(), state.attitude.y(), state.attitude.z(), R1_local);
    //    counter++;
    //}
    // Step 4: Apply drone's local RPY rotation to the NED coordinates
    quaternion_to_rotation_matrix(state.attitude.w(), state.attitude.x(), state.attitude.y(), state.attitude.z(), R2_local);
    quaternion_to_rotation_matrix(1.0, 0.0, 0.0, 0.0, R1_local);

    // Step 3: Add the local position to the NED coordinates

    drone1_ned[0] += state.position[0];  // Add local x to NED north
    drone1_ned[1] += state.position[1];  // Add local y to NED east
    drone1_ned[2] += state.position[2];  // Add local z to NED down
    apply_rotation(drone1_ned, R1_local, rotated_drone1_ned);
    //apply_rotation(drone2_ned, R2_local, rotated_drone2_ned);

    // Step 5: Define a global rotation matrix (for example, rotating global frame by some RPY angles)
    quaternion_to_rotation_matrix(q_global.w(), q_global.x(), q_global.y(), q_global.z(), R_global);

    //rpy_to_rotation_matrix(global_roll, global_pitch, global_yaw, R_global);

    // Apply global rotation to the NED coordinates of the drone
    apply_rotation(rotated_drone1_ned, R_global, global_rotated_drone1_ned);

    //apply_rotation(rotated_drone2_ned, R_global, global_rotated_drone2_ned);

    // Step 6: Define a translation vector to the global reference frame

    // Step 7: Translate the drone to the global frame

    translate_to_global(global_rotated_drone1_ned, global_translation, final_global_drone1_ned);
    //translate_to_global(global_rotated_drone2_ned, global_translation, final_global_drone2_ned);


    // Step 8: Combine the local and global rotation matrices to get the final orientation in the global frame

    multiply_matrices(R_global, R1_local, R1_combined);
    //multiply_matrices(R_global, R2_local, R2_combined);


    // Step 9: Extract the final global RPY angles

    //rotation_matrix_to_rpy(R2_local, global_roll1_final, global_pitch1_final, global_yaw1_final);
    rotation_matrix_to_rpy(R2_local, global_roll1_final, global_pitch1_final, global_yaw1_final);

    //rotation_matrix_to_rpy(R2_combined, global_roll2_final, global_pitch2_final, global_yaw2_final);

    capture_msg.global_pos_shuttle[0] = final_global_drone1_ned[0];
    capture_msg.global_pos_shuttle[1] = final_global_drone1_ned[1];
    capture_msg.global_pos_shuttle[2] = final_global_drone1_ned[2];

    capture_msg.quaternion[0] = state.attitude.w();
    capture_msg.quaternion[1] = state.attitude.x();
    capture_msg.quaternion[2] = state.attitude.y();
    capture_msg.quaternion[3] = state.attitude.z();

    publisher_->publish(capture_msg);

    // Compute the position error and velocity error using the path desired position and velocity
    Eigen::Vector3d pos_vector = final_global_drone1_ned - target_pos;
    //RCLCPP_WARN(this->node_->get_logger(), "Position (%f, %f, %f)", pos_vector[0], pos_vector[1], pos_vector[2]);
    //integral_error[0] += pos_vector[0] * dt;
    //integral_error[1] += pos_vector[1] * dt;
    //integral_error[2] += pos_vector[2] * dt;
    //Eigen::Vector3d vel_error = target_velocity - state.velocity;
    //state = get_vehicle_state();
    // Compute the desired control output acceleration for each controller
    //u[0] = - Kp * pos_vector[0] - Kv * state.velocity[0];
    //u[1] = - Kp * pos_vector[1] - Kv * state.velocity[1];
    //u[2] = - Kpz * pos_vector[2] - Kvz * state.velocity[2];

    //u[0] = -Kp * pos_vector[0] - Kv * state.velocity[0]; //- Ki * integral_error[0];
    //u[1] = -Kp * pos_vector[1] - Kv * state.velocity[1]; //- Ki * integral_error[1];
    //u[2] = -Kpz * pos_vector[2] - Kvz * state.velocity[2]; //- Ki * integral_error[2];

    //u[1] = pos_error[1] * Kp + vel_error[1] * Kv;
    //u[2] = pos_error[2] * Kpz + vel_error[2] * Kvz;

    //acel[0] = -Kp * (P[0]-Pd[waypointIndex][0]) - Kv * V[0];
    //u[2] = u[2] - 9.81;
    
    // Set the controller to track the target position and attitude
    //this->controller_->set_inertial_acceleration(u, dt);
    //this->controller_->set_position(pos_vector, this->target_yaw, dt);

    //Cp[0] = 90 - final_global_drone1_ned[0];
    //Cp[1] = 10 - final_global_drone1_ned[1];
    //Cp[2] = (-23) - final_global_drone1_ned[2];

    Eigen::Vector3d Cp = target_pos - final_global_drone1_ned;

    // Calculate the magnitude of the direction vector
    double magnitude = std::sqrt(Cp[0] * Cp[0] + Cp[1] * Cp[1] + Cp[2] * Cp[2]);

    // Normalize the direction vector and scale it to a constant speed of 5 m/s
    if (magnitude > 0) { // Avoid division by zero
        velocity_[0] = (Cp[0] / magnitude) * 5.0;
        velocity_[1] = (Cp[1] / magnitude) * 5.0;
        velocity_[2] = (Cp[2] / magnitude) * 5.0;
    } else {
        velocity_[0] = 0;
        velocity_[1] = 0;
        velocity_[2] = 0;
    }

    // Send the velocity command
    this->controller_->set_inertial_velocity(velocity_, Pegasus::Rotations::rad_to_deg(yawd), dt);
}

void CoordenadasMode::waypoint_callback(const pegasus_msgs::srv::Waypoint::Request::SharedPtr request, const pegasus_msgs::srv::Waypoint::Response::SharedPtr response) {
    
    // Set the waypoint
    this->target_pos[0] = request->position[0];
    this->target_pos[1] = request->position[1];
    this->target_pos[2] = request->position[2];
    this->target_yaw = request->yaw;

    // Set the waypoint flag
    this->waypoint_set_ = true;

    // Return true to indicate that the waypoint has been set successfully
    response->success = true;
    //RCLCPP_WARN(this->node_->get_logger(), "Waypoint set to (%f, %f, %f) with yaw %f", this->target_pos[0], this->target_pos[1], this->target_pos[2], this->target_yaw);
}

void CoordenadasMode::target_pos_callback2(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

    // Update the position and velocity of the target
    //P2 = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    // Update the heading of the target
    q.x() = msg->pose.pose.orientation.x;
    q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z;
    q.w() = msg->pose.pose.orientation.w;
    yawd = Pegasus::Rotations::yaw_from_quaternion(q);

    //RCLCPP_WARN(this->node_->get_logger(), "Position (%f, %f, %f)", Pd[0], Pd[1], Pd[2]);
}


} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::CoordenadasMode, autopilot::Mode)