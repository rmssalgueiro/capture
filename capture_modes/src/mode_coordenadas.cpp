#include "capture_modes/mode_coordenadas.hpp"
#include "pegasus_utils/rotations.hpp"
#include <iostream>
#include <cmath>

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

    target_pos_sub2_ = node_->create_subscription<nav_msgs::msg::Odometry>("/drone2/fmu/filter/state", rclcpp::SensorDataQoS(),
    std::bind(&CoordenadasMode::target_pos_callback2, this, std::placeholders::_1));

    RCLCPP_INFO(this->node_->get_logger(), "CoordenadasMode initialized");


}

void CoordenadasMode::print_vector(const std::string& label, const double vec[3]) {
    std::cout << label << ": [" << vec[0] << ", " << vec[1] << ", " << vec[2] << "]" << std::endl;
}

// Function to convert LLA to ECEF
void CoordenadasMode::lla_to_ecef(const double lla[3], double ecef[3]) {
    double lat_rad = lla[0] * DEG2RAD;
    double lon_rad = lla[1] * DEG2RAD;
    double alt = lla[2];

    double N = a / std::sqrt(1 - e_sq * std::sin(lat_rad) * std::sin(lat_rad));

    ecef[0] = (N + alt) * std::cos(lat_rad) * std::cos(lon_rad);
    ecef[1] = (N + alt) * std::cos(lat_rad) * std::sin(lon_rad);
    ecef[2] = (N * (1 - e_sq) + alt) * std::sin(lat_rad);
}

// Function to convert ECEF to NED relative to a reference point
void CoordenadasMode::ecef_to_ned(const double ecef[3], const double ecef_ref[3], const double lla_ref[3], double ned[3]) {
    double lat_ref_rad = lla_ref[0] * DEG2RAD;
    double lon_ref_rad = lla_ref[1] * DEG2RAD;

    // Calculate the difference between ECEF coordinates and reference
    double dx = ecef[0] - ecef_ref[0];
    double dy = ecef[1] - ecef_ref[1];
    double dz = ecef[2] - ecef_ref[2];

    // Rotation matrix from ECEF to NED (transposed)
    ned[0] = -std::sin(lat_ref_rad) * std::cos(lon_ref_rad) * dx
             - std::sin(lat_ref_rad) * std::sin(lon_ref_rad) * dy
             + std::cos(lat_ref_rad) * dz;

    ned[1] = -std::sin(lon_ref_rad) * dx
             + std::cos(lon_ref_rad) * dy;

    ned[2] = -std::cos(lat_ref_rad) * std::cos(lon_ref_rad) * dx
             - std::cos(lat_ref_rad) * std::sin(lon_ref_rad) * dy
             - std::sin(lat_ref_rad) * dz;
}

// Function to generate the rotation matrix from RPY
void CoordenadasMode::rpy_to_rotation_matrix(double roll, double pitch, double yaw, double R[3][3]) {
    // Convert angles from degrees to radians
    roll = roll * DEG2RAD;
    pitch = pitch * DEG2RAD;
    yaw = yaw * DEG2RAD;

    // Rotation matrix components
    double cos_roll = std::cos(roll);
    double sin_roll = std::sin(roll);
    double cos_pitch = std::cos(pitch);
    double sin_pitch = std::sin(pitch);
    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);

    // Combined rotation matrix R = R_z(yaw) * R_y(pitch) * R_x(roll)
    R[0][0] = cos_yaw * cos_pitch;
    R[0][1] = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
    R[0][2] = cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll;
    
    R[1][0] = sin_yaw * cos_pitch;
    R[1][1] = sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;
    R[1][2] = sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll;

    R[2][0] = -sin_pitch;
    R[2][1] = cos_pitch * sin_roll;
    R[2][2] = cos_pitch * cos_roll;
}

// Function to apply the rotation matrix to the NED vector
void CoordenadasMode::apply_rotation(const double ned[3], const double R[3][3], double rotated_ned[3]) {
    rotated_ned[0] = R[0][0] * ned[0] + R[0][1] * ned[1] + R[0][2] * ned[2];
    rotated_ned[1] = R[1][0] * ned[0] + R[1][1] * ned[1] + R[1][2] * ned[2];
    rotated_ned[2] = R[2][0] * ned[0] + R[2][1] * ned[1] + R[2][2] * ned[2];
}

void CoordenadasMode::multiply_matrices(const double A[3][3], const double B[3][3], double C[3][3]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            C[i][j] = 0;
            for (int k = 0; k < 3; ++k) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
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
/*
void CoordenadasMode::inverse_apply_rotation(const double R[3][3], const double vec[3], double result[3]) {
    for (int i = 0; i < 3; ++i) {
        result[i] = 0;
        for (int j = 0; j < 3; ++j) {
            result[i] += R[j][i] * vec[j];  // Using transpose of R (inverse for rotation matrix)
        }
    }
}
*/
void CoordenadasMode::rotation_matrix_to_rpy(const double R[3][3], double& roll, double& pitch, double& yaw) {
    pitch = -std::asin(R[2][0]);

    if (std::abs(R[2][0]) < 0.99999) {
        roll = std::atan2(R[2][1], R[2][2]);
        yaw = std::atan2(R[1][0], R[0][0]);
    } else {
        roll = std::atan2(-R[1][2], R[1][1]);
        yaw = 0.0;
    }

    // Convert back to degrees
    roll *= RAD2DEG;
    pitch *= RAD2DEG;
    yaw *= RAD2DEG;
}

// Function to translate NED coordinates to a global reference frame
void CoordenadasMode::translate_to_global(const double ned[3], const double global_translation[3], double global_ned[3]) {
    global_ned[0] = ned[0] + global_translation[0];
    global_ned[1] = ned[1] + global_translation[1];
    global_ned[2] = ned[2] + global_translation[2];
}

/*
void global_to_local(const double global_pos[3], const double global_translation[3], 
                     const double R_global[3][3], const double R_local[3][3], 
                     double local_pos[3]) {
    double translated_pos[3];
    
    // Step 1: Undo global translation
    for (int i = 0; i < 3; ++i) {
        translated_pos[i] = global_pos[i] - global_translation[i];  // Undo translation
    }

    // Step 2: Undo global rotation (inverse of global rotation matrix)
    double R_global_inv[3][3];
    inverse_rotation_matrix(R_global, R_global_inv);  // Inverse of global rotation matrix

    double ned_pos[3];
    multiply_matrix_vector(R_global_inv, translated_pos, ned_pos);  // Apply inverse global rotation

    // Step 3: Undo local rotation (inverse of local rotation matrix)
    double R_local_inv[3][3];
    inverse_rotation_matrix(R_local, R_local_inv);  // Inverse of local rotation matrix

    multiply_matrix_vector(R_local_inv, ned_pos, local_pos);  // Apply inverse local rotation to get back to local frame
}
*/

void CoordenadasMode::target_gps_callback2(const pegasus_msgs::msg::SensorGps::ConstSharedPtr msg) {
    
    if (drone2_lla[0] == 0.0 && drone2_lla[1] == 0.0 && drone2_lla[2] == 0.0) {
        drone2_lla[0] = msg->latitude_deg;
        drone2_lla[1] = msg->longitude_deg;
        drone2_lla[2] = msg->altitude_msl;
        //double drone1_lla[3] = {47.397769, 8.545594, 488.05};
        //double drone2_lla[3] = {47.397742, 8.545634, 488.05};

    }
}
void CoordenadasMode::target_gps_callback(const pegasus_msgs::msg::SensorGps::ConstSharedPtr msg) {
    
    // Drone 1's position in LLA
    update_vehicle_state();

    //posição inicial LLA drone 1
    if (drone1_lla[0] == 0.0 && drone1_lla[1] == 0.0 && drone1_lla[2] == 0.0) {
        drone1_lla[0] = msg->latitude_deg;
        drone1_lla[1] = msg->longitude_deg;
        drone1_lla[2] = msg->altitude_msl;
        //double drone1_lla[3] = {47.397769, 8.545594, 488.05};
        //double drone2_lla[3] = {47.397742, 8.545634, 488.05};

    }

    //double drone1_lla[3] = {msg->latitude_deg, msg->longitude_deg, msg->altitude_msl};  // Example drone 1 position
   

    
    //double ref_lla[3] = {38.645, -8.2154, 10}; // Reference point LLA
    // Drone 1's RPY angles
    //RCLCPP_WARN(this->node_->get_logger(), "LLA (%f, %f, %f)", drone1_lla[0], drone1_lla[1], drone1_lla[2]);

    double drone1_local_position[3] = {P[0], P[1], P[2]};  // Drone has moved in local frame
    double drone2_local_position[3] = {P2[0], P2[1], P2[2]};  // Drone has moved in local frame

    // Step 1: Convert reference point and both drones' LLA positions to ECEF
    double ref_ecef[3], drone1_ecef[3], drone2_ecef[3];
    lla_to_ecef(ref_lla, ref_ecef);
    lla_to_ecef(drone1_lla, drone1_ecef);
    lla_to_ecef(drone2_lla, drone2_ecef);


    // Step 2: Convert both drones' positions from ECEF to NED
    
    double drone1_ned[3];
    ecef_to_ned(drone1_ecef, ref_ecef, ref_lla, drone1_ned);

    double drone2_ned[3];
    ecef_to_ned(drone2_ecef, ref_ecef, ref_lla, drone2_ned);

    // Step 3: Add the local position to the NED coordinates
    drone1_ned[0] += drone1_local_position[0];  // Add local x to NED north
    drone1_ned[1] += drone1_local_position[1];  // Add local y to NED east
    drone1_ned[2] += drone1_local_position[2];  // Add local z to NED down
    
    drone2_ned[0] += drone2_local_position[0];  // Add local x to NED north
    drone2_ned[1] += drone2_local_position[1];  // Add local y to NED east
    drone2_ned[2] += drone2_local_position[2];  // Add local z to NED down
    
     // Step 4: Apply drone's local RPY rotation to the NED coordinates
    double R1_local[3][3];
    double R2_local[3][3];
    rpy_to_rotation_matrix(roll, pitch, yaw, R1_local);
    rpy_to_rotation_matrix(roll2, pitch2, yaw2, R2_local);
    double rotated_drone1_ned[3];
    double rotated_drone2_ned[3];
    apply_rotation(drone1_ned, R1_local, rotated_drone1_ned);
    apply_rotation(drone2_ned, R2_local, rotated_drone2_ned);

    // Step 5: Define a global rotation matrix (for example, rotating global frame by some RPY angles)
    double global_roll = 0.0, global_pitch = 0.0, global_yaw = -90.0;
    double R_global[3][3];
    rpy_to_rotation_matrix(global_roll, global_pitch, global_yaw, R_global);

    // Apply global rotation to the NED coordinates of the drone
    double global_rotated_drone1_ned[3];
    apply_rotation(rotated_drone1_ned, R_global, global_rotated_drone1_ned);

    double global_rotated_drone2_ned[3];
    apply_rotation(rotated_drone2_ned, R_global, global_rotated_drone2_ned);

    // Step 6: Define a translation vector to the global reference frame
    double global_translation[3] = {0, 0, 0};  // Translation in meters

    // Step 7: Translate the drone to the global frame
    double final_global_drone1_ned[3];
    double final_global_drone2_ned[3];
    translate_to_global(global_rotated_drone1_ned, global_translation, final_global_drone1_ned);
    translate_to_global(global_rotated_drone2_ned, global_translation, final_global_drone2_ned);

    if(pos_ned[0] == 0.0 && pos_ned[1] == 0.0 && pos_ned[2] == 0.0) {
        pos_ned[0] = final_global_drone1_ned[0];
        pos_ned[1] = final_global_drone1_ned[1];
        pos_ned[2] = final_global_drone1_ned[2];
    }
    // Output the final global NED coordinates for the drone
    print_vector("Drone Global NED Coordinates", final_global_drone1_ned);
    print_vector("Drone Global NED Coordinates", final_global_drone2_ned);
    //RCLCPP_WARN(this->node_->get_logger(), "Waypoint set to (%f, %f, %f)", final_global_drone1_ned[0], final_global_drone1_ned[1], final_global_drone1_ned[2]);


    // Step 8: Combine the local and global rotation matrices to get the final orientation in the global frame
    double R1_combined[3][3];
    double R2_combined[3][3];
    multiply_matrices(R_global, R1_local, R1_combined);
    multiply_matrices(R_global, R2_local, R2_combined);


    // Step 9: Extract the final global RPY angles
    double global_roll1_final, global_pitch1_final, global_yaw1_final;
    double global_roll2_final, global_pitch2_final, global_yaw2_final;

    rotation_matrix_to_rpy(R1_combined, global_roll1_final, global_pitch1_final, global_yaw1_final);
    rotation_matrix_to_rpy(R2_combined, global_roll2_final, global_pitch2_final, global_yaw2_final);

    
    // Output the final global RPY orientation
    std::cout << "Drone Global RPY:" << std::endl;
    std::cout << "Roll: " << global_roll1_final <<","<< global_roll2_final<< " degrees" << std::endl;
    std::cout << "Pitch: " << global_pitch1_final << "," << global_pitch2_final <<" degrees" << std::endl;
    std::cout << "Yaw: " << global_yaw1_final << "," << global_pitch2_final << " degrees" << std::endl;
    
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

    Kp = 1;
	Kv = 2;
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

    // Compute the position error and velocity error using the path desired position and velocity
    Eigen::Vector3d pos_vector = target_pos - pos_ned;
    //RCLCPP_WARN(this->node_->get_logger(), "Position (%f, %f, %f)", pos_vector[0], pos_vector[1], pos_vector[2]);

    //Eigen::Vector3d vel_error = target_velocity - state.velocity;

    // Compute the desired control output acceleration for each controller
    u[0] = - Kp * pos_vector[0] - Kv * state.velocity[0];
    u[1] = - Kp * pos_vector[1] - Kv * state.velocity[1];
    u[2] = - Kpz * pos_vector[2] - Kvz * state.velocity[2];
    //u[1] = pos_error[1] * Kp + vel_error[1] * Kv;
    //u[2] = pos_error[2] * Kpz + vel_error[2] * Kvz;

    //acel[0] = -Kp * (P[0]-Pd[waypointIndex][0]) - Kv * V[0];
    //u[2] = u[2] - 9.81;
    
    // Set the controller to track the target position and attitude
    //this->controller_->set_inertial_acceleration(u, dt);
    this->controller_->set_position(pos_vector, this->target_yaw, dt);

    //RCLCPP_WARN(this->node_->get_logger(), "Coordinates (%f, %f, %f)", global_drone1_ned[0], global_drone1_ned[1], global_drone1_ned[2]);

    //this->controller_->set_inert (this->target_pos, this->target_yaw, dt);
    //RCLCPP_WARN(this->node_->get_logger(), "Waypoint set to (%f, %f, %f)", u[0], u[1], u[2]);RCLCPP_WARN
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
    P2 = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

}
void CoordenadasMode::update_vehicle_state() {

    // Get the current state of the vehicle
    State state = get_vehicle_state();

    // Update the MPC state
    P = state.position;
    //V = state.velocity;
    //yaw = Pegasus::Rotations::yaw_from_quaternion(state.attitude);
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::CoordenadasMode, autopilot::Mode)