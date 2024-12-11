#include "pegasus_utils/rotations.hpp"
#include "capture_modes/mode_capture_target.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <chrono>
#include <capture_msgs/msg/capture.hpp>



namespace autopilot {

CaptureTargetMode::~CaptureTargetMode() {}

void CaptureTargetMode::initialize() {

    target_sub_ = node_->create_subscription<capture_msgs::msg::Capture>(
        "/drone2/capture/status", 
        rclcpp::SensorDataQoS(), 
        std::bind(&CaptureTargetMode::target_state_callback, this, std::placeholders::_1));
    
    target_state_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>("/drone2/fmu/filter/state", rclcpp::SensorDataQoS(), 
        std::bind(&CaptureTargetMode::target_state_pegasus_callback, this, std::placeholders::_1));
    
    target_gps_sub_ = node_->create_subscription<pegasus_msgs::msg::SensorGps>("/drone1/fmu/sensors/gps", rclcpp::SensorDataQoS(), 
        std::bind(&CaptureTargetMode::target_gps_callback, this, std::placeholders::_1));

    //node_->declare_parameter<std::string>("capture.publishers.status", "capture/status");
    publisher_ = this->node_->create_publisher<capture_msgs::msg::Capture>(
        this->node_->get_parameter("capture.publishers.status").as_string(), rclcpp::SensorDataQoS());


    // Load the gains of the controller
    node_->declare_parameter<double>("autopilot.CaptureTargetMode.gains.Kp", 3.0);
    node_->declare_parameter<double>("autopilot.CaptureTargetMode.gains.Kv", 5.0);
    node_->declare_parameter<double>("autopilot.CaptureTargetMode.gains.Kpz", 2.0);
    node_->declare_parameter<double>("autopilot.CaptureTargetMode.gains.Kvz", 3.0);
    Kp = node_->get_parameter("autopilot.CaptureTargetMode.gains.Kp").as_double();
    Kv = node_->get_parameter("autopilot.CaptureTargetMode.gains.Kv").as_double();
    Kpz = node_->get_parameter("autopilot.CaptureTargetMode.gains.Kpz").as_double();
    Kvz = node_->get_parameter("autopilot.CaptureTargetMode.gains.Kvz").as_double();

    RCLCPP_INFO(this->node_->get_logger(), "CaptureTargetMode Kp: %f", Kp);
    RCLCPP_INFO(this->node_->get_logger(), "CaptureTargetMode Kv: %f", Kv);
    RCLCPP_INFO(this->node_->get_logger(), "CaptureTargetMode Kpz: %f", Kpz);
    RCLCPP_INFO(this->node_->get_logger(), "CaptureTargetMode Kvz: %f", Kvz);

    // Initialize the MPC library
    compile_mpc_controller();

    RCLCPP_INFO(this->node_->get_logger(), "CaptureTargetMode initialized");
}


void CaptureTargetMode::compile_mpc_controller() {

    std::string file_name = "gen";
    std::string lib_path = ament_index_cpp::get_package_share_directory("capture_modes");

    // Remove the unnecessary part of the path
    std::string toErase = "/share/capture_modes";
    size_t pos = lib_path.find(toErase);
    lib_path.erase(pos, toErase.length());

    // Add the path to the .so file
    lib_path = lib_path + "/lib/libgen.so";

    // Check if the .so file exists
    std::ifstream file(lib_path);
    if (file.good()) {

        // Use CasADi's "external" to load the compiled function
        mpc_controller_ = casadi::external("F", lib_path);
        RCLCPP_INFO(this->node_->get_logger(), "MPC controller already compiled");

    } else {
        throw std::runtime_error("MPC controller not compiled - libgen.so not found");
    } 

}

void CaptureTargetMode::print_vector(const std::string& label, const Eigen::Vector3d& vec) {
    std::cout << label << ": [" << vec[0] << ", " << vec[1] << ", " << vec[2] << "]" << std::endl;
}

// Function to convert LLA to ECEF

void CaptureTargetMode::lla_to_ecef(const Eigen::Vector3d &lla, Eigen::Vector3d &ecef) {
    const double lat_rad = lla[0] * DEG2RAD;
    const double lon_rad = lla[1] * DEG2RAD;
    const double alt = lla[2];

    const double N = a / std::sqrt(1 - e_sq * std::sin(lat_rad) * std::sin(lat_rad));

    ecef[0] = (N + alt) * std::cos(lat_rad) * std::cos(lon_rad);
    ecef[1] = (N + alt) * std::cos(lat_rad) * std::sin(lon_rad);
    ecef[2] = (N * (1 - e_sq) + alt) * std::sin(lat_rad);
}

// Function to convert ECEF to NED relative to a reference poin
void CaptureTargetMode::ecef_to_ned(const Eigen::Vector3d &ecef, const Eigen::Vector3d &ecef_ref, const Eigen::Vector3d &lla_ref, Eigen::Vector3d &ned) {
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
void CaptureTargetMode::rpy_to_rotation_matrix(double roll, double pitch, double yaw, Eigen::Matrix3d &R) {
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

void CaptureTargetMode::quaternion_to_rotation_matrix(double q_w, double q_x, double q_y, double q_z, Eigen::Matrix3d &R) {
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
void CaptureTargetMode::apply_rotation(const Eigen::Vector3d &ned, const Eigen::Matrix3d &R, Eigen::Vector3d &rotated_ned) {
    rotated_ned = R * ned;  // Eigen handles the matrix-vector multiplication internally
}

void CaptureTargetMode::multiply_matrices(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B, Eigen::Matrix3d& C) {
    C = A * B;  // Eigen handles matrix multiplication natively
}

void CaptureTargetMode::multiply_matrix_vector(const double R[3][3], const double vec[3], double result[3]) {
    for (int i = 0; i < 3; ++i) {
        result[i] = 0;
        for (int j = 0; j < 3; ++j) {
            result[i] += R[i][j] * vec[j];
        }
    }
}

// Function to compute the inverse of a 3x3 rotation matrix (for orthogonal matrices, it's simply the transpose)
void CaptureTargetMode::inverse_rotation_matrix(const double, double R[3][3], double R_inv[3][3]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R_inv[i][j] = R[j][i];  // Transpose of R
        }
    }
}


void CaptureTargetMode::rotation_matrix_to_rpy(const Eigen::Matrix3d& R, double& roll, double& pitch, double& yaw) {
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
void CaptureTargetMode::translate_to_global(const Eigen::Vector3d &ned, const Eigen::Vector3d &global_translation, Eigen::Vector3d &global_ned) {
    global_ned = ned + global_translation;
}

bool CaptureTargetMode::enter() {
    return true;
}

void CaptureTargetMode::update(double dt) {

    // Get the current state of the vehicle
    State state = get_vehicle_state();
    yaw = Pegasus::Rotations::yaw_from_quaternion(state.attitude);
    
    V = state.velocity;

    q_global.x() = 0.0;
    q_global.y() = 0.0;
    q_global.z() = 0.0;//- sqrt(2.0) / 2.0;
    q_global.w() = 1.0;//sqrt(2.0) / 2.0;
    
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

    // Step 4: Apply drone's local RPY rotation to the NED coordinates
    //quaternion_to_rotation_matrix(state.attitude.w(), state.attitude.x(), state.attitude.y(), state.attitude.z(), R2_local);
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
    //rotation_matrix_to_rpy(R2_local, global_roll1_final, global_pitch1_final, global_yaw1_final);
    //rotation_matrix_to_rpy(R2_combined, global_roll2_final, global_pitch2_final, global_yaw2_final);
    
    capture_msg.global_pos_shuttle[0] = final_global_drone1_ned[0];
    capture_msg.global_pos_shuttle[1] = final_global_drone1_ned[1];
    capture_msg.global_pos_shuttle[2] = final_global_drone1_ned[2];
    capture_msg.global_pos_target[0] = final_global_drone2_ned[0];
    capture_msg.global_pos_target[1] = final_global_drone2_ned[1];
    capture_msg.global_pos_target[2] = final_global_drone2_ned[2];

    publisher_->publish(capture_msg);

    // Check if we need to change MPC on
    //Pinform
    if((final_global_drone2_ned[0] + 5 > -24.29 && final_global_drone2_ned[0] - 5 < -24.29) && (final_global_drone2_ned[1] + 5 > 37.41 && final_global_drone2_ned[1] - 5 < 37.41)) {
	   operation_mode_ = OperationMode::MPC_ON;
    }
    // Apply the correct control mode
    if (operation_mode_ == OperationMode::MPC_ON) {

        if(counter==10){
            mode_mpc_on();
            //The MPC was made to work at 5 Hz(200ms), so we need to call it every 10 iterations, because the update function is called at 50 Hz(20ms).
            //this->controller_->set_inertial_acceleration(acel_, dt);
            this->controller_->set_inertial_velocity(velocity_, Pegasus::Rotations::rad_to_deg(yaw_input), dt);

            capture_msg.acel[0] = velocity_[0];
            capture_msg.acel[1] = velocity_[1];
            capture_msg.acel[2] = velocity_[2];

            publisher_->publish(capture_msg);
            
            counter=0;
        }
        counter++;

    } else {
        mode_mpc_off();
        this->controller_->set_inertial_velocity(velocity_, 0, dt);

    }
}

bool CaptureTargetMode::check_finished() {

    //if((P[0] + 1 > 0 && P[0] - 1 < 0) && (P[1] + 1 > 10 && P[1] - 1 < 10)) {
    if((std::abs(final_global_drone1_ned[2] - final_global_drone2_ned[2]) <  0.8) && (operation_mode_ == OperationMode::MPC_ON)){// > because NED referencial
        //signal_mode_finished();
        operation_mode_ = OperationMode::MPC_OFF;
        catched = true;
        RCLCPP_INFO_STREAM(node_->get_logger(), "Capture finished.");
        return true;
        }
    
    
    return false;
}

void CaptureTargetMode::mode_mpc_on() {


    // Create the state vector for the MPC
    x0 = casadi::DM::vertcat({final_global_drone1_ned[0], final_global_drone1_ned[1], final_global_drone1_ned[2], V(0), V(1), V(2), yaw, final_global_drone2_ned[0], final_global_drone2_ned[1], final_global_drone2_ned[2], Vd(0), Vd(1), Vd(2), yawd});
    //x0 = casadi::DM::vertcat({final_global_drone1_ned[0], final_global_drone1_ned[1], final_global_drone1_ned[2], V(0), V(1), V(2), yaw, A[0], A[1], A[2], B[0], B[1], B[2], yawd});
    RCLCPP_WARN(this->node_->get_logger(), "x0: (%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f)", final_global_drone1_ned[0], final_global_drone1_ned[1], final_global_drone1_ned[2], V(0), V(1), V(2), final_global_drone2_ned[0], final_global_drone2_ned[1], final_global_drone2_ned[2], Vd(0), Vd(1), Vd(2));

    // Create the input vector for the MPC
    std::vector<casadi::DM> arg1 = {x0, uu, xx};

    auto start = std::chrono::high_resolution_clock::now();
    // Call the MPC controller
    std::vector<casadi::DM> res = mpc_controller_(arg1);
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    capture_msg.computation_time = duration.count();
    // Get the first input of the MPC
    casadi::Matrix<double> result_xx = res.at(1);
    casadi::Matrix<double> result_uu = res.at(0);



    // Update the state and input vectors
    xx = result_xx;
    uu = result_uu;
    //uu_mpc = result_uu_mpc;

    casadi::Matrix<double> result_matrix = res.at(1);

    // Set the velocity input to apply to the vehicle
    
    velocity_[0] = static_cast<double>(result_matrix(3,1));
    velocity_[1] = static_cast<double>(result_matrix(4,1));
    velocity_[2] = static_cast<double>(result_matrix(5,1));
    yaw_input = static_cast<double>(result_matrix(6,1));

    //acel_[0] = static_cast<double>(result_matrix(0,0));
    //acel_[1] = static_cast<double>(result_matrix(1,0));
    //acel_[2] = static_cast<double>(result_matrix(2,0));
    

    //RCLCPP_WARN(this->node_->get_logger(), "acc set to (%f, %f, %f)", acel_[0], acel_[1], acel_[2]);
    check_finished();
}

void CaptureTargetMode::mode_mpc_off() {
    
    // TODO - explain what is going on here...
    Kp=0.5;

    Eigen::Vector3d Cp;

    if(catched){
        //Pfinal
        Cp[0] = 10 - final_global_drone1_ned[0];
        Cp[1] = 10 - final_global_drone1_ned[1];
        Cp[2] = (-13) - final_global_drone1_ned[2];
    }
    else{
        //Pwait
        Cp[0] = -41.76 - final_global_drone1_ned[0];
        Cp[1] = 43.78 - final_global_drone1_ned[1];
        Cp[2] = (-13) - final_global_drone1_ned[2];
    }

    velocity_[0] = Kp * Cp[0];
    velocity_[1] = Kp * Cp[1];
    velocity_[2] = Kp * Cp[2];

}

bool CaptureTargetMode::exit() {
    return true;
}

void CaptureTargetMode::target_state_callback(const capture_msgs::msg::Capture::ConstSharedPtr msg) {

    final_global_drone2_ned[0] = msg->global_pos_target[0];
    final_global_drone2_ned[1] = msg->global_pos_target[1];
    final_global_drone2_ned[2] = msg->global_pos_target[2];
}

void CaptureTargetMode::target_state_pegasus_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

    Vd = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    
    Eigen::Quaterniond q;
    q.x() = msg->pose.pose.orientation.x;
    q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z;
    q.w() = msg->pose.pose.orientation.w;
    yawd = Pegasus::Rotations::yaw_from_quaternion(q);
}

void CaptureTargetMode::target_gps_callback(const pegasus_msgs::msg::SensorGps::ConstSharedPtr msg) {

    //posição inicial LLA drone 1
    if (drone1_lla[0] == 0.0 && drone1_lla[1] == 0.0 && drone1_lla[2] == 0.0) {
        //drone1_lla[0] = msg->latitude_deg;
        //drone1_lla[1] = msg->longitude_deg;
        //drone1_lla[2] = msg->altitude_msl;
    }     
}
} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::CaptureTargetMode, autopilot::Mode)