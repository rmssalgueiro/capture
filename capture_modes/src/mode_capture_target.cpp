#include "pegasus_utils/rotations.hpp"
#include "capture_modes/mode_capture_target.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace autopilot {

CaptureTargetMode::~CaptureTargetMode() {}

void CaptureTargetMode::initialize() {
    /*
    // Initialize the target state subscribers
    node_->declare_parameter<std::string>("autopilot.CaptureTargetMode.target_state_topic", "target_state"); 
    
    target_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        node_->get_parameter("autopilot.CaptureTargetMode.target_state_topic").as_string(), 
        rclcpp::SensorDataQoS(), 
        std::bind(&CaptureTargetMode::target_state_callback, this, std::placeholders::_1)
    );
    */
    target_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/drone2/fmu/filter/state", 
        rclcpp::SensorDataQoS(), 
        std::bind(&CaptureTargetMode::target_state_callback, this, std::placeholders::_1)
    );

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

bool CaptureTargetMode::enter() {
    return true;
}

void CaptureTargetMode::update(double dt) {

    // Get the current state of the vehicle
    update_vehicle_state();

    // Check if we need to change MPC on
    //Pinform
    if((Pd[0] + 5 > 80 && Pd[0] - 5 < 80) && (Pd[1] + 1 > 10 && Pd[1] - 1 < 10)) {
	   operation_mode_ = OperationMode::MPC_ON;
    }
    // Apply the correct control mode
    if (operation_mode_ == OperationMode::MPC_ON) {

        

        if(counter==10){
            mode_mpc_on();
            //The MPC was made to work at 5 Hz(200ms), so we need to call it every 10 iterations, because the update function is called at 50 Hz(20ms).
            // Make the controller track the reference
            //this->controller_->set_inertial_velocity(velocity_, Pegasus::Rotations::rad_to_deg(yawd), dt);
            this->controller_->set_inertial_acceleration(acel_, dt);
            counter=0;
        }
        counter++;

    } else {
        mode_mpc_off();
        this->controller_->set_inertial_velocity(velocity_, Pegasus::Rotations::rad_to_deg(yawd), dt);

    }
}

bool CaptureTargetMode::check_finished() {
    
    update_vehicle_state();

    //if((P[0] + 1 > 0 && P[0] - 1 < 0) && (P[1] + 1 > 10 && P[1] - 1 < 10)) {
    if((P[2] - Pd[2] > - 0.5) && (operation_mode_ == OperationMode::MPC_ON)){// > because NED referencial
        signal_mode_finished();
        RCLCPP_INFO_STREAM(node_->get_logger(), "Capture finished.");
        return true;
        }
    
    
    return false;
}

void CaptureTargetMode::mode_mpc_on() {

    
    // Create the state vector for the MPC
    x0 = casadi::DM::vertcat({P(0), P(1), P(2), V(0), V(1), V(2), yaw, Pd2(0), Pd2(1), Pd2(2), Vd(0), Vd(1), Vd(2), yawd});
    RCLCPP_WARN(this->node_->get_logger(), "x0: (%f, %f, %f, %f, %f, %f)", P(0), P(1), P(2), Pd2(0), Pd2(1), Pd2(2));

    // Create the input vector for the MPC
    std::vector<casadi::DM> arg1 = {x0, uu, xx};

    // Call the MPC controller
    std::vector<casadi::DM> res = mpc_controller_(arg1);

    // Get the first input of the MPC
    casadi::Matrix<double> result_xx = res.at(1);
    casadi::Matrix<double> result_uu = res.at(0);

    // Update the state and input vectors
    xx = result_xx;
    uu = result_uu;

    casadi::Matrix<double> result_matrix = res.at(0);

    // Set the velocity input to apply to the vehicle
    /*
    velocity_[0] = static_cast<double>(result_matrix(3,1));
    velocity_[1] = static_cast<double>(result_matrix(4,1));
    velocity_[2] = static_cast<double>(result_matrix(5,1));
    */
    acel_[0] = static_cast<double>(result_matrix(0,1));
    acel_[1] = static_cast<double>(result_matrix(1,1));
    acel_[2] = static_cast<double>(result_matrix(2,1));

    //RCLCPP_WARN(this->node_->get_logger(), "acc set to (%f, %f, %f)", acel_[0], acel_[1], acel_[2]);
    check_finished();
}

void CaptureTargetMode::mode_mpc_off() {
    //Pwait
    // TODO - explain what is going on here...
    Kp=0.5;

    Eigen::Vector3d Cp;
    Cp[0] = 90 - P[0];
    Cp[1] = 10 - P[1];
    Cp[2] = (-23) - P[2];

    velocity_[0] = Kp * Cp[0];
    velocity_[1] = Kp * Cp[1];
    velocity_[2] = Kp * Cp[2];

}

bool CaptureTargetMode::exit() {
    return true;
}

void CaptureTargetMode::target_state_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

    // Update the position and velocity of the target
    Pd = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Pd2[0] = Pd[0]-3;
    Pd2[1] = Pd[1];
    Pd2[2] = Pd[2];

    Vd = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);

    // Update the heading of the target
    Eigen::Quaterniond q;
    q.x() = msg->pose.pose.orientation.x;
    q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z;
    q.w() = msg->pose.pose.orientation.w;
    yawd = Pegasus::Rotations::yaw_from_quaternion(q);

    //RCLCPP_WARN(this->node_->get_logger(), "Position (%f, %f, %f)", Pd[0], Pd[1], Pd[2]);

}

void CaptureTargetMode::update_vehicle_state() {

    // Get the current state of the vehicle
    State state = get_vehicle_state();

    // Update the MPC state
    P = state.position;
    V = state.velocity;
    yaw = Pegasus::Rotations::yaw_from_quaternion(state.attitude);
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::CaptureTargetMode, autopilot::Mode)