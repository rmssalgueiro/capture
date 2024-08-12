#include "pegasus_utils/rotations.hpp"
#include "capture_modes/mode_capture_target.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace autopilot {

CaptureTargetMode::~CaptureTargetMode() {}

void CaptureTargetMode::initialize() {

    // Initialize the target state subscribers
    node_->declare_parameter<std::string>("autopilot.CaptureTargetMode.target_state_topic", "target_state"); 
    // target_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    //     node_->get_parameter("target_filter.CaptureTargetMode.target_state_topic").as_string(), 
    //     rclcpp::SensorDataQoS(), 
    //     std::bind(&CaptureTargetMode::target_state_callback, this, std::placeholders::_1)
    // );

    // Load the gains of the controller
    node_->declare_parameter<double>("autopilot.CaptureTargetMode.gains.Kp", 1.0);
    node_->declare_parameter<double>("autopilot.CaptureTargetMode.gains.Kv", 1.0);
    node_->declare_parameter<double>("autopilot.CaptureTargetMode.gains.Kpz", 1.0);
    node_->declare_parameter<double>("autopilot.CaptureTargetMode.gains.Kvz", .0);
    Kp = node_->get_parameter("autopilot.CaptureTargetMode.gains.Kp").as_double();
    Kv = node_->get_parameter("autopilot.CaptureTargetMode.gains.Kv").as_double();
    Kpz = node_->get_parameter("autopilot.CaptureTargetMode.gains.Kpz").as_double();
    Kvz = node_->get_parameter("autopilot.CaptureTargetMode.gains.Kvz").as_double();

    // Initialize the MPC library
    std::string file_name = "gen";
    std::string package_name = "capture_modes";
    std::string prefix_lib = ament_index_cpp::get_package_share_directory(package_name) + "/include/";

    // Create a new NLP solver instance from the compiled code
    std::string lib_name = prefix_lib + file_name + ".so";

    // Use CasADi's "external" to load the compiled function
    mpc_controller_ = casadi::external("F",lib_name);

    RCLCPP_INFO(this->node_->get_logger(), "CaptureTargetMode initialized");
}

bool CaptureTargetMode::enter() {
    return true;
}

void CaptureTargetMode::update(double dt) {

    // Get the current state of the vehicle
    update_vehicle_state();

    // Check if we need to change MPC on
    if((Pd[0] + 0.7 > 28 && Pd[0] - 0.7 < 28) && (Pd[1] + 0.7 > 5 && Pd[1] - 0.7 < 5)) {
	   operation_mode_ = OperationMode::MPC_ON;
    }

    // Apply the correct control mode
    if (operation_mode_ == OperationMode::MPC_ON) {
        mode_mpc_on();
    } else {
        mode_mpc_off();
    }

    // Make the controller track the reference
    this->controller_->set_inertial_velocity(velocity_, Pegasus::Rotations::rad_to_deg(yawd), dt);
}

void CaptureTargetMode::mode_mpc_on() {

    // Create the state vector for the MPC
    x0 = casadi::DM::vertcat({P(0), P(1), P(2), V(0), V(1), V(2), yaw, Pd(0), Pd(1), Pd(2), Vd(0), Vd(1), Vd(2), yawd});
    
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

    casadi::Matrix<double> result_matrix = res.at(1);

    // Set the velocity input to apply to the vehicle
    velocity_[0] = static_cast<double>(result_matrix(3,1));
    velocity_[1] = static_cast<double>(result_matrix(4,1));
    velocity_[2] = static_cast<double>(result_matrix(5,1));
}

void CaptureTargetMode::mode_mpc_off() {
    
    // TODO - explain what is going on here...
    Kp=0.5;

    Eigen::Vector3d Cp;
    Cp[0] = 35 - P[0];
    Cp[1] = 5 - P[1];
    Cp[2] = (-15) - P[2];

    velocity_[0] = Kp * Cp[0];
    velocity_[1] = Kp * Cp[1];
    velocity_[2] = Kp * Cp[2];
}

bool CaptureTargetMode::exit() {
    return true;
}


void CaptureTargetMode::target_state_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    // Update the position and velocity of the target
    Pd = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Vd = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);

    // Update the heading of the target
    Eigen::Quaterniond q;
    q.x() = msg->pose.pose.orientation.x;
    q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z;
    q.w() = msg->pose.pose.orientation.w;
    yawd = Pegasus::Rotations::yaw_from_quaternion(q); 
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