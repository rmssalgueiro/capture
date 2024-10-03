#include "capture_modes/mode_waypoint_acc.hpp"

namespace autopilot {

WaypointModeAcc::~WaypointModeAcc() {
    // Terminate the waypoint service
    this->waypoint_service_.reset();
}

void WaypointModeAcc::initialize() {
    
    // Create the waypoint service server
    node_->declare_parameter<std::string>("autopilot.WaypointModeAcc.set_waypoint_service", "set_waypoint"); 
    this->waypoint_service_ = this->node_->create_service<pegasus_msgs::srv::Waypoint>(node_->get_parameter("autopilot.WaypointModeAcc.set_waypoint_service").as_string(), std::bind(&WaypointModeAcc::waypoint_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->node_->get_logger(), "WaypointModeAcc initialized");
}

bool WaypointModeAcc::enter() {

    Kp = 1;
	Kv = 2;
	Kpz = 1;
	Kvz = 3;
    // Return true to indicate that the mode has been entered successfully
    return true;
}

bool WaypointModeAcc::exit() {
    
    // Nothing to do here
    return true;   // Return true to indicate that the mode has been exited successfully
}

void WaypointModeAcc::update(double dt) {

    // Get the current state of the vehicle
    State state = get_vehicle_state();

    // Compute the position error and velocity error using the path desired position and velocity
    Eigen::Vector3d pos_error = state.position - target_pos;
    //Eigen::Vector3d vel_error = target_velocity - state.velocity;

    // Compute the desired control output acceleration for each controller
    u[0] = - Kp * pos_error[0] - Kv * state.velocity[0];
    u[1] = - Kp * pos_error[1] - Kv * state.velocity[1];
    u[2] = - Kpz * pos_error[2] - Kvz * state.velocity[2];
    //u[1] = pos_error[1] * Kp + vel_error[1] * Kv;
    //u[2] = pos_error[2] * Kpz + vel_error[2] * Kvz;

    //acel[0] = -Kp * (P[0]-Pd[waypointIndex][0]) - Kv * V[0];
    //u[2] = u[2] - 9.81;
    
    // Set the controller to track the target position and attitude
    this->controller_->set_inertial_acceleration(u, dt);
    //this->controller_->set_inert (this->target_pos, this->target_yaw, dt);
    RCLCPP_WARN(this->node_->get_logger(), "Waypoint set to (%f, %f, %f)", u[0], u[1], u[2]);
}

void WaypointModeAcc::waypoint_callback(const pegasus_msgs::srv::Waypoint::Request::SharedPtr request, const pegasus_msgs::srv::Waypoint::Response::SharedPtr response) {
    
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

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::WaypointModeAcc, autopilot::Mode)