#pragma once

#include <autopilot/mode.hpp>
#include "pegasus_msgs/srv/waypoint.hpp"

namespace autopilot {

class WaypointModeAcc : public autopilot::Mode {

public:

    ~WaypointModeAcc();

    void initialize() override;
    bool enter() override;
    bool exit() override;
    void update(double dt) override;

    int Kp = 1;
	int Kv = 2;
	int Kpz = 1;
	int Kvz = 3;

    Eigen::Vector3d u{Eigen::Vector3d::Zero()};
    Eigen::Vector3d target_pos{Eigen::Vector3d::Zero()};


protected:

    // The waypoint service callback
    void waypoint_callback(const pegasus_msgs::srv::Waypoint::Request::SharedPtr request, const pegasus_msgs::srv::Waypoint::Response::SharedPtr response);

    // Check if the waypoint is already set
    bool waypoint_set_{false};

    // The target position and attitude waypoint to be at
    float target_yaw{0.0f};

    // The waypoint service server that sets the position and attitude waypoints at a given target
    rclcpp::Service<pegasus_msgs::srv::Waypoint>::SharedPtr waypoint_service_{nullptr};
};

}