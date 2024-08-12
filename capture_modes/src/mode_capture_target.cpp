#include "capture_modes/mode_capture_target.hpp"

namespace autopilot {

CaptureTargetMode::~CaptureTargetMode() {}

void CaptureTargetMode::initialize() {
    RCLCPP_INFO(this->node_->get_logger(), "CaptureTargetMode initialized");
}

bool CaptureTargetMode::enter() {
    return true;
}

void CaptureTargetMode::update(double dt) {

    //State state = get_vehicle_state();
    //controller_.set_position();
    //signal_mode_finished();
}

bool CaptureTargetMode::exit() {
    return true;
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::CaptureTargetMode, autopilot::Mode)