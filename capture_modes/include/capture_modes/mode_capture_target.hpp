#pragma once

#include <autopilot/mode.hpp>

namespace autopilot {

class CaptureTargetMode : public autopilot::Mode {

public:

    ~CaptureTargetMode();

    void initialize() override;
    virtual bool enter();
    virtual bool exit() override;
    virtual void update(double dt);

protected:

    std::vector<double> waypoints_;
};
}