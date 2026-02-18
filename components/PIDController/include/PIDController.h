// PIDController.h
#pragma once
#include <algorithm>

class PIDController {
public:
    PIDController(float kp, float ki, float kd,
                  float outMin, float outMax)
        : kp_(kp), ki_(ki), kd_(kd),
          outMin_(outMin), outMax_(outMax) {}

    void setTunings(float kp, float ki, float kd) {
        kp_ = kp; ki_ = ki; kd_ = kd;
    }
    void setOutputLimits(float minV, float maxV) {
        outMin_ = minV; outMax_ = maxV;
    }
    void reset() { integ_ = 0; prevErr_ = 0; first_ = true; }

    float compute(float setpoint, float measured, float dt) {
        float err = setpoint - measured;
        integ_ += ki_ * err * dt;
        // anti-windup (pre-clamp)
        integ_ = std::clamp(integ_, outMin_, outMax_);
        float deriv = first_ ? 0.f : (err - prevErr_) / dt;
        first_ = false;
        float out = kp_ * err + integ_ + kd_ * deriv;
        // output clamp
        out = std::clamp(out, outMin_, outMax_);
        prevErr_ = err;
        return out;
    }

private:
    float kp_, ki_, kd_;
    float outMin_, outMax_;
    float integ_ = 0.f;
    float prevErr_ = 0.f;
    bool  first_ = true;
};