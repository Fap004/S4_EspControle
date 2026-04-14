#pragma once
#include "McpwmServo.h"
#include <algorithm>
#include <atomic>

class SteeringController {
public:
    SteeringController(McpwmServo& servo, float minDeg, float maxDeg)
    : servo_(servo), minDeg_(minDeg), maxDeg_(maxDeg) {}

    void enable() {
        enabled_.store(true, std::memory_order_release);
        servo_.setAngleDeg(targetDeg_);  // reprend à l'angle courant
    }

    void disable() {
        enabled_.store(false, std::memory_order_release);
        servo_.holdNeutral();  // ✅ fige à 1500µs, servo au centre
    }

    bool isEnabled() const { return enabled_.load(std::memory_order_acquire); }

    void setTargetAngle(float deg) {
        targetDeg_ = std::clamp(deg, minDeg_, maxDeg_);
    }
    float targetAngle() const { return targetDeg_; }

    void update() {
        if (!enabled_.load(std::memory_order_acquire)) return;
        servo_.setAngleDeg(targetDeg_);
    }

private:
    McpwmServo&       servo_;
    float             minDeg_, maxDeg_;
    float             targetDeg_ = 0.0f;
    std::atomic<bool> enabled_   { false };
};