// WheelController.h
#pragma once
#include "IMotorDriver.h"
#include "IEncoder.h"
#include "PIDController.h"
#include <algorithm> // std::min, std::clamp

class WheelController {
public:
    WheelController(IMotorDriver& driver, IEncoder& enc,
                    float maxRpm, float kp, float ki, float kd)
    : driver_(driver), enc_(enc),
      pid_(kp, ki, kd, -1.0f, 1.0f),
      maxRpm_(maxRpm) {}

    void setTargetRPM(float rpm) { targetRpm_ = rpm; }
    float targetRPM() const { return targetRpm_; }
    float measuredRPM() const { return lastRpm_; }

    void update(float dt) {
        // Mesure RPM
        int32_t dTicks = enc_.getAndClearDeltaTicks();
        float rev = static_cast<float>(dTicks) / enc_.ticksPerRev();
        lastRpm_ = (rev / dt) * 60.0f;

        // Direction + amplitude via PID
        float u = pid_.compute(targetRpm_, lastRpm_, dt); // -1..+1
        MotorDir dir = (u >= 0) ? MotorDir::Forward : MotorDir::Reverse;
        float duty = std::min(1.0f, std::abs(u));

        // Sécurité petites commandes (deadzone)
        if (std::abs(targetRpm_) < 1.0f) { duty = 0.0f; }

        driver_.setDirection(dir);
        driver_.setDuty(duty);
    }

    void stop() {
        driver_.setDuty(0.0f);
    }

private:
    IMotorDriver&  driver_;
    IEncoder&      enc_;
    PIDController  pid_;
    float          maxRpm_;
    float          targetRpm_ = 0.0f;
    float          lastRpm_   = 0.0f;
};
