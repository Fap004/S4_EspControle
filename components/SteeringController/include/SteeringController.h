// SteeringController.h
#pragma once
#include "IServo.h"
#include <algorithm>

class SteeringController {
public:
    SteeringController(IServo& servo, float minDeg, float maxDeg)
    : servo_(servo), minDeg_(minDeg), maxDeg_(maxDeg) {}

    void setTargetAngle(float deg) {
        targetDeg_ = std::clamp(deg, minDeg_, maxDeg_);
    }
    float targetAngle() const { return targetDeg_; }

    void update() {
        // si pas de retour capteur, écriture directe
        servo_.setAngleDeg(targetDeg_);
    }

private:
    IServo& servo_;
    float minDeg_, maxDeg_;
    float targetDeg_ = 0.0f;
};