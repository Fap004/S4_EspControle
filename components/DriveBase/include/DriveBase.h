// DriveBase.h
#pragma once
#include "WheelController.h"
#include "SteeringController.h"

class DriveBase {
public:
    DriveBase(WheelController& left, WheelController& right,
              SteeringController& steer)
    : left_(left), right_(right), steer_(steer) {}

    // Interface haut niveau
    void setTargets(float rpmLeft, float rpmRight, float steerDeg) {
        left_.setTargetRPM(rpmLeft);
        right_.setTargetRPM(rpmRight);
        steer_.setTargetAngle(steerDeg);
    }

    // Pour commandes différentielle (v forward, w rotation)
    void setVW(float v_mps, float w_radps, float wheelRadius_m, float axleTrack_m) {
        float vL = v_mps - (w_radps * axleTrack_m / 2.0f);
        float vR = v_mps + (w_radps * axleTrack_m / 2.0f);
        float rpmL = (vL / (2.0f * 3.1415926f * wheelRadius_m)) * 60.0f;
        float rpmR = (vR / (2.0f * 3.1415926f * wheelRadius_m)) * 60.0f;
        left_.setTargetRPM(rpmL);
        right_.setTargetRPM(rpmR);
    }

    void update(float dt_wheels, bool updateSteer = true) {
        left_.update(dt_wheels);
        right_.update(dt_wheels);
        if (updateSteer) steer_.update();
    }

private:
    WheelController&     left_;
    WheelController&     right_;
    SteeringController&  steer_;
};