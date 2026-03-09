#pragma once
#include <algorithm>
#include <cmath>
#include "WheelController.h"

class DriveBase
{
public:
    struct Geometry {
        float wheel_radius_m = 0.035f; // rayon roue [m] (AJUSTER)
        float track_width_m  = 0.180f; // entraxe roues [m] (AJUSTER)
        float wheel_base_m   = 0.220f; // empattement [m] (AJOUTÉ pour Ackermann/servo)
    };

    explicit DriveBase(WheelController& left,
                       WheelController& right,
                       const Geometry& geom,
                       float rpm_max = 3000.0f)
    : left_(left), right_(right), geom_(geom), rpm_max_(rpm_max) {}

    // ----- API existante : cibles RPM directes -----
    inline void setTargets(float rpmLeft, float rpmRight, float rpmMax)
    {
        float lo = -std::abs(rpmMax);
        float hi =  std::abs(rpmMax);
        float L  = std::clamp(rpmLeft,  lo, hi);
        float R  = std::clamp(rpmRight, lo, hi);
        left_.setTargetRpm(L);
        right_.setTargetRpm(R);
    }
    inline void setTargets(float rpmLeft, float rpmRight)
    {
        setTargets(rpmLeft, rpmRight, rpm_max_);
    }

    // ----- API existante : (v, ω) -----
    inline void setVW(float v_mps, float omega_radps, float wheel_radius_m, float track_width_m)
    {
        const float b    = track_width_m * 0.5f;
        const float vL   = v_mps - omega_radps * b;
        const float vR   = v_mps + omega_radps * b;
        const float wL   = (wheel_radius_m > 0.0f) ? (vL / wheel_radius_m) : 0.0f; // rad/s
        const float wR   = (wheel_radius_m > 0.0f) ? (vR / wheel_radius_m) : 0.0f; // rad/s
        const float rpmL = wL * (60.0f / (2.0f * (float)M_PI));
        const float rpmR = wR * (60.0f / (2.0f * (float)M_PI));
        setTargets(rpmL, rpmR, rpm_max_);
    }
    inline void setVW(float v_mps, float omega_radps)
    {
        setVW(v_mps, omega_radps, geom_.wheel_radius_m, geom_.track_width_m);
    }

    // ====== NOUVEAU : API “servo/ackermann” (v, angle de braquage) ======
    inline void setVSteer(float v_mps, float steer_rad)
    {
        // ω = v * tan(steer) / wheel_base
        const float wb = (geom_.wheel_base_m > 1e-6f) ? geom_.wheel_base_m : 1e-6f;
        const float omega = v_mps * std::tanf(steer_rad) / wb;
        setVW(v_mps, omega);
    }
    // ================================================================

    inline void update(float dt_s)
    {
        left_.update(dt_s);
        right_.update(dt_s);
    }

    inline void setRpmMax(float rpm_max) { rpm_max_ = std::fabs(rpm_max); }
    inline void setGeometry(const Geometry& g) { geom_ = g; }
    inline float   rpmMax()    const { return rpm_max_; }
    inline Geometry geometry() const { return geom_; }

private:
    WheelController& left_;
    WheelController& right_;
    Geometry         geom_;
    float            rpm_max_ = 3000.0f;
};