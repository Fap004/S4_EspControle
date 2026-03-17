#pragma once
#include <algorithm>
#include <cmath>
#include "esp_log.h"
#include "IMotorDriver.h"
#include "PIDController.h"
#include "PcntEncoder.h"

class WheelController
{
public:
    struct Config {
        float rpm_max         = 8000.0f; // plafond consigne (met 8000 si tu veux tester >3000)
        float lp_alpha        = 0.30f;   // lissage mesure RPM (0..1)
        float ticks_per_rev   = 68.0f;   // 17 PPR x4 = 68 (si enc.ticksPerRev n’existe pas)
        bool  use_encoder_tpr = true;    // lire le TPR depuis l’encodeur si dispo
        float duty_min        = 0.10f;   // dead-zone soft (8–12 % typ.)
    };

    WheelController(IMotorDriver& motor, PcntEncoder& enc, PIDController& pid, const Config& cfg)
    : motor_(motor), enc_(enc), pid_(pid), cfg_(cfg) {}

    esp_err_t init() {
        // Idempotent côté PCNT
        return enc_.init();
    }

    void setTargetRpm(float rpm) {
        target_rpm_ = std::clamp(rpm, -cfg_.rpm_max, +cfg_.rpm_max);
    }
    float targetRpm()   const { return target_rpm_; }
    float measuredRpm() const { return rpm_meas_;   }

    // Appelée périodiquement (ex. 100 Hz) avec dt RÉEL
void update(float dt)
{
    const int32_t d = enc_.getDelta();
    const float tpr = cfg_.use_encoder_tpr ? enc_.ticksPerRev() : cfg_.ticks_per_rev;
    float rpm = 0.0f;
    if (tpr > 0.0f && dt > 0.0f) {
        rpm = (static_cast<float>(d) / tpr) / dt * 60.0f;
    }

    rpm_meas_ = cfg_.lp_alpha * rpm + (1.0f - cfg_.lp_alpha) * rpm_meas_;

    const float u = pid_.compute(target_rpm_, rpm_meas_, dt);   // [-1..+1]

    // 🔎 DIAG (temporaire)
    ESP_LOGI("WC-L", "ref=%.0f rpm=%.0f dt=%.3f u=%.3f",
             target_rpm_, rpm_meas_, dt, u);

    if (u == 0.0f) {
        motor_.setDuty(0.0f);
        return;
    }

    float duty = std::fabs(u);
    if (duty > 0.f && duty < 0.10f) duty = 0.10f;   // dead‑zone soft

    motor_.setDirection(u >= 0.0f ? MotorDir::Forward : MotorDir::Reverse);
    motor_.setDuty(duty);
}

private:
    IMotorDriver&  motor_;
    PcntEncoder&   enc_;
    PIDController& pid_;
    Config         cfg_;

    float target_rpm_ = 0.0f;
    float rpm_meas_   = 0.0f; // lissé
};