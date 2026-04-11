#include "WheelController.h"
#include <algorithm>
#include <cmath>

static const char* TAG_WC = "WheelCtrl";

// ──────────────────────────────────────────────────────────────
esp_err_t WheelController::init()
{
    return enc_.init();
}

// ──────────────────────────────────────────────────────────────
void WheelController::setTargetRpm(float rpm)
{
    // Consigne brute (REMOTE / DriveBase)
    target_rpm_cmd_ = std::clamp(rpm, -cfg_.rpm_max, +cfg_.rpm_max);
}

float WheelController::targetRpm() const
{
    return target_rpm_;
}

float WheelController::measuredRpm() const
{
    return rpm_meas_;
}

// ──────────────────────────────────────────────────────────────
// Rampe utilitaire
static inline float ramp(float cur, float tgt, float rate, float dt)
{
    const float d = tgt - cur;
    const float step = rate * dt;
    if (d >  step) return cur + step;
    if (d < -step) return cur - step;
    return tgt;
}

// ──────────────────────────────────────────────────────────────
void WheelController::update(float dt)
{
    if (dt <= 0.0f) return;

    // 1) Rampe RPM (PROTECTION DRIVER)
    const float rpm_ramp_rate = 1500.0f; // RPM/s ✅ safe pour VNH7070
    target_rpm_ = ramp(target_rpm_, target_rpm_cmd_, rpm_ramp_rate, dt);

    // 2) Lecture encodeur
    const int32_t d   = enc_.getDelta();
    const float   tpr = cfg_.use_encoder_tpr ? enc_.ticksPerRev()
                                             : cfg_.ticks_per_rev;

    // 3) Conversion ticks → RPM
    float rpm_raw = 0.0f;
    if (tpr > 0.0f)
        rpm_raw = (static_cast<float>(d) / tpr) / dt * 60.0f;

    // 4) Filtre passe-bas
    rpm_meas_ = cfg_.lp_alpha * rpm_meas_
              + (1.0f - cfg_.lp_alpha) * rpm_raw;

    // 5) Normalisation pour le PID
    const float ref_norm  = target_rpm_ / cfg_.rpm_max;
    const float meas_norm = rpm_meas_   / cfg_.rpm_max;

    const float u = pid_.compute(ref_norm, meas_norm, dt);

    // 6) Limitation PWM à très basse vitesse (ANTI SUR-COURANT)
    float duty = std::fabs(u);
    if (std::fabs(rpm_meas_) < 100.0f)
        duty = std::min(duty, 0.4f);   // ✅ jamais 100% à l'arrêt

    // 7) Dead-zone propre
    if (duty < cfg_.duty_min)
    {
        motor_.setDuty(0.0f);
        return;
    }

    // 8) Commande moteur
    motor_.setDirection(u >= 0.0f ? MotorDir::Forward
                                  : MotorDir::Reverse);
    motor_.setDuty(duty);
}
