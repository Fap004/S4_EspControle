#include "WheelController.h"

static const char* TAG_WC = "WheelCtrl";

// ──────────────────────────────────────────────────────────────────────────────
esp_err_t WheelController::init()
{
    return enc_.init();
}

// ──────────────────────────────────────────────────────────────────────────────
void WheelController::setTargetRpm(float rpm)
{
    target_rpm_ = std::clamp(rpm, -cfg_.rpm_max, +cfg_.rpm_max);
}

float WheelController::targetRpm()   const { return target_rpm_; }
float WheelController::measuredRpm() const { return rpm_meas_;   }

// ──────────────────────────────────────────────────────────────────────────────
void WheelController::update(float dt)
{
    // 1) Lecture delta ticks — seul appel à getDelta() dans tout le projet pour cet encodeur
    const int32_t d   = enc_.getDelta();
    const float   tpr = cfg_.use_encoder_tpr ? enc_.ticksPerRev() : cfg_.ticks_per_rev;

    // 2) Conversion ticks → RPM brut
    float rpm_raw = 0.0f;
    if (tpr > 0.0f && dt > 0.0f)
        rpm_raw = (static_cast<float>(d) / tpr) / dt * 60.0f;

    // 3) Filtre passe-bas exponentiel
    //    lp_alpha = poids sur le PASSÉ  (0 = pas de filtre, 0.70 = lissage modéré)
    rpm_meas_ = cfg_.lp_alpha * rpm_meas_ + (1.0f - cfg_.lp_alpha) * rpm_raw;

    // 4) Normalisation [-1..+1] avant le PID
    //    L'erreur reste dans [−1..+1] → les gains sont intuitifs (Kp=0.4, Ki=0.1 pour démarrer)
    const float ref_norm  = target_rpm_ / cfg_.rpm_max;
    const float meas_norm = rpm_meas_   / cfg_.rpm_max;
    const float u = pid_.compute(ref_norm, meas_norm, dt);

    //ESP_LOGI(TAG_WC, "ref=%.0f  meas=%.0f  ref_n=%.3f  meas_n=%.3f  u=%.3f  dt=%.4f",
    //         target_rpm_, rpm_meas_, ref_norm, meas_norm, u, dt);

    // 5) Dead-zone sans snap brutal
    //    En dessous de duty_min, on éteint proprement le moteur
    const float duty = std::fabs(u);
    if (duty < cfg_.duty_min)
    {
        motor_.setDuty(0.0f);
        return;
    }

    // 6) Commande moteur
    motor_.setDirection(u >= 0.0f ? MotorDir::Forward : MotorDir::Reverse);
    motor_.setDuty(duty);
}
