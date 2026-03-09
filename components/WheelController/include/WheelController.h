#pragma once
#include <algorithm>
#include <cmath>
#include "esp_log.h"
#include "IMotorDriver.h"     // MotorDir, IMotorDriver
#include "PIDController.h"    // ci-dessus
#include "PcntEncoder.h"      // ton encodeur PCNT

class WheelController
{
public:
    struct Config {
        float rpm_max         = 3000.0f; // limite pour la consigne
        float lp_alpha        = 0.3f;    // lissage (0..1), 0=pas de lissage, 1=pas de mémoire
        float ticks_per_rev   = 2048.0f; // utilisé si enc.ticksPerRev() n'existe pas
        bool  use_encoder_tpr = true;    // true => lire tpr via enc.ticksPerRev()
    };

    WheelController(IMotorDriver& motor, PcntEncoder& enc, PIDController& pid, const Config& cfg)
    : motor_(motor), enc_(enc), pid_(pid), cfg_(cfg) {}

    esp_err_t init()
    {
        // Si tu veux initialiser le PCNT ici :
        return enc_.init(); // OK si déjà fait ailleurs, c'est idempotent
    }

    // Fixe la consigne en RPM (signée, + = Forward)
    void setTargetRpm(float rpm)
    {
        target_rpm_ = std::clamp(rpm, -cfg_.rpm_max, +cfg_.rpm_max);
    }

    float targetRpm()   const { return target_rpm_; }
    float measuredRpm() const { return rpm_meas_;   }

    // À appeler périodiquement (ex. 100 Hz)
    void update(float dt)
    {
        // 1) Mesure delta ticks sur dt
        const int32_t d = enc_.getAndClearDeltaTicks();
        const float tpr = cfg_.use_encoder_tpr ? enc_.ticksPerRev() : cfg_.ticks_per_rev;
        float rpm = 0.0f;
        if (tpr > 0 && dt > 0.0f) {
            rpm = (static_cast<float>(d) / tpr) / dt * 60.0f;
        }

        // 2) Lissage
        rpm_meas_ = cfg_.lp_alpha * rpm + (1.0f - cfg_.lp_alpha) * rpm_meas_;

        // 3) PID (entrée = RPM)
        const float u = pid_.compute(target_rpm_, rpm_meas_, dt); // u ∈ [-1..+1]

        // 4) Application driver
        if (u == 0.0f) {
            // Stop : idle = coast/brake selon ton driver
            motor_.setDuty(0.0f);
            return;
        }

        // Signe = sens
        const bool forward = (u >= 0.0f);
        motor_.setDirection(forward ? MotorDir::Forward : MotorDir::Reverse);
        motor_.setDuty(std::fabs(u)); // |u| ∈ [0..1]
    }

private:
    IMotorDriver&  motor_;
    PcntEncoder&   enc_;
    PIDController& pid_;
    Config         cfg_;

    float target_rpm_ = 0.0f;
    float rpm_meas_   = 0.0f; // lissé
};