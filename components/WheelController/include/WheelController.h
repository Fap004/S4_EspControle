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
        float rpm_max         = 8000.0f; // plafond consigne RPM
        float lp_alpha        = 0.70f;   // filtre passe-bas RPM
        float ticks_per_rev   = 68.0f;   // utilisé si use_encoder_tpr=false
        bool  use_encoder_tpr = true;
        float duty_min        = 0.08f;   // seuil PWM minimal
    };

    WheelController(IMotorDriver& motor,
                    PcntEncoder&  enc,
                    PIDController& pid,
                    const Config&  cfg)
        : motor_(motor)
        , enc_(enc)
        , pid_(pid)
        , cfg_(cfg)
    {}

    // ── Init encodeur
    esp_err_t init();

    // ── API publique
    void  setTargetRpm(float rpm);   // consigne BRUTE (rampe interne)
    float targetRpm()   const;
    float measuredRpm() const;

    // ── Boucle de contrôle
    void update(float dt);

private:
    // ── Dépendances
    IMotorDriver&  motor_;
    PcntEncoder&   enc_;
    PIDController& pid_;
    Config         cfg_;

    // ── État interne
    float target_rpm_cmd_ = 0.0f;   // ✅ consigne externe (REMOTE / DriveBase)
    float target_rpm_     = 0.0f;   // ✅ consigne RAMPÉE (safe moteur)
    float rpm_meas_       = 0.0f;   // ✅ RPM mesurée filtrée

    // ── Sauvegarde des gains PID
    float pid_ki_nominal_ = 1.0f;   // ✅ valeur normale de Ki
};