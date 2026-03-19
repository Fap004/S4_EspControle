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
        float rpm_max         = 8000.0f; // plafond consigne — doit correspondre au moteur réel
        float lp_alpha        = 0.70f;   // lissage mesure RPM : poids sur le PASSÉ (0=pas de filtre, 1=gelé)
        float ticks_per_rev   = 68.0f;   // 17 PPR x4 = 68 (utilisé si use_encoder_tpr=false)
        bool  use_encoder_tpr = true;    // lire le TPR depuis l'encodeur si disponible
        float duty_min        = 0.08f;   // seuil minimal de duty : en dessous → moteur off
    };

    WheelController(IMotorDriver& motor, PcntEncoder& enc, PIDController& pid, const Config& cfg)
        : motor_(motor), enc_(enc), pid_(pid), cfg_(cfg) {}

    esp_err_t init();

    void  setTargetRpm(float rpm);
    float targetRpm()   const;
    float measuredRpm() const;

    void update(float dt);

private:
    IMotorDriver&  motor_;
    PcntEncoder&   enc_;
    PIDController& pid_;
    Config         cfg_;

    float target_rpm_ = 0.0f;
    float rpm_meas_   = 0.0f;
};