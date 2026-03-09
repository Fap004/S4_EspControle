#pragma once
#include "LedcHBridgeDriver.h"
#include "IMotorDriver.h"
#include "PcntEncoder.h"
#include "PIDController.h"
#include "WheelController.h"
#include "DriveBase.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Regroupe tous les objets "longue durée" et ressources partagées
struct AppContext
{
    // === Paramétrage matériel (ADAPTER) ===
    // Moteur gauche
    gpio_num_t L_INA  = GPIO_NUM_1;
    gpio_num_t L_INB  = GPIO_NUM_10;
    gpio_num_t L_SEL0 = GPIO_NUM_11;
    gpio_num_t L_PWM  = GPIO_NUM_13;

    // Moteur droit
    gpio_num_t R_INA  = GPIO_NUM_5;
    gpio_num_t R_INB  = GPIO_NUM_4;
    gpio_num_t R_SEL0 = GPIO_NUM_6;
    gpio_num_t R_PWM  = GPIO_NUM_0;

    // Encodeurs
    gpio_num_t L_ENC_A = GPIO_NUM_22;
    gpio_num_t L_ENC_B = GPIO_NUM_21;
    gpio_num_t R_ENC_A = GPIO_NUM_20;
    gpio_num_t R_ENC_B = GPIO_NUM_19;

    // === Instances bas niveau ===
    LedcHBridgeDriver::Config L_cfg;
    LedcHBridgeDriver::Config R_cfg;

    LedcHBridgeDriver::Pins  L_pins { L_INA, L_INB, L_PWM, L_SEL0 };
    LedcHBridgeDriver::Pins  R_pins { R_INA, R_INB, R_PWM, R_SEL0 };

    LedcHBridgeDriver motor_left  { L_pins, L_cfg };
    LedcHBridgeDriver motor_right { R_pins, R_cfg };

    // Encodeurs — TPR roue (A-DAPTER)
    static constexpr float TPR_LEFT  = 2000.0f;   // ← METS TA VALEUR MESURÉE, 17PPR ENCODER
    static constexpr float TPR_RIGHT = 2000.0f;   // ← METS TA VALEUR MESURÉE

    PcntEncoder enc_left  { L_ENC_A, L_ENC_B, TPR_LEFT  };
    PcntEncoder enc_right { R_ENC_A, R_ENC_B, TPR_RIGHT };

    // WheelControllers (utilisent TPR de l’encodeur)
    WheelController::Config wc_cfg {
        .rpm_max         = 7000.0f,
        .lp_alpha        = 0.7f,      // lissage
        .ticks_per_rev   = 0.0f,      // ignoré
        .use_encoder_tpr = true       // ← important !
    };

    WheelController wheel_left  { motor_left,  enc_left,  pid_left,  wc_cfg };
    WheelController wheel_right { motor_right, enc_right, pid_right, wc_cfg };

    // PID
    PIDController::Gains  gains { .Kp = 0.0035f, .Ki = 0.5f, .Kd = 0.0005f };
    PIDController::Limits lims  { .u_min = -1.0f, .u_max = +1.0f, .i_min = -0.5f, .i_max = +0.5f };
    PIDController pid_left  { gains, lims };
    PIDController pid_right { gains, lims };

    // DriveBase
    DriveBase::Geometry geom { .wheel_radius_m = 0.035f, .track_width_m = 0.180f };
    DriveBase drive { wheel_left, wheel_right, geom, /*rpm_max=*/7000.0f };

    // === Communication (queues/structs) ===
    // Une file de commande "haute-niveau" (v,ω) ou (rpmL,rpmR)
    struct CmdVW { float v_mps; float omega; };
    QueueHandle_t q_cmd_vw = nullptr;

    // Une file pour TX (télémétrie)
    struct Telemetry { float rpmL; float rpmR; };
    QueueHandle_t q_tlm = nullptr;
};

// Initialise configs LEDC, drivers, queues
esp_err_t appctx_init(AppContext& ctx);