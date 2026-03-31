#pragma once

#include <cstdint>
#include <algorithm>
#include <cmath>

#include "McpwmHBridgeDriver.h"
#include "IMotorDriver.h"
#include "PcntEncoder.h"
#include "PIDController.h"
#include "WheelController.h"
#include "DriveBase.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "driver/gpio.h"

#include "McpwmServo.h"
#include "SteeringController.h"

// ----------------------------------------------------------------------------
// AppContext : regroupe les objets "longue durée" et ressources partagées
// ----------------------------------------------------------------------------
struct AppContext
{
    // === Paramétrage matériel ===
    gpio_num_t L_INA  = GPIO_NUM_4;
    gpio_num_t L_INB  = GPIO_NUM_5;
    gpio_num_t L_SEL0 = GPIO_NUM_6;
    gpio_num_t L_PWM  = GPIO_NUM_0;

    gpio_num_t R_INA  = GPIO_NUM_1;
    gpio_num_t R_INB  = GPIO_NUM_10;
    gpio_num_t R_SEL0 = GPIO_NUM_11;
    gpio_num_t R_PWM  = GPIO_NUM_13;

    gpio_num_t L_ENC_A = GPIO_NUM_21;
    gpio_num_t L_ENC_B = GPIO_NUM_22;
    gpio_num_t R_ENC_A = GPIO_NUM_19;//inv
    gpio_num_t R_ENC_B = GPIO_NUM_20;//inv

    // === Instances bas niveau (MCPWM) ===
    McpwmHBridgeDriver::Config L_cfg{};
    McpwmHBridgeDriver::Config R_cfg{};

    McpwmHBridgeDriver::Pins L_pins{ L_INA, L_INB, L_PWM, L_SEL0 };
    McpwmHBridgeDriver::Pins R_pins{ R_INA, R_INB, R_PWM, R_SEL0 };

    McpwmHBridgeDriver motor_left  { L_pins, L_cfg };
    McpwmHBridgeDriver motor_right { R_pins, R_cfg };

    // === Encodeurs ===
    static constexpr float TPR_LEFT  = 68.0f;   // 17 PPR × 4
    static constexpr float TPR_RIGHT = 68.0f;

    PcntEncoder enc_left  { L_ENC_A, L_ENC_B, TPR_LEFT  };
    PcntEncoder enc_right { R_ENC_A, R_ENC_B, TPR_RIGHT };

    // === PID ===
    // ⚠️ Les gains s'appliquent sur une erreur NORMALISÉE [-1..+1]
    //    (WheelController divise par rpm_max avant d'appeler pid.compute())
    //
    //    Lecture intuitive :
    //      Kp = 0.5  → 50 % de duty pour 100 % d'erreur (moteur à l'arrêt, consigne max)
    //      Ki = 0.2  → intégrale modérée ; augmenter si erreur statique résiduelle
    //      Kd = 0.0  → dérivé désactivé pour commencer (activer après avoir réglé P+I)
    //
    //    Procédure de tuning recommandée :
    //      1) Ki=0, Kd=0 → augmenter Kp jusqu'à oscillation légère, puis reculer 20 %
    //      2) Ajouter Ki petit à petit pour éliminer l'erreur statique
    //      3) Ajouter Kd seulement si dépassement résiduel (filtrer avec d_alpha ≈ 0.85)
    PIDController::Limits lims  { .u_min = -1.0f, .u_max = +1.0f,
                                .i_min = -0.5f, .i_max = +0.5f };

    PIDController::Gains gains_left  { .Kp = 1.5f, .Ki = 1.0f, .Kd = 0.1f };
    PIDController::Gains gains_right { .Kp = 1.5f, .Ki = 1.0f, .Kd = 0.1f };

    PIDController pid_left  { gains_left,  lims };
    PIDController pid_right { gains_right, lims };

    // === WheelControllers ===
    WheelController::Config wc_cfg {
        .rpm_max         = 8000.0f,  // RPM max réel du moteur (à vérifier sur datasheet)
        .lp_alpha        = 0.70f,    // filtre LP : poids sur le PASSÉ (convention corrigée)
        .ticks_per_rev   = 0.0f,     // ignoré car use_encoder_tpr = true
        .use_encoder_tpr = true,
        .duty_min        = 0.08f,    // cohérent avec McpwmHBridgeDriver::Config::duty_min
    };

    WheelController wheel_left  { motor_left,  enc_left,  pid_left,  wc_cfg };
    WheelController wheel_right { motor_right, enc_right, pid_right, wc_cfg };

    // === DriveBase ===
    DriveBase::Geometry geom { .wheel_radius_m = 0.035f, .track_width_m = 0.180f };
    DriveBase drive { wheel_left, wheel_right, geom, /*rpm_max=*/8000.0f };

    // === Servo de direction (GPIO 23) ===
    McpwmServo::Config servo_cfg{
    .unit      = MCPWM_UNIT_0,
    .timer     = MCPWM_TIMER_2,  // libre chez toi ✅
    .freq_hz   = 50,             // ✅ servo RC standard
    .duty_init = 0.0f,           // ✅ AUCUN signal au boot
    .pw_min_us = 1000,
    .pw_max_us = 2000,
    .angle_max = 30.0f
};
    McpwmServo         servo { GPIO_NUM_23, servo_cfg };
    SteeringController steering { servo, -30.0f, +30.0f };

    // === Sélecteur de mode & watchdog RX ===
    enum class ControlMode : uint8_t { LOCAL, REMOTE };
    ControlMode ctrl_mode        = ControlMode::LOCAL;
    TickType_t  last_rx_cmd_tick = 0;
    TickType_t  RX_CMD_TIMEOUT   = pdMS_TO_TICKS(200);

    // === Communication (queues) ===
    struct CmdVW     { float v_mps; float omega; };
    struct Telemetry { float rpmL;  float rpmR;  };

    QueueHandle_t q_cmd_vw = nullptr;
    QueueHandle_t q_tlm    = nullptr;
};

esp_err_t appctx_init(AppContext& ctx);
