#pragma once

#include "McpwmHBridgeDriver.h"   // <- driver moteur (MCPWM)
#include "IMotorDriver.h"
#include "PcntEncoder.h"
#include "PIDController.h"
#include "WheelController.h"
#include "DriveBase.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "driver/gpio.h"

// ----------------------------------------------------------------------------
// AppContext : regroupe les objets "longue durée" et ressources partagées
// ----------------------------------------------------------------------------
struct AppContext
{
    // === Paramétrage matériel ===
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

    // === Instances bas niveau (MCPWM) ===
    McpwmHBridgeDriver::Config L_cfg{};
    McpwmHBridgeDriver::Config R_cfg{};

    McpwmHBridgeDriver::Pins  L_pins{ L_INA, L_INB, L_PWM, L_SEL0 };
    McpwmHBridgeDriver::Pins  R_pins{ R_INA, R_INB, R_PWM, R_SEL0 };

    McpwmHBridgeDriver        motor_left { L_pins, L_cfg };
    McpwmHBridgeDriver        motor_right{ R_pins, R_cfg };

    // === Encodeurs ===
    static constexpr float TPR_LEFT  = 68.0f;  // ← mets ta valeur mesurée
    static constexpr float TPR_RIGHT = 68.0f;  // ← idem

    PcntEncoder enc_left  { L_ENC_A, L_ENC_B, TPR_LEFT  };
    PcntEncoder enc_right { R_ENC_A, R_ENC_B, TPR_RIGHT };

    // === PID & WheelControllers ===
    PIDController::Gains  gains { .Kp = 0.0035f, .Ki = 0.0f, .Kd = 0.0f };
    PIDController::Limits lims  { .u_min = -1.0f, .u_max = +1.0f, .i_min = -0.5f, .i_max = +0.5f };
    PIDController pid_left  { gains, lims };
    PIDController pid_right { gains, lims };

    WheelController::Config wc_cfg {
        .rpm_max         = 7000.0f,
        .lp_alpha        = 0.7f,
        .ticks_per_rev   = 0.0f,   // ignoré si use_encoder_tpr = true
        .use_encoder_tpr = true
    };

    WheelController wheel_left  { motor_left,  enc_left,  pid_left,  wc_cfg };
    WheelController wheel_right { motor_right, enc_right, pid_right, wc_cfg };

    // === DriveBase ===
    DriveBase::Geometry geom { .wheel_radius_m = 0.035f, .track_width_m = 0.180f };
    DriveBase drive { wheel_left, wheel_right, geom, /*rpm_max=*/7000.0f };

    // === Communication (queues) ===
    struct CmdVW     { float v_mps; float omega; };
    struct Telemetry { float rpmL;  float rpmR;  };

    QueueHandle_t q_cmd_vw = nullptr;  // commandes haut-niveau (v, ω)
    QueueHandle_t q_tlm    = nullptr;  // télémétrie
};

// Initialise configs MCPWM, drivers, encodeurs, queues
esp_err_t appctx_init(AppContext& ctx);