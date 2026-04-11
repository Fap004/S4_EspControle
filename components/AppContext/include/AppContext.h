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
#include "Protocol.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "driver/gpio.h"

#include "McpwmServo.h"
#include "SteeringController.h"

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
    gpio_num_t R_ENC_A = GPIO_NUM_19;
    gpio_num_t R_ENC_B = GPIO_NUM_20;

    // === Instances bas niveau (MCPWM) ===
    McpwmHBridgeDriver::Config L_cfg{};
    McpwmHBridgeDriver::Config R_cfg{};

    McpwmHBridgeDriver::Pins L_pins{ L_INA, L_INB, L_PWM, L_SEL0 };
    McpwmHBridgeDriver::Pins R_pins{ R_INA, R_INB, R_PWM, R_SEL0 };

    McpwmHBridgeDriver motor_left  { L_pins, L_cfg };
    McpwmHBridgeDriver motor_right { R_pins, R_cfg };

    // === Encodeurs ===
    static constexpr float TPR_LEFT  = 68.0f;
    static constexpr float TPR_RIGHT = 68.0f;

    PcntEncoder enc_left  { L_ENC_A, L_ENC_B, TPR_LEFT  };
    PcntEncoder enc_right { R_ENC_A, R_ENC_B, TPR_RIGHT };

    // === PID ===
    PIDController::Limits lims { .u_min = -1.0f, .u_max = +1.0f,
                                 .i_min = -0.5f, .i_max = +0.5f };

    PIDController::Gains gains_left  { .Kp = 1.5f, .Ki = 1.0f, .Kd = 0.1f };
    PIDController::Gains gains_right { .Kp = 1.5f, .Ki = 1.0f, .Kd = 0.1f };

    PIDController pid_left  { gains_left,  lims };
    PIDController pid_right { gains_right, lims };

    // === WheelControllers ===
    WheelController::Config wc_cfg {
        .rpm_max         = 2000.0f,//8000
        .lp_alpha        = 0.70f,//essayer plus bas
        .ticks_per_rev   = 0.0f,
        .use_encoder_tpr = true,
        .duty_min        = 0.08f,
    };

    WheelController wheel_left  { motor_left,  enc_left,  pid_left,  wc_cfg };
    WheelController wheel_right { motor_right, enc_right, pid_right, wc_cfg };

    // === DriveBase ===
    DriveBase::Geometry geom {
        .wheel_radius_m = 0.037f,
        .track_width_m  = 0.190f,
        .wheel_base_m   = 0.190f
    };
    DriveBase drive { wheel_left, wheel_right, geom, /*rpm_max=*/2000.0f };

    // === Servo de direction (GPIO 23) ===
    McpwmServo::Config servo_cfg {
        .unit      = MCPWM_UNIT_0,
        .timer     = MCPWM_TIMER_2,  // ✅ libre
        .freq_hz   = 50,             // ✅ servo RC standard
        .duty_init = 0.0f,
        .pw_min_us = 500,            // ✅ DS3218 datasheet
        .pw_max_us = 2500,           // ✅ DS3218 datasheet
        .angle_max = 135.0f          // ✅ 270°/2
    };
    McpwmServo         servo    { GPIO_NUM_23, servo_cfg };
    SteeringController steering { servo, -30.0f, +30.0f }; // limite mécanique

    // === Sélecteur de mode & watchdog RX ===
    enum class ControlMode : uint8_t { LOCAL, REMOTE };
    ControlMode ctrl_mode        = ControlMode::LOCAL;
    TickType_t  last_rx_cmd_tick = 0;
    TickType_t  RX_CMD_TIMEOUT   = pdMS_TO_TICKS(200);
    uint8_t tlm_unit = PROTO_UNIT_KMH;  // ← défaut km/h

    // === Communication (queues) ===
    // v_mps    : vitesse linéaire (m/s)
    // steer_deg: angle de braquage (-30° à +30°)
    struct CmdVW     { float v_mps; float steer_deg; };
    struct Telemetry { float rpmL;  float rpmR;      };

    QueueHandle_t q_cmd_vw = nullptr;
    QueueHandle_t q_tlm    = nullptr;
};

esp_err_t appctx_init(AppContext& ctx);