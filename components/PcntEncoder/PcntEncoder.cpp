#include "PcntEncoder.h"
#include "esp_log.h"

static const char* TAG = "PCNT_ENC";

PcntEncoder::PcntEncoder(gpio_num_t pinA, gpio_num_t pinB,
                         float ticks_per_rev,
                         uint32_t glitch_ns,
                         bool use_pullups,
                         bool inverted,
                         int high_limit,
                         int low_limit)
: pinA_(pinA),
  pinB_(pinB),
  ticks_per_rev_(ticks_per_rev > 0.f ? ticks_per_rev : 1.f),
  glitch_ns_(glitch_ns),
  use_pullups_(use_pullups),
  inverted_(inverted),
  high_limit_(high_limit),
  low_limit_(low_limit)
{}

// ─────────────────────────────────────────────
esp_err_t PcntEncoder::configGpioInputs_()
{
    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << pinA_) | (1ULL << pinB_);
    io.mode         = GPIO_MODE_INPUT;
    io.pull_up_en   = use_pullups_ ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type    = GPIO_INTR_DISABLE;

    return gpio_config(&io);
}

// ─────────────────────────────────────────────
esp_err_t PcntEncoder::init()
{
    if (unitA_ || unitB_) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(configGpioInputs_(), TAG, "GPIO config");

    // ───────────── UNIT CONFIG ─────────────
    pcnt_unit_config_t ucfg = {};
    ucfg.high_limit = high_limit_;
    ucfg.low_limit  = low_limit_;

    ESP_RETURN_ON_ERROR(pcnt_new_unit(&ucfg, &unitA_), TAG, "unit A");
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&ucfg, &unitB_), TAG, "unit B");

    // ───────────── CHANNEL A ─────────────
    pcnt_chan_config_t cA = {};
    cA.edge_gpio_num  = pinA_;
    cA.level_gpio_num = pinB_;

    ESP_RETURN_ON_ERROR(pcnt_new_channel(unitA_, &cA, &chA_), TAG, "chan A");

    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_edge_action(
            chA_,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE),
        TAG, "edge A");

    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_level_action(
            chA_,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
        TAG, "level A");

    // ───────────── CHANNEL B ─────────────
    pcnt_chan_config_t cB = {};
    cB.edge_gpio_num  = pinB_;
    cB.level_gpio_num = pinA_;

    ESP_RETURN_ON_ERROR(pcnt_new_channel(unitB_, &cB, &chB_), TAG, "chan B");

    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_edge_action(
            chB_,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE),
        TAG, "edge B");

    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_level_action(
            chB_,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
        TAG, "level B");

    // ───────────── GLITCH FILTER ─────────────
    if (glitch_ns_ > 0) {
        pcnt_glitch_filter_config_t f = {
            .max_glitch_ns = glitch_ns_
        };

        esp_err_t errA = pcnt_unit_set_glitch_filter(unitA_, &f);
        esp_err_t errB = pcnt_unit_set_glitch_filter(unitB_, &f);

        if (errA != ESP_OK || errB != ESP_OK) {
            ESP_LOGW(TAG, "Glitch filter not supported → disabling");
            pcnt_glitch_filter_config_t f0 = {.max_glitch_ns = 0};
            pcnt_unit_set_glitch_filter(unitA_, &f0);
            pcnt_unit_set_glitch_filter(unitB_, &f0);
            glitch_ns_ = 0;
        }
    }

    // ───────────── START ─────────────
    ESP_RETURN_ON_ERROR(pcnt_unit_enable(unitA_), TAG, "enable A");
    ESP_RETURN_ON_ERROR(pcnt_unit_enable(unitB_), TAG, "enable B");

    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(unitA_), TAG, "clear A");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(unitB_), TAG, "clear B");

    ESP_RETURN_ON_ERROR(pcnt_unit_start(unitA_), TAG, "start A");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(unitB_), TAG, "start B");

    started_ = true;

    pcnt_unit_get_count(unitA_, &last_raw_A_);
    pcnt_unit_get_count(unitB_, &last_raw_B_);

    accum_total_ = 0;
    rpm_lpf_     = 0.0f;

    ESP_LOGI(TAG, "Encoder OK (A=%d B=%d, glitch=%u ns)",
             (int)pinA_, (int)pinB_, glitch_ns_);

    return ESP_OK;
}

// ─────────────────────────────────────────────
void PcntEncoder::deinit()
{
    if (!unitA_ && !unitB_) return;

    if (started_) {
        pcnt_unit_stop(unitA_);
        pcnt_unit_stop(unitB_);
        started_ = false;
    }

    if (chA_) { pcnt_del_channel(chA_); chA_ = nullptr; }
    if (chB_) { pcnt_del_channel(chB_); chB_ = nullptr; }
    if (unitA_) { pcnt_del_unit(unitA_); unitA_ = nullptr; }
    if (unitB_) { pcnt_del_unit(unitB_); unitB_ = nullptr; }
}

// ─────────────────────────────────────────────
void PcntEncoder::reset()
{
    pcnt_unit_clear_count(unitA_);
    pcnt_unit_clear_count(unitB_);

    pcnt_unit_get_count(unitA_, &last_raw_A_);
    pcnt_unit_get_count(unitB_, &last_raw_B_);

    accum_total_ = 0;
    rpm_lpf_     = 0.0f;
}

// ─────────────────────────────────────────────
int32_t PcntEncoder::getDelta()
{
    int rawA = 0, rawB = 0;

    pcnt_unit_get_count(unitA_, &rawA);
    pcnt_unit_get_count(unitB_, &rawB);

    int32_t dA = rawA - last_raw_A_;
    int32_t dB = rawB - last_raw_B_;

    last_raw_A_ = rawA;
    last_raw_B_ = rawB;

    int32_t delta = dA + dB;

    if (inverted_) delta = -delta;

    accum_total_ += delta;

    return delta;
}

// ─────────────────────────────────────────────
float PcntEncoder::rpm(float dt_s, bool lpf)
{
    if (dt_s <= 0.0f) return rpm_lpf_;

    int32_t dticks = getDelta();

    float revs = dticks / ticks_per_rev_;
    float rpm  = (revs / dt_s) * 60.0f;

    if (!lpf) {
        rpm_lpf_ = rpm;
        return rpm;
    }

    rpm_lpf_ = lp_alpha_ * rpm_lpf_ + (1.0f - lp_alpha_) * rpm;
    return rpm_lpf_;
}

// ─────────────────────────────────────────────
int32_t PcntEncoder::getAndClearDeltaTicks()
{
    return getDelta();
}