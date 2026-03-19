#include "PcntEncoder.h"
#include "esp_log.h"
#include "esp_check.h"

static const char* TAG_Q = "PCNT_QUAD";

PcntEncoder::PcntEncoder(gpio_num_t pinA, gpio_num_t pinB,
                         float ticks_per_rev,
                         uint32_t glitch_ns,
                         bool use_pullups,
                         bool inverted,
                         int high_limit,
                         int low_limit)
: pinA_(pinA), pinB_(pinB),
  ticks_per_rev_(ticks_per_rev > 0.f ? ticks_per_rev : 1.f),
  glitch_ns_(glitch_ns),
  use_pullups_(use_pullups),
  inverted_(inverted),
  high_limit_(high_limit),
  low_limit_(low_limit) {}

esp_err_t PcntEncoder::configGpioInputs_()
{
    if (!use_pullups_) return ESP_OK;
    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << pinA_) | (1ULL << pinB_);
    io.mode         = GPIO_MODE_INPUT;
    io.pull_up_en   = GPIO_PULLUP_ENABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type    = GPIO_INTR_DISABLE;
    return gpio_config(&io);
}

esp_err_t PcntEncoder::init()
{
    if (unitA_) {
        ESP_LOGW(TAG_Q, "Already initialized");
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(configGpioInputs_(), TAG_Q, "gpio_config");

    // 1) Unité PCNT
    pcnt_unit_config_t ucfg = {};
    ucfg.high_limit = high_limit_;
    ucfg.low_limit  = low_limit_;
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&ucfg, &unitA_), TAG_Q, "pcnt_new_unit");

    // 2) Canal A : edge=A, level=B
    pcnt_chan_config_t cA = {};
    cA.edge_gpio_num  = pinA_;
    cA.level_gpio_num = pinB_;
    ESP_RETURN_ON_ERROR(pcnt_new_channel(unitA_, &cA, &chA_), TAG_Q, "new_channel A");

    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_edge_action(
            chA_, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE),
        TAG_Q, "edge_action A");

    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_level_action(
            chA_, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
        TAG_Q, "level_action A");

    // 3) Canal B : edge=B, level=A
    pcnt_chan_config_t cB = {};
    cB.edge_gpio_num  = pinB_;
    cB.level_gpio_num = pinA_;
    ESP_RETURN_ON_ERROR(pcnt_new_channel(unitA_, &cB, &chB_), TAG_Q, "new_channel B");

    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_edge_action(
            chB_, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE),
        TAG_Q, "edge_action B");

    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_level_action(
            chB_, PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP),
        TAG_Q, "level_action B");

    // 4) Glitch filter : essai dégressif
    if (glitch_ns_ > 0) {
        static const uint32_t candidates_ns[] = {
            150000, 120000, 100000, 80000, 60000, 50000, 30000,
            20000, 10000, 5000, 2000, 1000
        };
        bool set_ok = false;
        for (uint32_t ns : candidates_ns) {
            if (ns > glitch_ns_) continue;
            pcnt_glitch_filter_config_t f = { .max_glitch_ns = ns };
            if (pcnt_unit_set_glitch_filter(unitA_, &f) == ESP_OK) {
                glitch_ns_ = ns; set_ok = true;
                //ESP_LOGI(TAG_Q, "glitch filter set to %u ns", ns);
                break;
            }
        }
        if (!set_ok) { ESP_LOGW(TAG_Q, "glitch filter disabled"); glitch_ns_ = 0; }
    }

    // 5) Démarrer
    ESP_RETURN_ON_ERROR(pcnt_unit_enable(unitA_),      TAG_Q, "enable");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(unitA_), TAG_Q, "clear");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(unitA_),       TAG_Q, "start");
    started_ = true;

    (void)pcnt_unit_get_count(unitA_, &last_raw_A_);
    accum_total_ = 0;
    rpm_lpf_     = 0.0f;

    //ESP_LOGI(TAG_Q, "PCNT QUAD OK: A=%d B=%d TPR=%.1f glitch=%u ns inverted=%d",
    //         (int)pinA_, (int)pinB_, ticks_per_rev_, glitch_ns_, inverted_);
    return ESP_OK;
}

void PcntEncoder::deinit()
{
    if (!unitA_) return;
    if (started_) { (void)pcnt_unit_stop(unitA_); started_ = false; }
    if (chA_) { (void)pcnt_del_channel(chA_); chA_ = nullptr; }
    if (chB_) { (void)pcnt_del_channel(chB_); chB_ = nullptr; }
    (void)pcnt_del_unit(unitA_);
    unitA_ = nullptr;
}

void PcntEncoder::reset()
{
    if (!unitA_) return;
    (void)pcnt_unit_clear_count(unitA_);
    (void)pcnt_unit_get_count(unitA_, &last_raw_A_);
    accum_total_ = 0;
    rpm_lpf_     = 0.0f;
}

void PcntEncoder::recenterIfNearLimit_(int raw)
{
    const int TH = 30000;
    if (raw > TH || raw < -TH) {
        (void)pcnt_unit_clear_count(unitA_);
        last_raw_A_ = 0;
    }
}

int32_t PcntEncoder::getDelta()
{
    if (!unitA_) return 0;

    int raw = 0;
    (void)pcnt_unit_get_count(unitA_, &raw);

    int32_t delta = raw - last_raw_A_;
    last_raw_A_   = raw;

    if (inverted_) delta = -delta;

    accum_total_ += delta;
    recenterIfNearLimit_(raw);
    return delta;
}

float PcntEncoder::rpm(float dt_s, bool lpf)
{
    if (!(dt_s > 0.0f) || !std::isfinite(dt_s)) return rpm_lpf_;

    const int32_t dticks = getDelta();
    const float   revs   = (ticks_per_rev_ > 0.f) ? (dticks / ticks_per_rev_) : 0.f;
    const float   rpm_i  = (revs / dt_s) * 60.0f;

    if (!lpf) { rpm_lpf_ = rpm_i; return rpm_i; }

    rpm_lpf_ = lp_alpha_ * rpm_lpf_ + (1.0f - lp_alpha_) * rpm_i;
    return rpm_lpf_;
}

int32_t PcntEncoder::getAndClearDeltaTicks() { return getDelta(); }