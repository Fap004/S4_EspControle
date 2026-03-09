#include "PcntEncoder.h"
#include "esp_log.h"
#include "esp_check.h"

static const char* TAG = "PcntEncoder";

PcntEncoder::PcntEncoder(gpio_num_t pulsePin, gpio_num_t ctrlPin,
                         float ticks_per_rev, uint32_t glitch_ns)
: pulse_pin_(pulsePin),
  ctrl_pin_(ctrlPin),
  ticks_per_rev_(ticks_per_rev),
  glitch_ns_(glitch_ns),
  last_total_(0)
{}

esp_err_t PcntEncoder::init()
{
    pcnt_unit_config_t ucfg = {};
    ucfg.high_limit =  32767;
    ucfg.low_limit  = -32768;

    ESP_RETURN_ON_ERROR(pcnt_new_unit(&ucfg, &unit_), TAG, "pcnt_new_unit");

    pcnt_chan_config_t cfg = {};
    cfg.edge_gpio_num  = pulse_pin_;
    cfg.level_gpio_num = ctrl_pin_;
    ESP_RETURN_ON_ERROR(pcnt_new_channel(unit_, &cfg, &channel_), TAG, "pcnt_new_channel");

    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_edge_action(
            channel_,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE),
        TAG, "edge_action");

    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_level_action(
            channel_,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
        TAG, "level_action");

    if (glitch_ns_ > 0) {
        pcnt_glitch_filter_config_t f = {};
        f.max_glitch_ns = glitch_ns_;
        ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(unit_, &f), TAG, "glitch_filter");
    }

    ESP_RETURN_ON_ERROR(pcnt_unit_enable(unit_), TAG, "enable");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(unit_), TAG, "clear");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(unit_), TAG, "start");

    // FIX : lecture en int puis recopie vers int32_t
    int raw = 0;
    pcnt_unit_get_count(unit_, &raw);
    last_total_ = raw;

    ESP_LOGI(TAG, "PCNT OK: A=%d, B=%d, TPR=%.1f",
             pulse_pin_, ctrl_pin_, ticks_per_rev_);
    return ESP_OK;
}

void PcntEncoder::reset()
{
    if (!unit_) return;
    pcnt_unit_clear_count(unit_);
    last_total_ = 0;
}

// Recentrage sécurité
void PcntEncoder::recenterIfNearLimit(int curr)
{
    const int TH = 30000;
    if (curr > TH || curr < -TH) {
        pcnt_unit_clear_count(unit_);
        last_total_ = 0;
    }
}

int32_t PcntEncoder::getDelta()
{
    if (!unit_) return 0;

    //FIX : lecture en int puis recopie dans int32_t
    int raw = 0;
    pcnt_unit_get_count(unit_, &raw);

    int32_t delta = raw - last_total_;
    last_total_   = raw;

    recenterIfNearLimit(raw);

    return delta;
}