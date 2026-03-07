#include "PcntEncoder.h"
#include "esp_log.h"
#include "esp_check.h"   // ESP_RETURN_ON_ERROR

static const char* TAG = "PcntEncoder";

// ----- CTOR -----
PcntEncoder::PcntEncoder(gpio_num_t pulsePin, gpio_num_t ctrlPin,
                         float ticks_per_rev, uint32_t glitch_ns)
: pulse_pin_(pulsePin),
  ctrl_pin_(ctrlPin),
  ticks_per_rev_(ticks_per_rev),
  glitch_ns_(glitch_ns)
{}

// ----- INIT -----
esp_err_t PcntEncoder::init()
{
    // 1) Créer l'unité PCNT
    pcnt_unit_config_t ucfg = {};             // <-- zero-init pour éviter l'ordre des designators
    ucfg.high_limit   =  32767;
    ucfg.low_limit    = -32768;
    // ucfg.intr_priority = 0;    // défaut
    // ucfg.flags = {};           // défaut

    ESP_RETURN_ON_ERROR(pcnt_new_unit(&ucfg, &unit_), TAG, "pcnt_new_unit");

    // 2) Créer le canal (A en edge, B en level)
    pcnt_chan_config_t edge_cfg = {};         // <-- zero-init
    edge_cfg.edge_gpio_num   = pulse_pin_;    // A
    edge_cfg.level_gpio_num  = ctrl_pin_;     // B
    edge_cfg.flags.invert_edge_input  = 0;
    edge_cfg.flags.invert_level_input = 0;
    // edge_cfg.flags.io_loop_back / virt_* : laissons à 0

    ESP_RETURN_ON_ERROR(pcnt_new_channel(unit_, &edge_cfg, &ch_edge_), TAG, "pcnt_new_channel");

    // 3) Actions sur EDGE : RISING (+1) / FALLING (-1)
    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_edge_action(
            ch_edge_,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,   // rising
            PCNT_CHANNEL_EDGE_ACTION_DECREASE    // falling
        ),
        TAG, "pcnt_channel_set_edge_action"
    );

    // 4) Action sur LEVEL (B) : quand level=1 → inverser le signe
    //    (level=0 : KEEP ; level=1 : INVERSE)
    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_level_action(
            ch_edge_,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE   // <-- nom exact dans driver/pulse_cnt.h
        ),
        TAG, "pcnt_channel_set_level_action"
    );

    // 5) Filtre anti-glitch (ignore impulsions < glitch_ns_)
    if (glitch_ns_ > 0) {
        pcnt_glitch_filter_config_t filter_cfg = {};
        filter_cfg.max_glitch_ns = glitch_ns_;
        ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(unit_, &filter_cfg), TAG, "glitch_filter");
    }

    // 6) Activer et démarrer
    ESP_RETURN_ON_ERROR(pcnt_unit_enable(unit_), TAG, "pcnt_unit_enable");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(unit_), TAG, "pcnt_unit_clear_count");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(unit_), TAG, "pcnt_unit_start");

    ESP_LOGI(TAG, "PCNT encoder init OK: A=%d, B=%d, ticks/rev=%.1f",
             (int)pulse_pin_, (int)ctrl_pin_, ticks_per_rev_);
    return ESP_OK;
}

// ----- RESET -----
void PcntEncoder::reset()
{
    if (unit_) {
        pcnt_unit_clear_count(unit_);
    }
}

// ----- GET ΔTICKS -----
int32_t PcntEncoder::getAndClearDeltaTicks()
{
    if (!unit_) return 0;
    int value = 0;
    pcnt_unit_get_count(unit_, &value);
    pcnt_unit_clear_count(unit_);
    return (int32_t)value;
}