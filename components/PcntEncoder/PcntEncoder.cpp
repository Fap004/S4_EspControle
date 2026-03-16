#include "PcntEncoder.h"
#include "esp_log.h"
#include "esp_check.h"   // <-- pour ESP_RETURN_ON_ERROR

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
    if (unitA_ || unitB_) {
        ESP_LOGW(TAG_Q, "Already initialized");
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(configGpioInputs_(), TAG_Q, "gpio_config");

    // 1) Créer 2 unités PCNT (une par voie)
    pcnt_unit_config_t ucfg = {};
    ucfg.high_limit = high_limit_;
    ucfg.low_limit  = low_limit_;

    ESP_RETURN_ON_ERROR(pcnt_new_unit(&ucfg, &unitA_), TAG_Q, "pcnt_new_unit A");
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&ucfg, &unitB_), TAG_Q, "pcnt_new_unit B");

    // 2) Canal A : edge = A, level = B
    pcnt_chan_config_t cA = {};
    cA.edge_gpio_num  = pinA_;
    cA.level_gpio_num = pinB_;
    ESP_RETURN_ON_ERROR(pcnt_new_channel(unitA_, &cA, &chA_), TAG_Q, "new_channel A");

    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_edge_action(
            chA_,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE),
        TAG_Q, "edge_action A");

    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_level_action(
            chA_,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
        TAG_Q, "level_action A");

    // 3) Canal B : edge = B, level = A (symétrique)
    pcnt_chan_config_t cB = {};
    cB.edge_gpio_num  = pinB_;
    cB.level_gpio_num = pinA_;
    ESP_RETURN_ON_ERROR(pcnt_new_channel(unitB_, &cB, &chB_), TAG_Q, "new_channel B");

    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_edge_action(
            chB_,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE),
        TAG_Q, "edge_action B");

    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_level_action(
            chB_,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
        TAG_Q, "level_action B");

    // 4) Glitch filter (anti-rebonds) pour les 2 unités
    //    Si la valeur est hors-plage sur cette cible, on tente des valeurs dégressives,
    //    puis on désactive le filtre pour éviter le crash en init.
    if (glitch_ns_ > 0) {
        // Liste de candidats (ns) du plus "large" au plus "court"
        static const uint32_t candidates_ns[] = {
            // commencez "haut", descendez si nécessaire
            150000, 120000, 100000, 80000, 60000, 50000, 30000, 20000, 10000, 5000, 2000, 1000
        };
        bool set_ok_A = false, set_ok_B = false;

        for (uint32_t ns : candidates_ns) {
            if (ns > glitch_ns_) continue; // ne pas dépasser la demande initiale
            pcnt_glitch_filter_config_t f = { .max_glitch_ns = ns };

            esp_err_t errA = pcnt_unit_set_glitch_filter(unitA_, &f);
            esp_err_t errB = pcnt_unit_set_glitch_filter(unitB_, &f);

            if (errA == ESP_OK) set_ok_A = true;
            if (errB == ESP_OK) set_ok_B = true;

            if (set_ok_A && set_ok_B) {
                glitch_ns_ = ns;
                ESP_LOGI(TAG_Q, "glitch filter set to %u ns on A & B", ns);
                break;
            } else {
                // si l’un des deux échoue, rollback celui qui avait réussi pour retenter une valeur plus basse
                if (set_ok_A) {
                    // désactiver pour rester cohérent le temps qu'on trouve une valeur commune
                    pcnt_glitch_filter_config_t f0 = { .max_glitch_ns = 0 };
                    (void)pcnt_unit_set_glitch_filter(unitA_, &f0);
                    set_ok_A = false;
                }
                if (set_ok_B) {
                    pcnt_glitch_filter_config_t f0 = { .max_glitch_ns = 0 };
                    (void)pcnt_unit_set_glitch_filter(unitB_, &f0);
                    set_ok_B = false;
                }
            }
        }

        if (!set_ok_A || !set_ok_B) {
            ESP_LOGW(TAG_Q,
                     "glitch %u ns out-of-range on this chip (A_ok=%d,B_ok=%d) -> disabling filter",
                     glitch_ns_, (int)set_ok_A, (int)set_ok_B);
            // désactiver le filtre sur les deux unités
            pcnt_glitch_filter_config_t f0 = { .max_glitch_ns = 0 };
            (void)pcnt_unit_set_glitch_filter(unitA_, &f0);
            (void)pcnt_unit_set_glitch_filter(unitB_, &f0);
            glitch_ns_ = 0;
        }
    }

    // 5) Démarrer les deux unités
    ESP_RETURN_ON_ERROR(pcnt_unit_enable(unitA_), TAG_Q, "enable A");
    ESP_RETURN_ON_ERROR(pcnt_unit_enable(unitB_), TAG_Q, "enable B");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(unitA_), TAG_Q, "clear A");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(unitB_), TAG_Q, "clear B");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(unitA_), TAG_Q, "start A");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(unitB_), TAG_Q, "start B");
    started_ = true;

    // Lecture initiale
    pcnt_unit_get_count(unitA_, &last_raw_A_);
    pcnt_unit_get_count(unitB_, &last_raw_B_);
    accum_total_ = 0;
    rpm_lpf_     = 0.0f;

    ESP_LOGI(TAG_Q, "PCNT QUAD OK: A=%d, B=%d, TPR=%.1f, glitch=%u ns, inverted=%d",
             (int)pinA_, (int)pinB_, ticks_per_rev_, glitch_ns_, inverted_);
    return ESP_OK;
}

void PcntEncoder::deinit()
{
    if (!unitA_ && !unitB_) return;

    if (started_) {
        if (unitA_) pcnt_unit_stop(unitA_);
        if (unitB_) pcnt_unit_stop(unitB_);
        started_ = false;
    }
    if (chA_) { pcnt_del_channel(chA_); chA_ = nullptr; }
    if (chB_) { pcnt_del_channel(chB_); chB_ = nullptr; }
    if (unitA_) { pcnt_del_unit(unitA_); unitA_ = nullptr; }
    if (unitB_) { pcnt_del_unit(unitB_); unitB_ = nullptr; }
}

void PcntEncoder::reset()
{
    if (unitA_) { pcnt_unit_clear_count(unitA_); pcnt_unit_get_count(unitA_, &last_raw_A_); }
    if (unitB_) { pcnt_unit_clear_count(unitB_); pcnt_unit_get_count(unitB_, &last_raw_B_); }
    accum_total_ = 0;
    rpm_lpf_     = 0.0f;
}

void PcntEncoder::recenterIfNearLimit_(int rawA, int rawB)
{
    const int TH = 30000; // marge avant ±32k
    if ((rawA > TH) || (rawA < -TH)) { pcnt_unit_clear_count(unitA_); last_raw_A_ = 0; }
    if ((rawB > TH) || (rawB < -TH)) { pcnt_unit_clear_count(unitB_); last_raw_B_ = 0; }
}

int32_t PcntEncoder::getDelta()
{
    if (!unitA_ || !unitB_) return 0;

    int rawA = 0, rawB = 0;
    pcnt_unit_get_count(unitA_, &rawA);
    pcnt_unit_get_count(unitB_, &rawB);

    int32_t dA = rawA - last_raw_A_;
    int32_t dB = rawB - last_raw_B_;
    last_raw_A_ = rawA;
    last_raw_B_ = rawB;

    // Somme quadrature (A+B)
    int32_t delta = dA + dB;

    if (inverted_) delta = -delta;

    accum_total_ += delta;

    recenterIfNearLimit_(rawA, rawB);
    return delta;
}

float PcntEncoder::rpm(float dt_s, bool lpf)
{
    if (!(dt_s > 0.0f) || !std::isfinite(dt_s)) return rpm_lpf_;

    const int32_t dticks = getDelta();
    const float revs     = (ticks_per_rev_ > 0.f) ? (dticks / ticks_per_rev_) : 0.f;
    const float rpm_i    = (revs / dt_s) * 60.0f;

    if (!lpf) { rpm_lpf_ = rpm_i; return rpm_i; }

    rpm_lpf_ = lp_alpha_ * rpm_lpf_ + (1.0f - lp_alpha_) * rpm_i;
    return rpm_lpf_;
}

// --- Compat shim optionnel si WheelController appelle encore cette méthode :
int32_t PcntEncoder::getAndClearDeltaTicks() { return getDelta(); }