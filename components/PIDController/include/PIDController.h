#pragma once
#include <algorithm>
#include <cmath>

class PIDController
{
public:
    struct Gains { float Kp=0, Ki=0, Kd=0; };
    struct Limits {
        float u_min=-1.0f, u_max=+1.0f;  // saturation sortie
        float i_min=-0.5f, i_max=+0.5f;  // clamp intégrale
    };

    PIDController(const Gains& g, const Limits& l) : g_(g), l_(l) {}

    void reset(float init_meas = 0.0f) {
        integ_ = 0.0f;
        prev_meas_ = init_meas;
        have_prev_ = false;
        d_filt_ = 0.0f;
    }

    // --- Nouveau : réglage du filtre dérivatif (alpha dans [0..1]) ---
    void setDerivFilterAlpha(float a) { d_alpha_ = std::clamp(a, 0.0f, 1.0f); }

    float compute(float target, float measured, float dt)
    {
        if (!(std::isfinite(target) && std::isfinite(measured) && dt > 0.0f)) {
            // Échantillon invalide : renvoyer la dernière sortie ou 0
            return last_u_;
        }

        // Erreur
        const float e = target - measured;

        // Proportionnel
        const float up = g_.Kp * e;

        // Dérivé sur la mesure (−d/dt mesuré) + filtre 1er ordre
        float d_raw = 0.0f;
        if (have_prev_) {
            d_raw = (prev_meas_ - measured) / dt;  // signe voulu
            // filtre exponentiel: d_filt = (1-α)*d_raw + α*d_filt
            d_filt_ = (1.0f - d_alpha_) * d_raw + d_alpha_ * d_filt_;
        } else {
            d_filt_ = 0.0f;
            have_prev_ = true;
        }
        prev_meas_ = measured;
        const float ud = g_.Kd * d_filt_;

        // Intégral (pré-calculé, sera validé après anti-windup conditionnel)
        float integ_new = integ_ + g_.Ki * e * dt;
        integ_new = std::clamp(integ_new, l_.i_min, l_.i_max);

        // Sortie "non saturée"
        float u_unsat = up + integ_new + ud;

        // Saturation
        float u_sat = std::clamp(u_unsat, l_.u_min, l_.u_max);

        // --- Anti-windup conditionnel ---
        // Si on est saturé ET que l'erreur pousse encore DANS le sens de la saturation,
        // on FREEZE l'intégrale (on garde l'ancienne) ; sinon on accepte integ_new.
        bool saturated = (u_sat != u_unsat);
        if (saturated) {
            // signe de la "volonté" de la sortie vs signe de l'erreur
            // si u_unsat > u_max et e > 0  -> on pousserait encore vers le haut  -> freeze
            // si u_unsat < u_min et e < 0  -> on pousserait encore vers le bas   -> freeze
            bool pushing_same_dir =
                ((u_unsat > l_.u_max) && (e > 0.0f)) ||
                ((u_unsat < l_.u_min) && (e < 0.0f));

            if (pushing_same_dir) {
                // ne pas intégrer davantage
                // (option : back-calculation au lieu du freeze)
                // integ_ = integ_; // FREEZE
            } else {
                // on accepte l'intégrale recalculée (on dégare l'erreur cumulée)
                integ_ = integ_new;
            }
        } else {
            integ_ = integ_new;
        }

        last_u_ = u_sat;
        return last_u_;
    }

    const Gains&  gains() const  { return g_; }
    const Limits& limits() const { return l_; }
    void setGains(const Gains& g)   { g_ = g; }
    void setLimits(const Limits& l) { l_ = l; }

private:
    Gains  g_;
    Limits l_;

    float integ_     = 0.0f;
    float prev_meas_ = 0.0f;
    bool  have_prev_ = false;

    // --- Nouveau : filtre dérivatif ---
    float d_filt_   = 0.0f;
    float d_alpha_  = 0.85f;   // 0=pas de filtre, 0.85=filtrage fort

    float last_u_   = 0.0f;
};
