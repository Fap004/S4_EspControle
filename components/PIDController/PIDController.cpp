#include "PIDController.h"

PIDController::PIDController(const Gains& g, const Limits& l)
    : g_(g), l_(l) {}

void PIDController::reset(float init_meas)
{
    integ_     = 0.0f;
    prev_meas_ = init_meas;
    have_prev_ = false;
    d_filt_    = 0.0f;
}

void PIDController::setDerivFilterAlpha(float a)
{
    d_alpha_ = std::clamp(a, 0.0f, 1.0f);
}

float PIDController::compute(float target, float measured, float dt)
{
    if (!(std::isfinite(target) && std::isfinite(measured) && dt > 0.0f))
    {
        // Échantillon invalide : renvoyer la dernière sortie
        return last_u_;
    }

    // Erreur
    const float e = target - measured;

    // Proportionnel
    const float up = g_.Kp * e;

    // Dérivé sur la mesure (−d/dt mesuré) + filtre 1er ordre
    float d_raw = 0.0f;
    if (have_prev_)
    {
        d_raw  = (prev_meas_ - measured) / dt;  // signe voulu
        // filtre exponentiel: d_filt = (1-α)*d_raw + α*d_filt
        d_filt_ = (1.0f - d_alpha_) * d_raw + d_alpha_ * d_filt_;
    }
    else
    {
        d_filt_    = 0.0f;
        have_prev_ = true;
    }
    prev_meas_ = measured;
    const float ud = g_.Kd * d_filt_;

    // Intégral (pré-calculé, validé après anti-windup conditionnel)
    float integ_new = integ_ + g_.Ki * e * dt;
    integ_new = std::clamp(integ_new, l_.i_min, l_.i_max);

    // Sortie non saturée
    const float u_unsat = up + integ_new + ud;

    // Saturation
    const float u_sat = std::clamp(u_unsat, l_.u_min, l_.u_max);

    // Anti-windup conditionnel
    const bool saturated = (u_sat != u_unsat);
    if (saturated) {
        const bool pushing_same_dir =
            ((u_unsat > l_.u_max) && (e > 0.0f)) ||
            ((u_unsat < l_.u_min) && (e < 0.0f));

        if (!pushing_same_dir)
        {
            // On accepte l'intégrale recalculée (on dégage l'erreur cumulée)
            integ_ = integ_new;
        }
        // Sinon : freeze (on ne modifie pas integ_)
    }
    else
    {
        // Pas de saturation : intégration normale
        integ_ = integ_new;
    }

    last_u_ = u_sat;
    return last_u_;
}

const PIDController::Gains& PIDController::gains() const  { return g_; }
const PIDController::Limits& PIDController::limits() const { return l_; }
void PIDController::setGains(const Gains& g)   { g_ = g; }
void PIDController::setLimits(const Limits& l) { l_ = l; }