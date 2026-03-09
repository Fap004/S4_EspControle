#pragma once
#include <algorithm>
#include <cmath>

class PIDController
{
public:
    struct Gains {
        float Kp = 0.0f;
        float Ki = 0.0f;
        float Kd = 0.0f;
    };

    struct Limits {
        float u_min = -1.0f;   // borne min commande (duty signé)
        float u_max = +1.0f;   // borne max commande
        float i_min = -0.5f;   // borne anti-windup intégrale
        float i_max = +0.5f;
    };

    PIDController(const Gains& g, const Limits& l) : g_(g), l_(l) {}

    // Réinitialise les états (intégrateur / dérivateur)
    void reset(float init_meas = 0.0f) {
        integ_ = 0.0f;
        prev_meas_ = init_meas;
        have_prev_ = false;
    }

    // Compute : target vs measured -> u signé dans [u_min..u_max]
    float compute(float target, float measured, float dt)
    {
        // Erreur (convention : e = target - measured)
        float e = target - measured;

        // Proportionnel
        float up = g_.Kp * e;

        // Intégral avec anti-windup par clamp
        integ_ += g_.Ki * e * dt;
        integ_ = std::clamp(integ_, l_.i_min, l_.i_max);

        // Dérivé (sur la mesure -> "derivative on measurement" pour moins de bruit)
        float deriv = 0.0f;
        if (have_prev_ && dt > 0.0f) {
            deriv = (prev_meas_ - measured) / dt; // signe inversé (−d/dt mesuré)
        }
        float ud = g_.Kd * deriv;
        prev_meas_ = measured;
        have_prev_ = true;

        // Somme et saturation
        float u = up + integ_ + ud;
        u = std::clamp(u, l_.u_min, l_.u_max);
        return u;
    }

    const Gains&  gains()  const { return g_; }
    const Limits& limits() const { return l_; }
    void setGains(const Gains& g)  { g_ = g; }
    void setLimits(const Limits& l){ l_ = l; }

private:
    Gains g_;
    Limits l_;

    float integ_     = 0.0f;
    float prev_meas_ = 0.0f;
    bool  have_prev_ = false;
};