#pragma once
#include <algorithm>
#include <cmath>

class PIDController
{
public:
    struct Gains { float Kp = 0, Ki = 0, Kd = 0; };
    struct Limits {
        float u_min = -1.0f, u_max = +1.0f;  // saturation sortie
        float i_min = -0.5f, i_max = +0.5f;  // clamp intégrale
    };

    PIDController(const Gains& g, const Limits& l);

    // Remet à zéro l'état interne (intégrateur, filtre D, etc.)
    void reset(float init_meas = 0.0f);

    // Réglage du filtre dérivatif (alpha dans [0..1])
    void setDerivFilterAlpha(float a);

    // Calcule la commande
    float compute(float target, float measured, float dt);

    // Accesseurs / mutateurs
    const Gains&  gains()  const;
    const Limits& limits() const;
    void setGains(const Gains& g);
    void setLimits(const Limits& l);

private:
    Gains  g_;
    Limits l_;

    float integ_     = 0.0f;
    float prev_meas_ = 0.0f;
    bool  have_prev_ = false;

    // Filtre dérivatif
    float d_filt_   = 0.0f;
    float d_alpha_  = 0.85f;   // 0 = pas de filtre, 0.85 = filtrage fort

    float last_u_   = 0.0f;
};