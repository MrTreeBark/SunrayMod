#include "lowpass_filter.h"

LowPassFilter::LowPassFilter(float time_constant)
    : Tf(time_constant)
    , y_prev(0.0f)
{
    timestamp_prev = millis();
}

float LowPassFilter::operator()(float x)
{
    unsigned long timestamp = millis();
    float dt = (timestamp - timestamp_prev) * 1e-3f;  // dt in Sekunden
    timestamp_prev = timestamp;

    // Begrenzung gegen Ausreißer
    if (dt < 1e-6f) dt = 1e-6f;
    if (dt > 1.0f)  dt = 1.0f;

    // Zeitinvariantes Alpha
    float alpha = 1.0f - expf(-dt / Tf);

    // Filter anwenden
    y_prev = alpha * x + (1.0f - alpha) * y_prev;
    return y_prev;
}

void LowPassFilter::reset()
{
    y_prev = 0.0f;
    timestamp_prev = millis();
}