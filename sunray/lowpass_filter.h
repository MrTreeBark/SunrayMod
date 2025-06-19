#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#include <Arduino.h>
#include <math.h>

class LowPassFilter
{
public:
    LowPassFilter(float Tf = 0.001f);   // default timeconstant in seconds
    ~LowPassFilter() = default;

    float operator() (float x);
    float Tf; // timeconstant in seconds
    void reset();

protected:
    unsigned long timestamp_prev;  // last call
    float y_prev;                  // last filter value
};

#endif