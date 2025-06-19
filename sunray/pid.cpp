/*  
   How to find out P,I,D:
    1. Increase P until system starts to oscillate
    2. Set I =0.6 * P and D = 0.125 * P 
   
*/

#include "pid.h"
#include "config.h"

PID::PID()
{
  consoleWarnTimeout = 0;
  lastControlTime = 0;
  output_ramp = 0;
  yold = 0;
}
    
PID::PID(float Kp, float Ki, float Kd){
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}


void PID::reset(void) {
  eold = 0;
  esum = 0;
  yold = 0;
  lastControlTime = millis();
}

float PID::compute() {
    unsigned long now = millis();
    float dt = (now - lastControlTime) * 1e-3f; // dt in Sekunden

    if (dt <= 0.0f) dt = 1e-3f;  // Schutz gegen Division durch 0

    float e = w - x;

    esum += e * dt;                            // I-Anteil (mit Zeitgewichtung)
    float dedt = (e - eold) / dt;              // D-Anteil (mit Zeitgewichtung)

    float u = Kp * e + Ki * esum + Kd * dedt;  // PID-Gleichung

    // Begrenzung
    if (u > y_max) u = y_max;
    else if (u < y_min) u = y_min;

    // Rampenbegrenzung
    if (output_ramp > 0) {
        float max_delta = output_ramp * dt;
        float delta = u - yold;
        if (delta > max_delta) u = yold + max_delta;
        else if (delta < -max_delta) u = yold - max_delta;
    }

    // Speichern
    y = u;
    yold = y;
    eold = e;
    lastControlTime = now;

    return y;
}

// ---------------------------------

VelocityPID::VelocityPID()
{
  output_ramp = 0;
}
    
VelocityPID::VelocityPID(float Kp, float Ki, float Kd){
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}


float VelocityPID::compute()
{   
  unsigned long now = micros();
  Ta = ((now - lastControlTime) / 1000000.0);
  lastControlTime = now;
  if (Ta > 1.0) Ta = 1.0;   // should only happen for the very first call

  // compute error
  float e = (w - x);

  // compute max/min output
  if (w < 0) { y_min = -max_output; y_max = 0; }
  if (w > 0) { y_min = 0; y_max = max_output; }     

  y = yold
      + Kp * (e - eold1)
      + Ki * Ta * e
      + Kd/Ta * (e - 2* eold1 + eold2);
     
  // restrict output to min/max 
  if (y > y_max) y = y_max;
  if (y < y_min) y = y_min; 

  // if output ramp defined
  if(output_ramp > 0){
      // limit the acceleration by ramping the output
      float output_rate = (y - yold)/Ta;
      if (output_rate > output_ramp)
          y = yold + output_ramp*Ta;
      else if (output_rate < -output_ramp)
          y = yold - output_ramp*Ta;
  }

  // save variable for next time
  eold2 = eold1;
  eold1 = e;
  yold = y ;  
  
  return y;
}

