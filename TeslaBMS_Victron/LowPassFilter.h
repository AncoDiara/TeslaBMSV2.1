#pragma once
#include <Arduino.h>

/*
  One-pole low-pass filter for the current measurement.
  Same calculation as FilterOnePole (LOWPASS) from https://github.com/JonHub/Filters,
  but built into the sketch so no external "Filters" library is needed.
  (There are several libraries called "Filters"; the wrong one gave the error
   'FilterOnePole does not name a type'.)
*/
class LowPassFilter
{
  public:
    LowPassFilter(float cutoffHz, float initialValue = 0.0f)
    {
      setFrequency(cutoffHz);
      Y = initialValue;
      lastUS = 0;
      started = false;
    }

    void setFrequency(float cutoffHz)
    {
      tauUS = (1.0f / (TWO_PI * cutoffHz)) * 1e6f; // tau = 1/omega, in microseconds
    }

    float input(float x)
    {
      uint32_t now = micros();
      if (!started)
      {
        started = true;
        lastUS = now;
        Y = x;
        return Y;
      }
      float elapsedUS = (float)(now - lastUS);
      lastUS = now;
      if (elapsedUS <= 0.0f) return Y;
      float ampFactor = expf(-elapsedUS / tauUS); // ~1 when called quickly
      Y = (1.0f - ampFactor) * x + ampFactor * Y;
      return Y;
    }

    float output()
    {
      return Y;
    }

    void setToNewValue(float v)
    {
      Y = v;
    }

  private:
    float tauUS;
    float Y;
    uint32_t lastUS;
    bool started;
};
