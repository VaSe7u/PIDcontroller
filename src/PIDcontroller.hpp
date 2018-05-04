#pragma once
#include <stdint.h>
#include <stdlib.h>

struct PIDconfig {
  float p = 0.0f;
  float i = 0.0f;
  float d = 0.0f;
  bool state = false;
  bool direct = true;
  uint32_t updatePeriod = 1000;
  float minimalOutput = 0.0f;
  float maximalOutput = 100.0f;
  float maximalNegativeIntegralSum = 0.0f;
  float maximalPositiveIntegralSum = 100.0f;
  float rampRateLimit = 0.0f;
};

class PIDcontroller {
public:
  PIDcontroller();
  bool config(const PIDconfig& pidConfig);

  float calculate(const float setpoint, const float input, float output = 0.0f);

  void on();
  void off();
  bool getState() const;

  bool setP(const float p);
  float getP() const;
  bool setI(const float i);
  float getI() const;
  bool setD(const float d);
  float getD() const;

  void setDirect();
  void setReverse();
  bool getDirect() const;

  void setUpdatePeriod(const uint32_t updatePeriod);
  uint32_t getUpdatePeriod() const;

  bool setOutputLimits(const float outputLimits);
  bool setOutputLimits(const float minimalOutput, const float maximalOutput);

  bool setMaximalIntegralSums(const float maximalIntegralSum);
  bool setMaximalIntegralSums(const float maximalNegativeIntegralSum,
                              const float maximalPositiveIntegralSum);

  void setRampRateLimit(const float rampRateLimit);

  void unwindIntegralSum(const float input = 0.0f, float output = 0.0f);

private:
  bool _state = false;
  bool _firstRun = true;
  bool _direct = true;

  // P
  float _p = 0.0f;
  float _pOriginal = 0.0f;

  // I
  float _i = 0.0f;
  float _iOriginal = 0.0f;
  float _integralSum = 0.0f;
  float _maximalNegativeIntegralSum = 0.0f;
  float _maximalPositiveIntegralSum = 100.0f;

  // D
  float _d = 0.0f;
  float _dOriginal = 0.0f;
  float _lastInput = 0.0f;

  float _rampRateLimit = 0.0f;
  float _lastOutput = 0.0f;

  float _minimalOutput = 0.0f;
  float _maximalOutput = 100.0f;

  uint32_t _updatePeriod = 1000;

  void _reverseDirection();
  static float _clamp(const float value, const float minimal, const float maximal);
};



