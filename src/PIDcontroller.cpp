#include "PIDcontroller.hpp"

PIDcontroller::PIDcontroller() {}

bool PIDcontroller::config(const PIDconfig& pidConfig) {
  bool s = true;
  s &= setP(pidConfig.p);
  s &= setI(pidConfig.i);
  s &= setD(pidConfig.d);
  if (pidConfig.state) { on(); } else { off(); }
  if (pidConfig.direct) { setDirect(); } else { setReverse(); }
  setUpdatePeriod(pidConfig.updatePeriod);
  s &= setOutputLimits(pidConfig.minimalOutput, pidConfig.maximalOutput);
  s &= setMaximalIntegralSums(pidConfig.maximalNegativeIntegralSum,
                              pidConfig.maximalPositiveIntegralSum);
  setRampRateLimit(pidConfig.rampRateLimit);
  return s;
}

float PIDcontroller::calculate(const float setpoint, const float input, float output) {
  if (!_state) return output;
  if (_firstRun) { _integralSum = output; _lastInput = input; _firstRun = false; }

  const float error = setpoint - input;

  _integralSum += error * _i;
  _integralSum = PIDcontroller::_clamp(_integralSum,
                                       _maximalNegativeIntegralSum,
                                       _maximalPositiveIntegralSum);

  //       ------P-----   ------I-----   -------------D-------------
  output = (error * _p) + _integralSum - ((input - _lastInput) * _d);

  if (_rampRateLimit) {
    output = PIDcontroller::_clamp(output,
                                   (_lastOutput - _rampRateLimit),
                                   (_lastOutput + _rampRateLimit));
  }
  output = PIDcontroller::_clamp(output, _minimalOutput, _maximalOutput);

  _lastInput = input;
  _lastOutput = output;
  return output;
}

void PIDcontroller::on() {
  if (_state == false) _firstRun = true;
  _state = true;
}

void PIDcontroller::off() {
  _state = false;
}

bool PIDcontroller::getState() const {
  return _state;
}


bool PIDcontroller::setP(const float p) {
  if (p < 0.0f) return false;
  _pOriginal = p;
  _p = p;
  if (_direct == false) {
    _p = (0.0f - _p);
  }
  return true;
}

float PIDcontroller::getP() const {
  return _pOriginal;
}

bool PIDcontroller::setI(const float i) {
  if (i < 0.0f) return false;
  _iOriginal = i;
  float sampleTime = (float)_updatePeriod / 1000000.0f;
  _i = i * sampleTime;
  if (_direct == false) {
    _i = (0.0f - _i);
  }
  return true;
}

float PIDcontroller::getI() const {
  return _iOriginal;
}

bool PIDcontroller::setD(const float d) {
  if (d < 0.0f) return false;
  _dOriginal = d;
  float sampleTime = (float)_updatePeriod / 1000000.0f;
  _d = d / sampleTime;
  if (_direct == false) {
    _d = (0.0f - _d);
  }
  return true;
}

float PIDcontroller::getD() const {
  return _dOriginal;
}


void PIDcontroller::setDirect() {
  if (_direct == false) {
    _reverseDirection();
    _direct = true;
  }
}

void PIDcontroller::setReverse() {
  if (_direct == true) {
    _reverseDirection();
    _direct = false;
  }
}

bool PIDcontroller::getDirect() const {
  return _direct;
}


void PIDcontroller::setUpdatePeriod(const uint32_t updatePeriod) {
  float ratio = (float)updatePeriod / (float)_updatePeriod;
  _i *= ratio;
  _d /= ratio;
  _updatePeriod = updatePeriod;
}

uint32_t PIDcontroller::getUpdatePeriod() const {
  return _updatePeriod;
}


bool PIDcontroller::setOutputLimits(const float outputLimits) {
  const float max = abs(outputLimits);
  const float min = 0.0f - max;
  return setOutputLimits(min, max);
}

bool PIDcontroller::setOutputLimits(const float minimalOutput, const float maximalOutput) {
  if (minimalOutput >= maximalOutput) return false;
  _minimalOutput = minimalOutput;
  _maximalOutput = maximalOutput;
  if (_maximalNegativeIntegralSum < _minimalOutput) {
    _maximalNegativeIntegralSum = _minimalOutput;
  }
  if (_maximalPositiveIntegralSum > _maximalOutput) {
    _maximalPositiveIntegralSum = maximalOutput;
  }
  _integralSum = PIDcontroller::_clamp(_integralSum,
                                       _maximalNegativeIntegralSum,
                                       _maximalPositiveIntegralSum);
  return true;
}


bool PIDcontroller::setMaximalIntegralSums(const float maximalIntegralSum) {
  const float max = abs(maximalIntegralSum);
  const float min = 0.0f - max;
  return setMaximalIntegralSums(min, max);
}

bool PIDcontroller::setMaximalIntegralSums(const float maximalNegativeIntegralSum,
    const float maximalPositiveIntegralSum) {
  if (maximalNegativeIntegralSum >= maximalPositiveIntegralSum) return false;
  if (_maximalNegativeIntegralSum < _minimalOutput
      || _maximalPositiveIntegralSum > _maximalOutput) { return false; }
  _maximalNegativeIntegralSum = maximalNegativeIntegralSum;
  _maximalPositiveIntegralSum = maximalPositiveIntegralSum;
  _integralSum = PIDcontroller::_clamp(_integralSum,
                                       _maximalNegativeIntegralSum,
                                       _maximalPositiveIntegralSum);
  return true;
}


void PIDcontroller::setRampRateLimit(const float rampRateLimit) {
  const float rrl = abs(rampRateLimit);
  if (rrl >= _maximalOutput) {
    _rampRateLimit = 0.0f;
  } else {
    _rampRateLimit = rrl;
  }
}


void PIDcontroller::unwindIntegralSum(const float input, float output) {
  _lastInput = input;
  output = PIDcontroller::_clamp(output, _minimalOutput, _maximalOutput);
  _lastOutput = output;
  _integralSum = output;
  _integralSum = PIDcontroller::_clamp(_integralSum,
                                       _maximalNegativeIntegralSum,
                                       _maximalPositiveIntegralSum);
}

void PIDcontroller::_reverseDirection() {
  _p = (0.0f - _p);
  _i = (0.0f - _i);
  _d = (0.0f - _d);
}

float PIDcontroller::_clamp(const float value, const float minimal, const float maximal) {
  if (value < minimal) return minimal;
  else if (value > maximal) return maximal;
  else return value;
}