/*
The MIT License (MIT)
Copyright (c) 2016 Vasil Kalchev
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/**
@file
PIDcontroller's interface file.
@author Vasil Kalchev
@date 2018
@copyright The MIT License
@version 1.4.0
*/

#pragma once
#include <stdint.h>
#include <stdlib.h>

struct PIDconfig {
  float p = 0.0f; ///< /explanation/
  float i = 0.0f; ///< /explanation/
  float d = 0.0f; ///< /explanation/
  bool state = false; ///< /explanation/
  bool direct = true; ///< /explanation/
  uint32_t updatePeriod = 1000; ///< /explanation/ microseconds
  float minimalOutput = 0.0f; ///< /explanation/
  float maximalOutput = 100.0f; ///< /explanation/
  float maximalNegativeIntegralSum = 0.0f; ///< /explanation/
  float maximalPositiveIntegralSum = 100.0f; ///< /explanation/
  float rampRateLimit = 0.0f; ///< /explanation/
};

/// brief
/**
notbrief
@param[in] /which param/ - /explanation/
@returns /what/
@note /some note/
@see /function/
*/
class PIDcontroller {
public:
  /// brief
  /**
  notbrief
  @param[in] /which param/ - /explanation/
  @returns /what/
  @note /some note/
  @see /function/
  */
  PIDcontroller();
  
  /// brief
  /**
  notbrief
  @param[in] /which param/ - /explanation/
  @returns /what/
  @note /some note/
  @see /function/
  */
  bool config(const PIDconfig& pidConfig);

  /// brief
  /**
  notbrief
  @param[in] /which param/ - /explanation/
  @returns /what/
  @note /some note/
  @see /function/
  */
  float calculate(const float setpoint, const float input, float output = 0.0f);
  
  /// brief
  /**
  notbrief
  @param[in] /which param/ - /explanation/
  @returns /what/
  @note /some note/
  @see /function/
  */
  void on();
  
  /// brief
  /**
  notbrief
  @param[in] /which param/ - /explanation/
  @returns /what/
  @note /some note/
  @see /function/
  */
  void off();
  
  /// brief
  /**
  notbrief
  @param[in] /which param/ - /explanation/
  @returns /what/
  @note /some note/
  @see /function/
  */
  bool getState() const;

  /// brief
  /**
  notbrief
  @param[in] /which param/ - /explanation/
  @returns /what/
  @note /some note/
  @see /function/
  */
  bool setP(const float p);
  
  /// brief
  /**
  notbrief
  @param[in] /which param/ - /explanation/
  @returns /what/
  @note /some note/
  @see /function/
  */
  float getP() const;
  
  /// brief
  /**
  notbrief
  @param[in] /which param/ - /explanation/
  @returns /what/
  @note /some note/
  @see /function/
  */
  bool setI(const float i);
  
  /// brief
  /**
  notbrief
  @param[in] /which param/ - /explanation/
  @returns /what/
  @note /some note/
  @see /function/
  */
  float getI() const;
  
  /// brief
  /**
  notbrief
  @param[in] /which param/ - /explanation/
  @returns /what/
  @note /some note/
  @see /function/
  */
  bool setD(const float d);
  
  /// brief
  /**
  notbrief
  @param[in] /which param/ - /explanation/
  @returns /what/
  @note /some note/
  @see /function/
  */
  float getD() const;

  /// brief
  /**
  notbrief
  @param[in] /which param/ - /explanation/
  @returns /what/
  @note /some note/
  @see /function/
  */
  void setDirect();
  
  /// brief
  /**
  notbrief
  @param[in] /which param/ - /explanation/
  @returns /what/
  @note /some note/
  @see /function/
  */
  void setReverse();
  
  /// brief
  /**
  notbrief
  @param[in] /which param/ - /explanation/
  @returns /what/
  @note /some note/
  @see /function/
  */
  bool getDirect() const;

  /// brief
  /**
  notbrief
  @param[in] /which param/ - /explanation/
  @returns /what/
  @note /some note/ microseconds
  @see /function/
  */
  void setUpdatePeriod(const uint32_t updatePeriod);
  
  /// brief
  /**
  notbrief
  @param[in] /which param/ - /explanation/ microseconds
  @returns /what/
  @note /some note/
  @see /function/
  */
  uint32_t getUpdatePeriod() const;

  /// brief
  /**
  notbrief
  @param[in] /which param/ - /explanation/
  @returns /what/
  @note /some note/
  @see /function/
  */
  bool setOutputLimits(const float outputLimits);
  
  /// brief
  /**
  notbrief
  @param[in] /which param/ - /explanation/
  @returns /what/
  @note /some note/
  @see /function/
  */
  bool setOutputLimits(const float minimalOutput, const float maximalOutput);

  /// brief
  /**
  notbrief
  @param[in] /which param/ - /explanation/
  @returns /what/
  @note /some note/
  @see /function/
  */
  bool setMaximalIntegralSums(const float maximalIntegralSum);
  
  /// brief
  /**
  notbrief
  @param[in] /which param/ - /explanation/
  @returns /what/
  @note /some note/
  @see /function/
  */
  bool setMaximalIntegralSums(const float maximalNegativeIntegralSum,
                              const float maximalPositiveIntegralSum);

  /// brief
  /**
  notbrief
  @param[in] /which param/ - /explanation/
  @returns /what/
  @note /some note/
  @see /function/
  */
  void setRampRateLimit(const float rampRateLimit);

  /// brief
  /**
  notbrief
  @param[in] /which param/ - /explanation/
  @returns /what/
  @note /some note/
  @see /function/
  */
  void unwindIntegralSum(const float input = 0.0f, float output = 0.0f);

private:
  bool _state = false; ///< /explanation/
  bool _firstRun = true; ///< /explanation/
  bool _direct = true; ///< /explanation/

  // P
  float _p = 0.0f; ///< /explanation/
  float _pOriginal = 0.0f; ///< /explanation/

  // I
  float _i = 0.0f; ///< /explanation/
  float _iOriginal = 0.0f; ///< /explanation/
  float _integralSum = 0.0f; ///< /explanation/
  float _maximalNegativeIntegralSum = 0.0f; ///< /explanation/
  float _maximalPositiveIntegralSum = 100.0f; ///< /explanation/

  // D
  float _d = 0.0f; ///< /explanation/
  float _dOriginal = 0.0f; ///< /explanation/
  float _lastInput = 0.0f; ///< /explanation/

  float _rampRateLimit = 0.0f; ///< /explanation/
  float _lastOutput = 0.0f; ///< /explanation/

  float _minimalOutput = 0.0f; ///< /explanation/
  float _maximalOutput = 100.0f; ///< /explanation/

  uint32_t _updatePeriod = 1000; ///< /explanation/ microseconds

  /// brief
  /**
  notbrief
  @param[in] /which param/ - /explanation/
  @returns /what/
  @note /some note/
  @see /function/
  */
  void _reverseDirection();
  
  /// brief
  /**
  notbrief
  @param[in] /which param/ - /explanation/
  @returns /what/
  @note /some note/
  @see /function/
  */
  static float _clamp(const float value, const float minimal, const float maximal);
};
