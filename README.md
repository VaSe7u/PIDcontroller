**PID controller library.**

[![release](https://img.shields.io/badge/release-0.1.0-yellow.svg)](https://github.com/VaSe7u/PIDcontroller/releases)
[![license](https://img.shields.io/github/license/mashape/apistatus.svg?maxAge=2592000)](https://opensource.org/licenses/mit-license.php)

This library was heavily infuenced by [Arduino-PID-Library][ArduinoPID] and [this][ArduinoPID_guide] guide. The difference is that it doesn't use references (this allows the setpoint, input and output to be integers) and is Arduino independent.


Resources
=========
 - [Examples][examples]
 - [Latest release][latest release]


Features
========
 - Robust.
 - Arduino independent.


Quick start
===========
```c++
PIDcontroller pid;
pid.setP(1.0f);
pid.setD(0.1f);
pid.setDirect();
pid.setUpdatePeriod(); // Needed for calculating the integral and derivative terms.
pid.setOutputLimits(0, 255);
pid.on();

byte setpoint = 3;
float input = 0.0f;
float output = 0.0f;

...
// measure input
...

if (timeToCalculate) {
  // Calculating the PID controller in regular intervals is handled by the user.
  output = pid.calculate(setpoint, input, output);
}

...
// set output
// change setpoint
...
```

License
=======
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

[doxygen classes]: https://VaSe7u.github.io/PIDcontroller/doc/Doxygen/html/annotated.html
[examples]: https://github.com/VaSe7u/PIDcontroller/tree/master/examples
[latest release]: https://github.com/VaSe7u/PIDcontroller/releases/latest
[ArduinoPID]: https://github.com/br3ttb/Arduino-PID-Library
[ArduinoPID_guide]: http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/