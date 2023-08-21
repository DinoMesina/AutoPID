#include "AutoPID_V2.h"

AutoPID_V2::AutoPID_V2(double *input, double *setpoint, double *output, double outputMin, double outputMax,
                 double Kp, double Ki, double Kd) {
  _input = input;
  _setpoint = setpoint;
  _output = output;
  _outputMin = outputMin;
  _outputMax = outputMax;
  setGains(Kp, Ki, Kd);
  _timeStep = 1000;
} // AutoPID_V2::AutoPID_V2

void AutoPID_V2::setGains(double Kp, double Ki, double Kd) {
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;
} // AutoPID_V2::setControllerParams

void AutoPID_V2::setBangBang(double bangOn, double bangOff) {
  _bangOn = bangOn;
  _bangOff = bangOff;
} // void AutoPID_V2::setBangBang

void AutoPID_V2::setBangBang(double bangRange) {
  setBangBang(bangRange, bangRange);
} // void AutoPID_V2::setBangBang

void AutoPID_V2::setOutputRange(double outputMin, double outputMax) {
  _outputMin = outputMin;
  _outputMax = outputMax;
} // void AutoPID_V2::setOutputRange

void AutoPID_V2::setTimeStep(unsigned long timeStep){
  _timeStep = timeStep;
}


bool AutoPID_V2::atSetPoint(double threshold) {
  return abs(*_setpoint - *_input) <= threshold;
} // bool AutoPID_V2::atSetPoint

void AutoPID_V2::run() {
  if (_stopped) {
    _stopped = false;
    reset();
  }
  // if bang thresholds are defined and we're outside of them, use bang-bang control
  if (_bangOn && ((*_setpoint - *_input) > _bangOn)) {
    *_output = _outputMax;
    _lastStep = millis();
  } else if (_bangOff && ((*_input - *_setpoint) > _bangOff)) {
    *_output = _outputMin;
    _lastStep = millis();
  } else {                                                    // otherwise use PID control
    unsigned long _dT = millis() - _lastStep;                 // calculate time since last update
    if (_dT >= _timeStep) {                                   // if long enough, do PID calculations
      _lastStep = millis();
      _dT /= 1000.0;                                          // delta time in seconds 
      double _error = *_setpoint - *_input;                   // error
      _proportional = _error * _Kp;                           // proportional 
      _derivative = ((*_input - _previousInput) / _dT) * _Kd; // derivative of input, it is not influenced by the setpoint changes 
      double PD = _proportional - _derivative;                // PD (-D because is referenced to the input values)
      _integral += ((_error + _previousError) / 2) * _dT;     // Riemann sum integral
      _integral = constrain(_integral, ((_outputMin - PD) / _Ki), ((_outputMax - PD) / _Ki)); // limit the integration sum to limit the output 
      double I = _integral * _Ki;                             // I
      _previousError = _error;
      _previousInput = *_input;
      *_output = constrain((PD + I), _outputMin, _outputMax);
    }
  }
} // void AutoPID_V2::run

void AutoPID_V2::stop() {
  _stopped = true;
  reset();
}
void AutoPID_V2::reset() {
  _lastStep = millis();
  _integral = 0.0;
  _previousError = 0.0;
}

bool AutoPID_V2::isStopped() {
  return _stopped;
}

double AutoPID_V2::getProportional() {
  return _proportional;
}

double AutoPID_V2::getIntegral() {
  return _integral;
}

double AutoPID_V2::getDerivative() {
  return _derivative;
}

void AutoPID_V2::setIntegral(double integral){
  _integral = integral;
}

void AutoPID_V2Relay::run() {
  AutoPID_V2::run();
  while ((millis() - _lastPulseTime) > _pulseWidth) _lastPulseTime += _pulseWidth;
  *_relayState = ((millis() - _lastPulseTime) < (_pulseValue * _pulseWidth));
}

double AutoPID_V2Relay::getPulseValue(){
  return (isStopped()?0:_pulseValue);
}

