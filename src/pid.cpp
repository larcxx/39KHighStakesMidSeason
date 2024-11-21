#include "main.h"
#include "pros/rtos.hpp"

PID::PID(double err, double kPi, double kIi, double kDi, double intBound, double AE, double TL, double STL) {
  error = err;
  prevError = err;

  integral = 0;
  derivative = 0;
  totalTime = 0;
  settledTime = 0;

  kP = kPi;
  kI = kIi;
  kD = kDi;

  integralBound = intBound;
  acceptedError = AE;
  timeLimit = TL;
  settledTimeLimit = STL;

  firstRun = true;

  output = kP * error;

  currTime = pros::millis();
  prevTime = pros::millis();
  deltaTime = 0;
}

double PID::calc(double err) {
  currTime = pros::millis();
  deltaTime = currTime - prevTime;

  if (!firstRun && deltaTime < 1) {
    return output;
  }

  prevTime = currTime;

  if (fabs(err) < integralBound) {
    integral += err * deltaTime;
  }
  
  if(positiveOrNegative(err) != positiveOrNegative(prevError)) {
    integral = 0;
  }

  derivative = firstRun ? 0 : (err - prevError) / deltaTime;

  prevError = err;

  totalTime += deltaTime;

  if (fabs(err) < acceptedError) {
    settledTime += deltaTime;
  }

  output = kP * err + kI * integral + kD * derivative;

  firstRun = false;

  return output;
}

bool PID::isSettled() {

  if (totalTime > timeLimit) {
    return true;
  }

  if (settledTime > settledTimeLimit) {
    return true;
  }
  
  return false;

}