#include "main.h"
#include <cmath>

int positiveOrNegative(double num) {
  return num ? fabs(num) / num : 0;
}
double cwDist(double targetHeading, double currentHeading) {
  double error = targetHeading - currentHeading;
  return error < 0 ? error + 360 : error;
}

double ccwDist(double targetHeading, double currentHeading) {
  double error = targetHeading - currentHeading;
  return error > 0 ? error - 360 : error;
}

double minDist(double targetHeading, double currentHeading) {
  double error = targetHeading - currentHeading;
  return fabs(error) > 180 ? -positiveOrNegative(error) * (360 - fabs(error)) : error;
}

double clamp(double num, double lowerBound, double upperBound) {
  return num > upperBound ? upperBound : num < lowerBound ? lowerBound : num;
}

double pctToVolt(double pct) {
  return pct * 12 / 100;
}

double expoDrive(double input) {
  return input >= 0 ? 1.2 * pow(1.043, input) + 0.2 * input - 1.2 : -(1.2 * pow(1.043, -input) - 0.2 * input - 1.2); //.203626461638
}