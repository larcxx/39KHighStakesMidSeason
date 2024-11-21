#pragma once

int positiveOrNegative(double num);

double cwDist(double targetHeading, double currentHeading);

double ccwDist(double targetHeading, double currentHeading);

double minDist(double targetHeading, double currentHeading);

double clamp(double num, double lowerBound, double upperBound);

double pctToVolt(double pct);

double expoDrive(double input);