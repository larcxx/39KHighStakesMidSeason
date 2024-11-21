#pragma once

struct PID {
  bool firstRun; 
  double totalTime;
  double settledTime;

  double kP;
  double kI;
  double kD;

  double error;
  double prevError;

  double integral;
  double integralBound;
  double integralCap;

  double derivative;

  double currTime;
  double prevTime;
  double deltaTime;

  double minOutput;
  double maxOutput;

  double acceptedError;

  double timeLimit;
  double settledTimeLimit;

  double output;

  PID(double err, double kPi, double kIi, double kDi, double intBound, double AE, double TL, double STL);
  
  double calc(double err);

  bool isSettled();

};