#pragma once

enum side {
    leftBase,
    rightBase
};

enum turnDir {
    cw,
    ccw,
    shortest
};

extern double distTravelled;

class Base {
public:
    double drive_kP;
    double drive_kI;
    double drive_kD;

    double drive_integralBound;

    double drive_acceptedError; 
    double drive_timeLimit; 
    double drive_settledTimeLimit;

    double drive_maxVolts;


    double heading_kP;
    double heading_kI;
    double heading_kD;

    double heading_integralBound;

    double heading_acceptedError;
    double heading_timeLimit; 
    double heading_settledTimeLimit;

    double heading_maxVolts;


    double tank_kP;
    double tank_kI;
    double tank_kD;

    double tank_integralBound;

    double tank_acceptedError;
    double tank_timeLimit;
    double tank_settledTimeLimit; 

    double tank_maxVolts; 


    double pivot_kP;
    double pivot_kI;
    double pivot_kD;

    double pivot_integralBound;

    double pivot_acceptedError; 
    double pivot_timeLimit; 
    double pivot_settledTimeLimit;

    double pivot_maxVolts;

    Base();

    void setDriveConstants(double kP, double kI, double kD, double integralBound, double maxVolts, double acceptedError, double timeLimit, double settledTimeLimit);
    void setHeadingConstants(double kP, double kI, double kD, double integralBound, double maxVolts, double acceptedError, double timeLimit, double settledTimeLimit);
    void setTankConstants(double kP, double kI, double kD, double integralBound, double maxVolts, double acceptedError, double timeLimit, double settledTimeLimit);
    void setPivotConstants(double kP, double kI, double kD, double integralBound, double maxVolts, double acceptedError, double timeLimit, double settledTimeLimit);
    void stop();

    double calcDist(double targetHeading, double currentHeading, turnDir t);

    void tankTo(double targetAngle, bool chain = false, turnDir t = shortest, double out = 6.5);
    // void tankFor();

    void pivotTo(double targetAngle, side s, turnDir t = shortest);
};

extern Base base;