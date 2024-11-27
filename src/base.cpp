#include "main.h"
#include "iostream"
#include "pros/rtos.hpp"
#include <ios>

#define BASE_MAX_SPEED 64.7953 //450/60 * M_PI * 2.75
#define KV 0.015 //1/BASE_MAX_SPEED
#define KP 0.01
#define KA 0.002

Base::Base() {}

Base base;

void Base::setDriveConstants(double kP, double kI, double kD, double integralBound, double maxVolts, double acceptedError, double timeLimit, double settledTimeLimit) {
    drive_kP = kP;
    drive_kI = kI / 10.0;
    drive_kD = kD * 10.0;
    drive_integralBound = integralBound;
    drive_maxVolts = maxVolts;
    drive_acceptedError = acceptedError;
    drive_timeLimit = timeLimit;
    drive_settledTimeLimit = settledTimeLimit;
}

void Base::setHeadingConstants(double kP, double kI, double kD, double integralBound, double maxVolts, double acceptedError, double timeLimit, double settledTimeLimit) {
    heading_kP = kP;
    heading_kI = kI / 10.0;
    heading_kD = kD * 10.0;
    heading_integralBound = integralBound;
    heading_maxVolts = maxVolts;
    heading_acceptedError = acceptedError;
    heading_timeLimit = timeLimit;
    heading_settledTimeLimit = settledTimeLimit;
}

void Base::setTankConstants(double kP, double kI, double kD, double integralBound, double maxVolts, double acceptedError, double timeLimit, double settledTimeLimit) {
    tank_kP = kP;
    tank_kI = kI / 10.0;
    tank_kD = kD * 10.0;
    tank_integralBound = integralBound;
    tank_maxVolts = maxVolts;
    tank_acceptedError = acceptedError;
    tank_timeLimit = timeLimit;
    tank_settledTimeLimit = settledTimeLimit;
}

void Base::setPivotConstants(double kP, double kI, double kD, double integralBound, double maxVolts, double acceptedError, double timeLimit, double settledTimeLimit) {
    pivot_kP = kP;
    pivot_kI = kI / 10.0;
    pivot_kD = kD * 10.0;
    pivot_integralBound = integralBound;
    pivot_maxVolts = maxVolts;
    pivot_acceptedError = acceptedError;
    pivot_timeLimit = timeLimit;
    pivot_settledTimeLimit = settledTimeLimit;
}







double Base::calcDist(double targetHeading, double currentHeading, turnDir t) {
    if (t == cw) {
        return cwDist(targetHeading, currentHeading);
    }
    if (t == ccw) {
        return ccwDist(targetHeading, currentHeading);
    }
    return minDist(targetHeading, currentHeading);
}


void Base::tankTo(double targetAngle, bool chain, turnDir t) {
    double currentAngle = 360 - i1.get_heading();
    double tankError = calcDist(targetAngle, currentAngle, t);
    PID tankPID(tankError, tank_kP, tank_kI, tank_kD, tank_integralBound, tank_acceptedError, tank_timeLimit, tank_settledTimeLimit);
    while(!tankPID.isSettled()) {
        currentAngle = 360 - i1.get_heading();

        tankError = calcDist(targetAngle, currentAngle, shortest);
        if(chain && fabs(tankError)<5) break;

        double tankOutput = clamp(tankPID.calc(tankError), -tank_maxVolts, tank_maxVolts);

        // Controller.Screen.clearScreen();
        // Controller.Screen.setCursor(1, 1);
        // Controller.Screen.print(currentAngle);
        // Controller.Screen.newLine();
        // Controller.Screen.print(targetAngle);
        // Controller.Screen.newLine();
        // Controller.Screen.print(tankError);
        // Controller.Screen.newLine();
        // Controller.Screen.print(tankOutput);

        double leftPower = -tankOutput;
        double rightPower = tankOutput;

        ls.move_voltage(leftPower*1000);
        rs.move_voltage(rightPower*1000);
        std::cout << tankError << ", " << std::endl;
        pros::delay(10);
    }
    ls.move_voltage(0);
    rs.move_voltage(0);
}

void Base::pivotTo(double targetAngle, side s, turnDir t) {
    double currentAngle = 360 - i1.get_heading();
    double pivotError = calcDist(targetAngle, currentAngle, t);
    PID pivotPID(pivotError, pivot_kP, pivot_kI, pivot_kD, pivot_integralBound, pivot_acceptedError, pivot_timeLimit, pivot_settledTimeLimit);
    while(!pivotPID.isSettled()) {
        currentAngle = 360 - i1.get_heading();
        pivotError = calcDist(targetAngle, currentAngle, shortest);

        double pivotOutput = clamp(pivotPID.calc(pivotError), -pivot_maxVolts, pivot_maxVolts);

        double leftPower = -pivotOutput;
        double rightPower = pivotOutput;
        
        if(s==leftBase){
            ls.move_voltage(leftPower*1000);
        }
        else{
            ls.move_voltage(0);
        }

        if(s==rightBase){
            rs.move_voltage(rightPower*1000);
        }
        else{
            rs.move_voltage(0);
        }

        pros::delay(10);
    }
    ls.move_voltage(0);
    rs.move_voltage(0);
}