#include "main.h"
#include "pros/distance.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/vision.h"

pros::v5::MotorGroup ls({14,-13,12}, pros::v5::MotorGears::blue); //left back, left mid, left front
pros::v5::MotorGroup rs({-4,3,-5}, pros::v5::MotorGears::blue); //right back, right mid, right front

pros::v5::Motor intake(-2, pros::v5::MotorGears::blue);

pros::v5::Motor hs(11,pros::v5::MotorGears::red);


pros::v5::IMU i1(21);
pros::v5::Rotation yOdomPod(-9);
pros::v5::Rotation xOdomPod(8);
pros::v5::Rotation hsAngle(10);
pros::v5::Optical color(1);
pros::v5::Optical colorTwo(19);


pros::adi::DigitalOut goalClamp('A');

pros::v5::Controller controller(pros::E_CONTROLLER_MASTER);

pros::adi::DigitalOut doinkerLeft('H');
pros::adi::DigitalOut doinkerRight('B');

pros::adi::DigitalOut intakeLift('C');

pros::v5::Distance distY(9);
pros::v5::Distance distX(8);