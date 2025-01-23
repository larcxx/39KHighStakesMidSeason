#include "main.h"
#include "pros/distance.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/vision.h"

pros::v5::MotorGroup ls({15,-16,17}, pros::v5::MotorGears::blue); //left back, left mid, left front
pros::v5::MotorGroup rs({-6,5,-7}, pros::v5::MotorGears::blue); //right back, right mid, right front

pros::v5::Motor intakeOne(19, pros::v5::MotorGears::green);
pros::v5::Motor intakeTwo(-8, pros::v5::MotorGears::green);
pros::v5::Motor hs(20,pros::v5::MotorGears::green); // they haven't decided yet


pros::v5::IMU i1(21);
pros::v5::Rotation yOdomPod(18);
pros::v5::Rotation xOdomPod(10);
pros::v5::Rotation hsAngle(14); // port 1 is just a holder value,
pros::v5::Optical color(4);


pros::adi::DigitalOut goalClamp('A');

pros::v5::Controller controller(pros::E_CONTROLLER_MASTER);

pros::adi::DigitalOut doinker('H');
pros::adi::DigitalOut intakeLift('F');