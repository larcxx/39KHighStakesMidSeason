#include "main.h"
#include "pros/distance.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/vision.h"

pros::v5::MotorGroup ls({-15,-11,-16}, pros::v5::MotorGears::blue); //left back, left mid, left front
pros::v5::MotorGroup rs({13,12,14}, pros::v5::MotorGears::blue); //right back, right mid, right front

pros::v5::Motor intake(-10, pros::v5::MotorGears::blue); //please change this
pros::v5::Motor hs(18,pros::v5::MotorGears::red); // they haven't decided yet


pros::v5::IMU i1(20);
pros::v5::Rotation yOdomPod(17);
pros::v5::Rotation xOdomPod(-9);
pros::v5::Rotation hsAngle(19); // port 1 is just a holder value,
pros::v5::Optical color(12);


pros::adi::DigitalOut goalClamp('H');

pros::v5::Controller controller(pros::E_CONTROLLER_MASTER);

pros::adi::DigitalOut doinker('G');
pros::adi::DigitalOut intakeLift('F');