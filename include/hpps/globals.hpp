#include "api.h"
#include "pros/adi.hpp"
#include "pros/rotation.hpp"

extern pros::v5::MotorGroup ls;
extern pros::v5::MotorGroup rs;

extern pros::v5::Motor intakeOne;
extern pros::v5::Motor intakeTwo;
extern pros::v5::Motor hs;

extern pros::v5::IMU i1;

extern pros::v5::Rotation yOdomPod;
extern pros::v5::Rotation xOdomPod;
extern pros::v5::Rotation hsAngle;

extern pros::v5::Optical color;

extern pros::adi::DigitalOut goalClamp;

extern pros::v5::Controller controller;

extern pros::adi::DigitalOut doinker;

extern pros::adi::DigitalOut intakeLift;

extern pros::v5::Distance distY;
extern pros::v5::Distance distX;
extern pros::v5::Distance distRing;