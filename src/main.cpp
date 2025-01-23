#include "main.h"
#include "iostream"
#include <string>
#include <vector>

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}


void screen() {
    // loop forever
    while (true) {
		
		
		
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
    	pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
		pros::lcd::print(3, "intake temp: %f", hs.get_temperature()); // print the y position
		pros::lcd::print(4, "base temp: %f", (ls.get_temperature_all()[0] + ls.get_temperature_all()[1] + ls.get_temperature_all()[2] + rs.get_temperature_all()[0] + rs.get_temperature_all()[1] + rs.get_temperature_all()[2])/6);
		pros::lcd::print(5, "intake ticks: %f", fmod(intakeTwo.get_position(),5175));

        pros::delay(50);
		
    }
}




void initialize() {
	pros::lcd::initialize();
	pros::lcd::register_btn1_cb(on_center_button);
	chassis.calibrate();
	pros::Task screenTask(screen);
	pros::Task ladyBrownTask(ladyBrown);
	pros::Task intakeTask(sort);
	rs.set_brake_mode_all(pros::MotorBrake::coast);
	ls.set_brake_mode_all(pros::MotorBrake::coast);
	intakeOne.set_brake_mode(pros::MotorBrake::brake);
	intakeTwo.set_brake_mode(pros::MotorBrake::brake);
	hs.set_brake_mode(pros::MotorBrake::coast);
	color.set_led_pwm(100);
	base.setDriveConstants(1.10, 0, 10, 0, 10.4, 1, 3500, 200);
	base.setTankConstants(0.72, 0.008, 7.2, 7, 12, 3, 3000, 0); // TUNED W/ STL @ 200 
	base.setPivotConstants(.37, 0, 1, 0, 3, /*LAST 3 VALS DONT MAKE DIFF; NEVER SETTLES */1, 1, 1);
}

/**
 * Runs while the ro bot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	setAutonState(true);

	//rs.set_brake_mode_all(pros::MotorBrake::coast);
	//ls.set_brake_mode_all(pros::MotorBrake::coast);

	//pros::delay(60000);
	skills();
	//blueLeft();
	//blueLeftElims();
	//redLeft();
	//redRight();
	//blueRight();
	//redRightElim();
	//blueLeftElim();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */


double t = 4;

double driveCurve(double input){
	return (exp(-t/10)+exp((fabs(input)-127)/10)*(1-exp(-t/10)))*input;
}


bool clampPrimed = true;
bool clampState = false;

bool doinkerPrimed = true;
bool doinkerState = false;

bool intakeLiftState = false;


void opcontrol() {
	setAutonState(false);
	setAdjustment(false);
	while (true) {
		double left = controller.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		double right = controller.get_analog(ANALOG_RIGHT_Y);  // Gets the turn left/right from right joystick
		


		setIntakeState(0);
		if(fabs(left)>10){
			ls.move_voltage(12000*driveCurve(left)/127);
		}
		else ls.move_voltage(0);
		if(fabs(right)>10){
			rs.move_voltage(12000*driveCurve(right)/127);
		}
		else rs.move_voltage(0);
		

		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y) && clampPrimed){
			clampState = !clampState;
			goalClamp.set_value(clampState);
			clampPrimed = false;
		}
		if(!controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
			clampPrimed = true;
		}




		
		if(!controller.get_digital(DIGITAL_L2)){
			if(controller.get_digital_new_press(DIGITAL_L1)){
				doinkerState = !doinkerState;
			}
			intakeLiftState = false;
		}
		else if(controller.get_digital(DIGITAL_L2)){
			if(controller.get_digital(DIGITAL_L1)){
				intakeLiftState = true;
			}
			else intakeLiftState = false;
		}

	

		intakeLift.set_value(intakeLiftState);
        
        doinker.set_value(doinkerState);






		pros::delay(10);
	}
}