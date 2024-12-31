#include "main.h"
#include <vector>

int sortState = 0; // 0 = no sort, 1 = blue sort, 2 = red sort
bool sortPrimed = true;
bool detection = false;


using namespace std;


bool auton = false;
bool ladyBrownAdjustment = false;

void setAutonState(bool value){
    auton = value;
}

void setAdjustment(bool value){
    ladyBrownAdjustment = true;
}









int mechState = 0; //0 = normal state, 1 = first ring, 2 = second ring, 3 = ram, 4 = push down
int prevMechState;
bool mechPrimed = false;

vector<int> hsStates = {0, 33, 110, 210};

double hs_kP = 0.23;
double hs_kI = 0.00 / 10.0;
double hs_kD = 0.015 * 10.0;
double hs_integralBound = 7;
double hs_maxVolts = 12;    
double hs_acceptedError = 0.5;
double hs_timeLimit = 800;
double hs_settledTimeLimit = 80;


void setMechState(int number){
    mechState = number;
}

int intakeState = 0;
void setIntakeState(int state){
    intakeState = state;
}

bool intakeToggle = false;


double prevError;

int ticksPerRev = 700;
int tick;
int prevTick;

void sort(){
    while(true){

        if(intakeToggle == false){

            if((controller.get_digital(DIGITAL_R2) && !controller.get_digital(DIGITAL_R1)) || intakeState == 1){
                
                intake.move_voltage(12000);
            }
            else if(((!controller.get_digital(DIGITAL_R2) && controller.get_digital(DIGITAL_R1)) || controller.get_digital(DIGITAL_RIGHT)) || intakeState == 2){
                intake.move_voltage(-12000);
            }
            else if(intakeState == 3) intake.move_voltage(3000);
            else{
                intake.move_voltage(0);
            }

            if(intake.get_voltage()>3000 && intake.get_actual_velocity()<6 && auton){
                intake.move_voltage(-12000);
                pros::delay(200);
                intake.move_voltage(12000);
            }
            if(intake.get_voltage()>3000 && intake.get_actual_velocity()<6 && ladyBrownAdjustment){
                intake.move_voltage(12000);
                pros::delay(100);
                intake.move_voltage(0);
                pros::delay(100);
                intake.move_voltage(12000);
                pros::delay(100);
                intake.move_voltage(0);
                pros::delay(100);
                intake.move_voltage(12000);
                pros::delay(100);
                intake.move_voltage(0);
                pros::delay(100);
                intake.move_voltage(12000);
                ladyBrownAdjustment = false;
            }
        }
        else intake.move_voltage(0);

        
        tick = intake.get_position();







        prevTick = tick;
    }
}


void ladyBrown(){
    while(1){
        if(controller.get_digital(DIGITAL_L2)){
            double currentPosition = (float) (hsAngle.get_angle()/100);
            if((currentPosition>0 && currentPosition<260) || currentPosition>342 && currentPosition<360){
                if(controller.get_digital(DIGITAL_R1) && !controller.get_digital(DIGITAL_R2)){
                    hs.move_voltage(12000);
                }
                else if(!controller.get_digital(DIGITAL_R1) && controller.get_digital(DIGITAL_R2)){
                    hs.move_voltage(-8000);
                }   
                else hs.move_voltage(0);            
            }
            else hs.move_voltage(0);
            intakeToggle = true;
        }
        else{
            if(controller.get_digital(DIGITAL_RIGHT) && mechPrimed){
                mechState = (mechState+1)%4;
                mechPrimed = false;
            }
            if(!controller.get_digital(DIGITAL_RIGHT)){
                mechPrimed = true;
            }

            double currentPosition = (float) (hsAngle.get_angle()/100);
            if (currentPosition>340) currentPosition -= 360;
            double error = currentPosition - hsStates[mechState];
            double derivative = error - prevError;


            double hs_kP = 0.20;
            if(mechState == 2) hs_kP = 0.18;
            if(mechState == 3) hs_kP = 0.25;




            double output = hs_kP*error + hs_kD*derivative;

            if(mechState ==3 && error<10){
                hs.move_voltage(0);
                hs.set_brake_mode(MOTOR_BRAKE_COAST);
            }
            else {
                hs.move_voltage(output*1000);
                hs.set_brake_mode(MOTOR_BRAKE_HOLD);
            }
            prevError = error;
            prevMechState = mechState;
            
            intakeToggle = false;
        }


        pros::delay(10);
    }
}




