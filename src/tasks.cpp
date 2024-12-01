#include "main.h"
#include <vector>

int sortState = 0; // 0 = no sort, 1 = blue sort, 2 = red sort
bool sortPrimed = true;
bool detection = false;


using namespace std;








void sort(){
    while(true){
        
        //--------------------------------------------------------Intake-----------------------------//
        
        if(controller.get_digital(DIGITAL_LEFT) && sortPrimed){
            sortState = (sortState+1)%3;
            sortPrimed = false;
        }
        if(!controller.get_digital(DIGITAL_LEFT)){
            sortPrimed = true;
        }
        



        if(controller.get_digital(DIGITAL_R2) && !controller.get_digital(DIGITAL_R1)){
            if(sortState == 0) intake.move_voltage(12000);
            else if(sortState == 1){
                if(color.get_hue() > 330 || color.get_hue() <30){
                    detection = true;
                }
            }
            else if(sortState == 2){
                if(color.get_hue() > 180 && color.get_hue() <270){
                    detection = true;
                }                
            }



            if(detection){
                pros::delay(100);
                intake.move_voltage(-12000);
                pros::delay(300);
                intake.move_voltage(12000);
                detection = false;
                pros::delay(400);
            }
            else{
                intake.move_voltage(12000);
            }



		}
		else if(!controller.get_digital(DIGITAL_R2) && controller.get_digital(DIGITAL_R1)){
			intake.move_voltage(-12000);
		}
		else{
			intake.move_voltage(0);
		}


        //----------------------------------------------------lady brown-------------------------------------//






















        pros::delay(10);
    }
}



int mechState = 0; //0 = normal state, 1 = first ring, 2 = second ring, 3 = ram, 4 = push down
int prevMechState;
bool mechPrimed = false;

vector<int> hsStates = {0, 43, 49, 105, 165};

double hs_kP = 0.25;
double hs_kI = 0.00 / 10.0;
double hs_kD = 0 * 10.0;
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


double prevError;


void ladyBrown(){
    while(1){
        if(controller.get_digital(DIGITAL_RIGHT) && mechPrimed){
            mechState = (mechState+1)%5;
            mechPrimed = false;
        }
        if(!controller.get_digital(DIGITAL_RIGHT)){
            mechPrimed = true;
        }

        double currentPosition = (float) (hsAngle.get_angle()/100);
        if (currentPosition>340) currentPosition -= 360;
        double error = currentPosition - hsStates[mechState];
        double derivative = error - prevError;


        double hs_kP = 0.25;
        if(mechState == 2) hs_kP = 0.33;




        double output = hs_kP*error + hs_kD*derivative;
        hs.move_voltage(output*1000);

        prevError = error;
        prevMechState = mechState;






        if((controller.get_digital(DIGITAL_R2) && !controller.get_digital(DIGITAL_R1)) || intakeState == 1){

            if(mechState == 1 || mechState == 2){
                intake.move_voltage(9000);
            }
            else intake.move_voltage(12000);
			



		}
		else if(((!controller.get_digital(DIGITAL_R2) && controller.get_digital(DIGITAL_R1)) || controller.get_digital(DIGITAL_RIGHT)) || intakeState == 2){
			intake.move_voltage(-12000);
		}
        else if(intakeState == 3) intake.move_voltage(3000);
		else{
			intake.move_voltage(0);
		}       

















        pros::delay(10);
    }
}




