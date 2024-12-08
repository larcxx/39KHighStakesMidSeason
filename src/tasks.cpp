#include "main.h"
#include <vector>

int sortState = 0; // 0 = no sort, 1 = blue sort, 2 = red sort
bool sortPrimed = true;
bool detection = false;


using namespace std;


bool auton = false;

void setAutonState(bool value){
    auton = value;
}









int mechState = 0; //0 = normal state, 1 = first ring, 2 = second ring, 3 = ram, 4 = push down
int prevMechState;
bool mechPrimed = false;

vector<int> hsStates = {0, 37, 105, 210};

double hs_kP = 0.23;
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


void sort(){
    while(true){

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

    }
}


void ladyBrown(){
    while(1){
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


        double hs_kP = 0.25;
        if(mechState == 2) hs_kP = 0.33;




        double output = hs_kP*error + hs_kD*derivative;
        hs.move_voltage(output*1000);

        prevError = error;
        prevMechState = mechState;


        pros::delay(10);
    }
}




