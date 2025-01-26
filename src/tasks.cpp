#include "main.h"
#include <vector>
using namespace std;



int sortState = 0; // 0 = no sort, 1 = blue sort, 2 = red sort
bool sortPrimed = true;
bool detection = false;

bool auton = false;
bool ladyBrownAdjustment = false;

void setAutonState(bool value){
    auton = value;
}

void setAdjustment(bool value){
    ladyBrownAdjustment = true;
}

int intakeState = 0;
void setIntakeState(int state){
    intakeState = state;
}

bool intakeToggle = false;




int ticksPerRev = 5175;
int tick;
int prevTick;

vector<int> hooksPositionsSensor = {3949,182,1450,2475};
vector<int> hooksPositionsEject = {0,1285,2562,3835};
vector<bool> ringToEject = {false,false,false,false};

//blue: 180 - 240
//red: 330 - 30

void sort(){
    while(true){

        tick = intakeTwo.get_position();
        tick = fmod(tick,ticksPerRev);
        
        if(tick<0) tick+=ticksPerRev;
        prevTick = tick;



        double colorValue = color.get_hue();

        if(colorValue > 180 && colorValue < 270){
            double smallestDiff = 700;
            int smallestIndex = 0;
            for(int i = 0; i < hooksPositionsSensor.size(); i++){
                double diff = fabs(tick - 600 - hooksPositionsSensor[i]);
                if(diff>1600){
                    diff -= ticksPerRev;
                    diff *= -1;
                }

                if(diff<smallestDiff){
                    smallestDiff = diff;
                    smallestIndex = i;
                } 
            }

            ringToEject[smallestIndex] = true;
        }






        if(intakeToggle == false){

            if((controller.get_digital(DIGITAL_R2) && !controller.get_digital(DIGITAL_R1)) || intakeState == 1){
                
                for(int i = 0; i<hooksPositionsEject.size(); i++){

                    
                    int predictTick = fmod(tick-40,ticksPerRev); //set the tick further in front so when it comes into the range it will be right there


                    double measuredDiff = fabs(predictTick-hooksPositionsEject[i]);
                    if(measuredDiff>1600){
                        measuredDiff -= ticksPerRev;
                        measuredDiff *= -1;
                    }

                    if(measuredDiff<50 && ringToEject[i]){
                        intakeOne.move_voltage(0);
                        intakeTwo.move_voltage(-12000);
                        pros::delay(200);
                        intakeOne.move_voltage(12000);
                        intakeTwo.move_voltage(12000);
                        ringToEject[i] = false;
                    }
                }
                
                intakeOne.move_voltage(12000);
                intakeTwo.move_voltage(12000);
            }
            else if(((!controller.get_digital(DIGITAL_R2) && controller.get_digital(DIGITAL_R1)) || controller.get_digital(DIGITAL_RIGHT)) || intakeState == 2){
                intakeOne.move_voltage(-12000);
                intakeTwo.move_voltage(-12000);
            }
            else if(intakeState == 3){
                intakeOne.move_voltage(3000);
                intakeTwo.move_voltage(3000);
            }
            else{
                intakeOne.move_voltage(0);
                intakeTwo.move_voltage(0);
            }
            /*
            if(intakeOne.get_voltage()>2000 && intakeOne.get_actual_velocity()<1 && auton){
                intakeOne.move_voltage(-12000);
                intakeTwo.move_voltage(-12000);
                pros::delay(100);
                intakeOne.move_voltage(12000);
                intakeTwo.move_voltage(12000);
            }
            */
        }
        else{
            intakeOne.move_voltage(0);
            intakeTwo.move_voltage(0);
        }

        

        pros::delay(10);
    }
}


int mechState = 0; //0 = normal state, 1 = first ring, 2 = second ring, 3 = ram, 4 = push down
int prevMechState;
bool mechPrimed = false;



double hs_kP = 0.23;
double hs_kI = 0.00 / 10.0;

double hs_integralBound = 7;
double hs_maxVolts = 12;    
double hs_acceptedError = 0.5;
double hs_timeLimit = 800;
double hs_settledTimeLimit = 80;


void setMechState(int number){
    mechState = number;
}
double error;
double prevError;

vector<double> hsStates = {0, 19, 157, 180, 110, 160};


double hsTick;

void ladyBrown(){
    while(1){

        double currentPosition = hsAngle.get_angle()/100;

        if(currentPosition < 0) currentPosition += 360;
        if(currentPosition > 340) currentPosition -= 360;


        
        if(controller.get_digital(DIGITAL_L2)){
            hs.set_brake_mode(MOTOR_BRAKE_HOLD);

            
            if(currentPosition>-15 && currentPosition<260){
                if(controller.get_digital(DIGITAL_R1) && !controller.get_digital(DIGITAL_R2)){
                    hs.move_voltage(12000);
                }
                else if(!controller.get_digital(DIGITAL_R1) && controller.get_digital(DIGITAL_R2)){
                    hs.move_voltage(-12000);
                }   
                else hs.move_voltage(0);            
            }
            else hs.move_voltage(0);
            intakeToggle = true;
        }
        else{
            if(controller.get_digital(DIGITAL_RIGHT) && mechPrimed){
                mechState = (mechState+1)%3;
                mechPrimed = false;
            }
            if(!controller.get_digital(DIGITAL_RIGHT)){
                mechPrimed = true;
            }

            error = currentPosition - hsStates[mechState];
            double derivative = error - prevError;


            double hs_kP = 0.20;
            if(mechState == 3) hs_kP = 0.1;

            double hs_kD =  0.015*10;



            double output = hs_kP*error + hs_kD*derivative;

            /*
            if(mechState==2 || mechState==0){
                hs.set_brake_mode(MOTOR_BRAKE_COAST);
            }
            else {
                hs.set_brake_mode(MOTOR_BRAKE_HOLD);
            }
            */

            if(mechState==2){
                if(error<0) hs.move_voltage(-12000);
                else hs.move_voltage(0);
            }
            else{
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




