#include "main.h"
#include <vector>
using namespace std;




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







int mechState = 0; //0 = normal state, 1 = first ring, 2 = second ring, 3 = ram, 4 = push down
int prevMechState;
bool mechPrimed = false;

int getMechState(){
    return mechState;
}

double hs_kP = 0.23;
double hs_kI = 0.00 / 10.0;

double hs_integralBound = 7;
double hs_maxVolts = 12;    
double hs_acceptedError = 0.5;
double hs_timeLimit = 800;
double hs_settledTimeLimit = 80;
double hs_kD;

void setMechState(int number){
    mechState = number;
}
double error;
double prevError;

vector<double> hsStates = {-6, 26, 130, 180, 140, 170, 105, 120, 200};


double hsTick;





void ladyBrown(){
    while(1){


        //if(mechState==0 && fabs(hs.get_actual_velocity())<0.5 && hsAngle.get_angle()<30){
        //    hsAngle.set_position(0);
        //}

        double currentPosition = hsAngle.get_angle()/100;

        if(currentPosition < 0) currentPosition += 360;
        if(currentPosition > 340) currentPosition -= 360;

        
        if(controller.get_digital(DIGITAL_L2)){
            hs.set_brake_mode(MOTOR_BRAKE_HOLD);

            

            if(controller.get_digital(DIGITAL_R1) && !controller.get_digital(DIGITAL_R2)){
                if(currentPosition>1 && currentPosition<260) hs.move_voltage(9000);
            }
            else if(!controller.get_digital(DIGITAL_R1) && controller.get_digital(DIGITAL_R2)){
                if(currentPosition>-15 && currentPosition<210) hs.move_voltage(-9000);
            }
            else hs.move_voltage(0);
            
            intakeToggle = true;
        }
        else{
            if(controller.get_digital_new_press(DIGITAL_RIGHT)){
                mechState = (mechState+1)%3;
            }
            if(controller.get_digital_new_press(DIGITAL_LEFT)){
                if(mechState!=8) mechState = 8;
                else mechState = 0;
            }


            error = currentPosition - hsStates[mechState];
            double derivative = error - prevError;

        
            hs_kP = 0.14;
            

            hs_kD = 0.15;
            /*
            if(mechState == 1) {
                hs_kP = 0.3;
                hs_kD = 0.25;
            }*/

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



int ticksPerRev = 2588;
int tick;
int prevTick;
vector<int> hooksPositionsSensor = {0,642,1279,1905};
vector<int> hooksPositionsEject = {546,1184,1824,2449};
vector<bool> ringToEject = {false,false,false,false};

bool intakeFourState = false;

int sortState = 0; // 0 = no sort, 1 = sort out blue, 2 = sort out red

void setSortState(int state){
    sortState = state;
}

bool intakeOnePrimed = true;

//blue: 180 - 240
//red: 330 - 30


void sort(){
    while(true){



        tick = intakeTwo.get_position();
        tick = fmod(tick,ticksPerRev);
        
        if(tick<0) tick+=ticksPerRev;
        prevTick = tick;



        double colorValue = color.get_hue();

        





        if(sortState==1){
            if(colorValue > 200 && colorValue < 240){
                double smallestDiff = ticksPerRev;
                int smallestIndex = 0;
                for(int i = 0; i < hooksPositionsSensor.size(); i++){
                    double diff = fabs(tick - hooksPositionsSensor[i]);
                    diff = min(diff, ticksPerRev-diff);

                    if(diff<smallestDiff){
                        smallestDiff = diff;
                        smallestIndex = i;
                    }
                }

                ringToEject[smallestIndex] = true;
            }
        }
        else if(sortState==2){
            if(colorValue > 344 || colorValue < 15){
                double smallestDiff = ticksPerRev;
                int smallestIndex = 0;
                for(int i = 0; i < hooksPositionsSensor.size(); i++){
                    double diff = fabs(tick - hooksPositionsSensor[i]);
                    diff = min(diff, ticksPerRev-diff);

                    if(diff<smallestDiff){
                        smallestDiff = diff;
                        smallestIndex = i;
                    }
                }

                ringToEject[smallestIndex] = true;
            }           
        }

        





        for(int ia = 0 ; ia<4; ia++){
            if(ringToEject[ia]){
                ringToEject[(ia+1)%4] = false;
            }
        }

        if(intakeToggle == false){

            if((controller.get_digital(DIGITAL_R2) && !controller.get_digital(DIGITAL_R1)) || intakeState == 1){
                intakeFourState = true;
                 
                if(sortState!=0){
                    for(int i = 0; i<hooksPositionsEject.size(); i++){
                        
                        if(ringToEject[i]){
                            double diffToEject = fabs(tick - hooksPositionsEject[i]-120); //delay 105ticks
                            diffToEject = min(diffToEject, ticksPerRev-diffToEject);
                            if(diffToEject<50){
                                intakeOne.move_voltage(12000);
                                intakeTwo.move_voltage(-12000);
                                pros::delay(300);
                                intakeOne.move_voltage(12000);
                                intakeTwo.move_voltage(12000);
                                ringToEject[i] = false;
                            }
                        }
                        
                    }
                }

                if(mechState==1 && auton){
                    if(intakeTwo.get_actual_velocity()<2 && intakeOnePrimed){
                        intakeTwo.move_voltage(1200000);
                        pros::delay(150);
                        intakeTwo.move_voltage(-2000);
                        pros::delay(100);
                        intakeOnePrimed = false;
                    }
                    else{
                        intakeOne.move_voltage(20000);
                        intakeTwo.move_voltage(1200000);       
                    }
                }
                else{
                    intakeOne.move_voltage(20000);
                    intakeTwo.move_voltage(1200000);
                }

            }
            else if((!controller.get_digital(DIGITAL_R2) && controller.get_digital(DIGITAL_R1))  || intakeState == 2){
                intakeOne.move_voltage(-12000);
                intakeTwo.move_voltage(-12000);
                intakeOnePrimed = true;

            }
            else if(intakeState == 3){
                intakeOne.move_voltage(12000);
                intakeTwo.move_voltage(0);
                intakeOnePrimed = true;
            }
            else if(intakeState == 4){
                intakeOne.move_voltage(0);
                intakeTwo.move_voltage(-12000);
                intakeOnePrimed = true;
            }
            else if(intakeState == 5){
                intakeOne.move_voltage(0);
                intakeTwo.move_voltage(12000);
                intakeOnePrimed = true;
            }
            else if(intakeState == 6 || (!auton && mechState==2)){
                /*
                int beginTime = pros::millis();
                intakeOne.move_voltage(20000);
                intakeTwo.move_voltage(1200000);
                while(pros::millis()-beginTime < 5000){
                    double colorVal = color.get_hue();
                    if((colorVal>180 && colorVal<270) || (colorVal>330 || colorVal<30)){
                        break;
                    }
                }
                */

                double colorVal = color.get_hue();
                if((colorVal>210 && colorVal<230) || (colorVal>350 || colorVal<10)){
                    intakeOne.move_voltage(0);
                    intakeTwo.move_voltage(0);      
                }
                else{
                    intakeOne.move_voltage(12000);
                    intakeTwo.move_voltage(12000);
                }
                intakeOnePrimed = true;
   
            }
            else if(intakeState == 7){

                double colorVal = color.get_hue();
                if(colorVal>350 || colorVal<10){
                    intakeOne.move_voltage(0);
                    intakeTwo.move_voltage(0);      
                }
                else{
                    intakeOne.move_voltage(12000);
                    intakeTwo.move_voltage(12000);
                }
                intakeOnePrimed = true;
     
            }
            else if(intakeState == 8){

                double colorVal = color.get_hue();
                if(colorVal>210 && colorVal<230){
                    intakeOne.move_voltage(0);
                    intakeTwo.move_voltage(0);      
                }
                else{
                    intakeOne.move_voltage(12000);
                    intakeTwo.move_voltage(12000);
                }
                intakeOnePrimed = true;

            }            
            else if(intakeState==9){
                intakeOne.move_voltage(12000);
                intakeTwo.move_voltage(12000);
                pros::delay(200);
                intakeTwo.move_voltage(0);
                pros::delay(50);
                intakeOnePrimed = true;

            }
            else if(intakeState == 10){
                intakeOne.move_voltage(0);
                intakeTwo.move_voltage(3000);
                intakeOnePrimed = true;

            }
            else{
                intakeOne.move_voltage(0);
                intakeTwo.move_voltage(0);
                intakeOnePrimed = true;

            }

            /*
            if(intakeOne.get_voltage()>2000 && intakeOne.get_actual_velocity()<50){
                intakeOne.move_voltage(-12000);
                pros::delay(100);
                intakeOne.move_voltage(12000);
            }
            if(intakeTwo.get_voltage()>2000 && intakeTwo.get_actual_velocity()<50){
                intakeTwo.move_voltage(-12000);
                pros::delay(100);
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


int returnSortStateOne(){
    return ringToEject[0];
}
int returnSortStateTwo(){
    return ringToEject[1];
}
int returnSortStateThree(){
    return ringToEject[2];
}
int returnSortStateFour(){
    return ringToEject[3];
}