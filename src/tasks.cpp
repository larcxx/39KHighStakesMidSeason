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

double derivative;
double lb_integral = 0;
double error;
double prevError;

vector<double> hsStates = {-6, 30, 150, 195, 140, 170, 105, 120, 200};


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
            lb_integral += error;
            derivative = error - prevError;

        
            hs_kP = 0.2;
        

            hs_kD = 0.9;
            /*
            if(mechState == 1) {
                hs_kP = 0.3;
                hs_kD = 0.25;
            }*/

            double output = hs_kP*error+ hs_kD*derivative;

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
vector<int> hooksPositionsSensor = {0,857,1710};
vector<int> hooksPositionsEject = {555,1418,2292};
vector<bool> ringToEject = {false,false,false,false};

bool intakeFourState = false;

int sortState = 0; // 0 = no sort, 1 = sort out blue, 2 = sort out red

void setSortState(int state){
    sortState = state;
}

bool intakeOnePrimed = true;


double rpm;
double prevrpm;

//blue: 180 - 240
//red: 330 - 30


void sort(){
    while(true){



        tick = intakeTwo.get_position();
        tick = fmod(tick,ticksPerRev);
        
        if(tick<0) tick+=ticksPerRev;
        prevTick = tick;



        double colorValue = color.get_hue();

        
        rpm = intakeTwo.get_actual_velocity();




        if(sortState==1){
            if((colorValue > 200 && colorValue < 240) && distRing.get_distance()<55){
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
            if((colorValue > 344 || colorValue < 15) && distRing.get_distance()<55){
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

        






        if(intakeToggle == false){

            if((controller.get_digital(DIGITAL_R2) && !controller.get_digital(DIGITAL_R1)) || intakeState == 1){
                intakeFourState = true;
                 
                if(sortState!=0){
                    for(int i = 0; i<hooksPositionsEject.size(); i++){
                        
                        if(ringToEject[i]){
                            double diffToEject = fabs(tick - hooksPositionsEject[i]-105); //delay 105ticks
                            diffToEject = min(diffToEject, ticksPerRev-diffToEject);
                            if(diffToEject<50){
                                intakeOne.move_voltage(12000);
                                intakeTwo.move_voltage(-12000);
                                pros::delay(150);
                                intakeOne.move_voltage(12000);
                                intakeTwo.move_voltage(12000);
                                ringToEject[i] = false;
                            }
                        }
                        
                    }
                }

                if(mechState==1){
                    if(intakeTwo.get_actual_velocity()<5 && intakeOnePrimed && (rpm - prevrpm < 0) && auton){
                        intakeTwo.move_voltage(1200000);
                        pros::delay(150);
                        intakeTwo.move_voltage(-2000);
                        pros::delay(100);
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
                /*
                if(intakeTwo.get_actual_velocity()<5){
                    intakeTwo.move_voltage(-12000);
                    pros::delay(500);
                    intakeTwo.move_voltage(12000);
                }
                */
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
            else if(intakeState == 6 || (!auton && mechState==2 && hsAngle.get_angle()>8000)){
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
                if(((colorVal>210 && colorVal<230) || (colorVal>350 || colorVal<10)) && distRing.get_distance()<100){
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

                
                if(!(mechState>1 && hsAngle.get_angle()>8000)){
                    double colorVal = color.get_hue();
                    if((colorVal>350 || colorVal<15)){
                        intakeOne.move_voltage(0);
                        intakeTwo.move_voltage(0);
                    }
                    else{
                        intakeOne.move_voltage(12000);
                        intakeTwo.move_voltage(12000);
                    }
                }
                intakeOnePrimed = true;
     
            }
            else if(intakeState == 8){
                if(!(mechState>1 && hsAngle.get_angle()>8000)){

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
            }            
            else if(intakeState==9){
                intakeOne.move_voltage(12000);
                intakeTwo.move_voltage(12000);
                pros::delay(200);
                intakeTwo.move_voltage(0);
                pros::delay(50);
                intakeOnePrimed = true;

            }
            else if(intakeState == 10 || controller.get_digital(DIGITAL_DOWN)){
                intakeOne.move_voltage(0);
                intakeTwo.move_voltage(3000);
                intakeOnePrimed = true;

            }
            else if(intakeState==11){

                if(fabs(intakeTwo.get_position()-200)>50){
                    intakeOne.move_voltage(-12000);
                    intakeTwo.move_voltage(-12000);
                }
                else{
                    intakeOne.move_voltage(0);
                    intakeTwo.move_voltage(0);
                }

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


        prevrpm = rpm;

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