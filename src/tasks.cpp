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

vector<double> hsStates = {-6, 31, 190, 195, 140, 170, 105, 120, 200};


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

        
            hs_kP = 0.15;
        

            hs_kD = 0.3;
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

double queue[4];


void sort(){
    while(true){



        tick = intake.get_position();
        tick = fmod(tick,ticksPerRev);
        
        if(tick<0) tick+=ticksPerRev;
        prevTick = tick;



        double colorValue = color.get_hue();

        
        rpm = intake.get_actual_velocity();

        bool red = colorValue>330 || colorValue<30;
        bool blue = colorValue>180 && colorValue<240;
        bool nothing = colorValue>500;

        bool whatToSort;

        if(sortState==1){
            whatToSort = blue;
        }
        else if(sortState==2){
            whatToSort = red;
        }
        else{
            whatToSort = nothing;
        }



        if(color.get_proximity()<10 && whatToSort){
            if(tick<100 || tick > ticksPerRev - 100){
                ringToEject[0] = 1;
            }
            else if(tick>757 && tick<957){
                ringToEject[1] = 1;
            }
            else if(tick>1610 && tick< 1810){
                ringToEject[2] = 1;
            }
            else{
                ;
            }
        }

        






        if(intakeToggle == false){

            if((controller.get_digital(DIGITAL_R2) && !controller.get_digital(DIGITAL_R1)) || intakeState == 1){
                intakeFourState = true;
                /*
                for(int i = 0; i<3; i++){
                    if(tick>fmod(hooksPositionsEject[i]-100,ticksPerRev) && tick<fmod(hooksPositionsEject[i]+100,ticksPerRev) && ringToEject[0]){
                        intake.move_voltage(-12000);
                        pros::delay(200);
                        intake.move_voltage(12000);
                    }
                }
                */

                intake.move_voltage(12000);
                
                
            }
            else if((!controller.get_digital(DIGITAL_R2) && controller.get_digital(DIGITAL_R1))  || intakeState == 2){
                intake.move_voltage(-12000);
                intakeOnePrimed = true;
            }
            else if(intakeState == 6 || (!auton && mechState==2 && hsAngle.get_angle()>8000)){
                double colorVal = color.get_hue();
                if(((colorVal>210 && colorVal<230) || (colorVal>350 || colorVal<10)) && color.get_proximity()<20){
                    intake.move_voltage(0);  
                }
                else{
                    intake.move_voltage(12000);
                }
                intakeOnePrimed = true;
   
            }
            else if(intakeState == 7){
                if(!(mechState>1 && hsAngle.get_angle()>8000) && color.get_proximity()<20){
                    double colorVal = color.get_hue();
                    if((colorVal>350 || colorVal<15)){
                        intake.move_voltage(0);
                    }
                    else{
                        intake.move_voltage(12000);
                    }
                }
                intakeOnePrimed = true;
     
            }
            else if(intakeState == 8){
                if(!(mechState>1 && hsAngle.get_angle()>8000) && color.get_proximity()<20){

                    double colorVal = color.get_hue();
                    if(colorVal>210 && colorVal<230){
                        intake.move_voltage(0);      
                    }
                    else{
                        intake.move_voltage(12000);
                    }
                    intakeOnePrimed = true;
                }
            }
            else if(intakeState == 10 || controller.get_digital(DIGITAL_DOWN)){
                intake.move_voltage(3000);
                intakeOnePrimed = true;
            }
            else if(intakeState==11){

                if(fabs(intake.get_position()-200)>50){
                    intake.move_voltage(-12000);
                }
                else{
                    intake.move_voltage(0);
                }

                intakeOnePrimed = true;
            }
            else{
                intake.move_voltage(0);
                intakeOnePrimed = true;

            }
        }

        else{
            intake.move_voltage(0);
        }


        prevrpm = rpm;

        pros::delay(10);
    }
}