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

vector<double> hsStates = {0, 31.2, 50, 180, 195, 120, 170, 105, 120, 200};


double hsTick;



void ladyBrown(){
    while(1){

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
                mechState = (mechState+1)%4;
            }
            if(controller.get_digital_new_press(DIGITAL_LEFT)){
                if(mechState!=9) mechState = 9;
                else mechState = 0;
            }


            error = currentPosition - hsStates[mechState];
            lb_integral += error;
            derivative = error - prevError;

        
            hs_kP = 0.15;
            hs_kD = 0.3;

            double output = hs_kP*error+ hs_kD*derivative;


            if(mechState==3){
                if(error<0) hs.move_voltage(-12000);
                else hs.move_voltage(0);
            }
            else if(mechState==2){
                if(error<0) hs.move_voltage(-8000);
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

bool edgePrimed = true;
bool edgeActivate = false;

void setEdgeState(int state){
    edgeActivate = state;
}

bool waitTillRing = false;

float prevOutput;

void sort(){
    while(true){



        tick = intake.get_position();
        tick = fmod(tick,ticksPerRev);
        
        if(tick<0) tick+=ticksPerRev;
        prevTick = tick;



        rpm = intake.get_actual_velocity();

    

        if(intakeToggle == false){

            if((controller.get_digital(DIGITAL_R2) && !controller.get_digital(DIGITAL_R1)) || intakeState == 1){

                
                intakeFourState = true;

                float degree;

                double colorValueOne = color.get_hue();
                double colorValueTwo = colorTwo.get_hue();
                double proximityOne = color.get_proximity();
                double proximityTwo = colorTwo.get_proximity();

                bool red = ((colorValueOne > 330 || colorValueOne < 30) || (colorValueTwo > 330 || colorValueTwo < 30)) && (proximityOne>120 || proximityTwo>120);
                bool blue = ((colorValueOne > 180 && colorValueOne < 270) || (colorValueTwo > 180 && colorValueTwo < 270)) && (proximityOne>120 || proximityTwo>120);   

                intake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
                degree = intake.get_position();
                float output = fmod(degree, 719.56666);

                intake.move_voltage(12000);

                if(red){
                    waitTillRing = true;
                }

                if(waitTillRing){
                    if(output > 680 || output < 20){
                        intake.move_voltage(-12000);
                        pros::delay(200);
                        intake.move_voltage(12000);
                        waitTillRing = false;
                    }
                }

                prevOutput = output;

            }


            else if((!controller.get_digital(DIGITAL_R2) && controller.get_digital(DIGITAL_R1))  || intakeState == 2){
                intake.move_voltage(-12000);
                intakeOnePrimed = true;
            }


            else if(intakeState == 6){
                double colorVal = color.get_hue();
                if(((colorVal>210 && colorVal<230) || (colorVal>350 || colorVal<10)) && color.get_proximity()>150){
                    intake.move_voltage(0);  
                }
                else{
                    intake.move_voltage(12000);
                }
                intakeOnePrimed = true;
   

            }
            else if(intakeState == 7){
                double colorVal = color.get_hue();
                if(((colorVal>350 || colorVal<30)) && color.get_proximity()>150){
                    intake.move_voltage(0);
                    intakeState = 0;
                }
                else{
                    intake.move_voltage(8000);
                }
                intakeOnePrimed = true;
   

            }
            else if(intakeState == 8){
                double colorVal = color.get_hue();
                if(((colorVal>180 && colorVal<270)) && color.get_proximity()>150){
                    intake.move_voltage(0);  
                }
                else{
                    intake.move_voltage(12000);
                }
                intakeOnePrimed = true;
   

            }
            


            else if(intakeState == 10 || controller.get_digital(DIGITAL_DOWN)){
                intake.move_voltage(-3000);
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