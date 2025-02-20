#include "main.h"
#include <vector>

PIDTWO::PIDTWO(double kP, double kI, double kD, double windupRange, bool signFlipReset) {
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->windupRange = windupRange;
    this->signFlipReset = signFlipReset;
}
float PIDTWO::runLoop(float error) {
    float output;
    integral += error;
    if (integral * kI > windupRange && windupRange != 0) {
        integral = windupRange / kI;
    }
    if(signFlipReset && lemlib::sgn(error) != lemlib::sgn(prevError)) {
        integral = 0;
    }
    derivative = error - prevError;
    output = kP * error + kI * integral + kD * derivative;
    prevError = error;
    return output;
}

PIDTWO drivePID(0.0026, 0.000007, 0.0025, 30.0, true);

void DrivePID(double distance, int Timeout) {
    chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
    int counter = 0;
    int curTime = pros::millis();
    int timeout = Timeout;
    double startRotation = (yOdomPod.get_position());
    double targetRotation = (yOdomPod.get_position() + (36000/(M_PI * 2.125) * distance));
    double currentRotation = (yOdomPod.get_position());
    double error = targetRotation - currentRotation;
    while(true) {
        if(pros::millis() - curTime >= timeout) {
            break;
        }
        if (fabs(error) < 1000) {
            counter++;
        }
        else {
            counter = 0;
        }
        if(counter == 30) {
            break;
        }
        currentRotation = (yOdomPod.get_position());
        error = (targetRotation - currentRotation);
        ls.move_velocity(drivePID.runLoop(error));
        rs.move_velocity(drivePID.runLoop(error));
        pros::delay(10);
    }
    ls.move_velocity(0);
    rs.move_velocity(0);
}





void skills(){

    chassis.setPose(63.5,0,270);
    setIntakeState(10);

    pros::delay(300);


    chassis.moveToPose(48,0,270,1000,{.minSpeed = 30});
    base.tankTo(90, true);

    setIntakeState(2);

    chassis.moveToPose(48,26,180,2000,{.forwards=false, .maxSpeed = 50,.minSpeed=10},true);
    chassis.waitUntil(17);
    setMechState(0);
    goalClamp.set_value(1);
    chassis.waitUntilDone();

    setIntakeState(1);
    base.tankTo(0);

    chassis.moveToPose(24,25.5,270,1500,{ .minSpeed = 60});
    base.tankTo(-55);

    chassis.moveToPose(-52,62.5,290,3000,{.horizontalDrift = 10, .lead = 0.65, .maxSpeed = 80, .minSpeed = 10},false);

    


    chassis.moveToPose(-4,40,290,2000,{.forwards=false, .minSpeed = 30}, true);
    chassis.waitUntil(7);
    setMechState(1);
    chassis.waitUntilDone();

    chassis.waitUntilDone();

    


    base.tankTo(-90);


    chassis.moveToPose(-4,65,0,1500,{.lead = 0, .maxSpeed = 80, .minSpeed = 40},false);

    setMechState(2);
    setIntakeState(4);

    pros::delay(200);

    setIntakeState(6);

    pros::delay(400);

    setMechState(1);
    pros::delay(500);

    setIntakeState(1);

    pros::delay(800);
    
    setIntakeState(4);
    setMechState(2);

    pros::delay(500);


    chassis.moveToPose(chassis.getPose().x ,48,0,1500,{.forwards=false, .minSpeed = 30, .earlyExitRange = 3},false);

    setMechState(0);

    setIntakeState(1);
    base.tankTo(180);

    chassis.moveToPose(61,52,90,3000,{.maxSpeed = 60, .minSpeed=40, .earlyExitRange=3});
    

    chassis.moveToPose(57,52,90,1000,{.forwards=false, .minSpeed=10, .earlyExitRange=1},false);
    setIntakeState(0);
    base.tankTo(-45, true);
    setIntakeState(1);
    chassis.moveToPose(48,61,315,1000,{});
    base.tankTo(20);
    goalClamp.set_value(0);
    chassis.moveToPose(63,65,260,1500,{.forwards=false, .minSpeed=20});

    

    chassis.moveToPose(48,0,180,2200,{.maxSpeed = 80},true);
    chassis.waitUntil(30);
    setIntakeState(2);
    chassis.waitUntilDone();

    base.tankTo(-90, true, shortest, 2);
    chassis.moveToPose(48, -24, 0, 1200, {.forwards = false, .maxSpeed = 50}, true);
    chassis.waitUntil(24);
    goalClamp.set_value(1);
    chassis.waitUntilDone();


    

    base.tankTo(0);
    setIntakeState(1);
    chassis.moveToPose(24,-24,270,1500,{.minSpeed = 1});
    base.tankTo(45);


    chassis.moveToPose(-49,-58,250,3000,{.horizontalDrift = 10, .lead = 0.65, .maxSpeed = 80},true);
    chassis.waitUntil(80);
    setMechState(1);
    chassis.waitUntilDone();

    
    chassis.moveToPose(-5,-40,250,2000,{.forwards=false, .minSpeed = 30}, false);

    base.tankTo(90);
    chassis.moveToPose(-5,-64,180,1500,{.lead = 0, .maxSpeed = 80, .minSpeed = 20},false);
    chassis.setPose(chassis.getPose().x, -63.9, chassis.getPose().theta);

    setMechState(2);
    setIntakeState(4);

    pros::delay(200);

    setIntakeState(6);

    pros::delay(400);

    setMechState(1);
    pros::delay(500);

    setIntakeState(1);

    pros::delay(800);
    
    setIntakeState(4);
    setMechState(2);

    pros::delay(500);


    chassis.moveToPose(chassis.getPose().x, -48,180,1000,{.forwards=false, .lead = 0, .minSpeed = 30},false);

    setMechState(0);
    base.tankTo(180);
    setIntakeState(1);
    chassis.moveToPose(61,-51,90,5000, {.maxSpeed=50, .minSpeed = 30},false);
    setIntakeState(0);
    base.tankTo(45, true);
    setIntakeState(1);
    chassis.moveToPose(48,-64,225,1500,{.earlyExitRange = 1.5});
    base.tankTo(-15);
    pros::delay(400);
    goalClamp.set_value(0);
    chassis.moveToPose(65,-63, 240,800,{.forwards=false, .minSpeed = 60});

    

    chassis.moveToPose(-28,-26,315,3000,{.maxSpeed = 100, .minSpeed = 70}, true);
    chassis.waitUntil(30);
    setMechState(1);
    chassis.waitUntilDone();

    chassis.moveToPose(-57,-26,220,2000,{.lead = 0.3, .maxSpeed = 80, .minSpeed = 55}, true);

    chassis.moveToPose(-72,-62,180,2500,{.lead = 0.3, .minSpeed = 50},false);

    chassis.setPose(-63 + ( (float) distX.get_distance() / (float) 25.4), chassis.getPose().y, chassis.getPose().theta);

    chassis.moveToPose(-42,3,200,2000,{.forwards = false, .maxSpeed = 60, .minSpeed = 20}, true);
    chassis.waitUntil(54);
    goalClamp.set_value(1);
    chassis.waitUntilDone();

    //chassis.moveToPose(-50,0,200,1500, {});

    base.tankTo(0);

    chassis.moveToPose(-70,-1,270,1500,{.minSpeed = 70}, false);    

    DrivePID(-7,1000);

    setIntakeState(4);
    setMechState(8);





    pros::delay(600);

    DrivePID(-7,500);

    setIntakeState(1);
    setMechState(0);



    base.tankTo(90);
    chassis.moveToPose(-48,-50,180,1500, {.minSpeed = 50});
    chassis.moveToPose(3, -11,45,3000,{.minSpeed = 50}, true);
    chassis.waitUntil(45);
    setIntakeState(3);
    chassis.waitUntilDone();
    
    //base.tankTo(315, true);

    chassis.moveToPose(-40,48,315,2000,{.maxSpeed = 100, .minSpeed = 70}, true);
    chassis.waitUntil(25);
    setIntakeState(1);
    chassis.waitUntilDone();


    setIntakeState(1);
    chassis.moveToPose(-55,50,270,1500,{.maxSpeed = 80, .minSpeed = 50},false);
    setIntakeState(1);
    chassis.moveToPose(-44,50,270,1000,{.forwards = false, .minSpeed = 80},true);
    chassis.waitUntil(8);
    doinker.set_value(1);
    setIntakeState(1);
    chassis.waitUntilDone();

    base.tankTo(200, true, shortest, 25);
    base.tankTo(135, true);

    doinker.set_value(0);

    goalClamp.set_value(0);

    DrivePID(-10,600);

    chassis.moveToPose(-28,38,135,2000,{.minSpeed=50});

    setMechState(3);
    base.tankTo(-45);

    DrivePID(-2000,5000);

    //chassis.moveToPose(-10,10,45,4000, {.forwards =false, .maxSpeed = 100, .minSpeed = 100});


}

void redLeft(){
    chassis.setPose(-58.33,13.23,217);
    
    setMechState(3);
    pros::delay(500);
    ls.move_voltage(-3000);
    rs.move_voltage(-3000);
    pros::delay(250);
    ls.move_voltage(0);
    ls.move_voltage(0);
    chassis.moveToPose(-20,24,255,1500,{.forwards=false, .maxSpeed = 80},true);
    chassis.waitUntil(20);
    setMechState(0);
    chassis.waitUntil(36);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);

    base.tankTo(-188);
    chassis.moveToPose(-6,56,0,2000,{.maxSpeed = 70},false);

    chassis.moveToPose(-24,24,45,1500,{.forwards=false, .lead = 0.6, .minSpeed = 40},false);
    setIntakeState(0);
    base.tankTo(-143);
    setIntakeState(1);
    //chassis.moveToPose(-24,50,0,1500,{});
    chassis.moveToPose(-24,50,0,2000,{.minSpeed = 70,.earlyExitRange=3}, false);
    chassis.moveToPose(-75,68,270,1500,{.lead = 0.2, .minSpeed = 100},false);

    setMechState(0);
    chassis.moveToPose(-48,24,0,2000,{.forwards=false, .minSpeed = 45, .earlyExitRange = 3}); 
    base.tankTo(37,true);

    intakeLift.set_value(1);
    chassis.moveToPose(-49,0,180,1200,{.earlyExitRange = 3});
    chassis.moveToPose(-49,15,180,1200,{.forwards = false, .earlyExitRange = 3});


    chassis.moveToPose(-20,5,110,1000,{},false);
    pros::delay(10);
    setMechState(5);
    
}

void redLeftElim(){
    chassis.setPose(-58.33,13.23,217);
    
    setMechState(3);
    pros::delay(500);
    ls.move_voltage(-3000);
    rs.move_voltage(-3000);
    pros::delay(250);
    ls.move_voltage(0);
    ls.move_voltage(0);
    chassis.moveToPose(-20,24,255,1500,{.forwards=false, .maxSpeed = 80},true);
    chassis.waitUntil(20);
    setMechState(0);
    chassis.waitUntil(36);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);

    base.tankTo(-188);
    chassis.moveToPose(-6,56,0,2000,{.maxSpeed = 55},false);

    chassis.moveToPose(-24,24,45,1500,{.forwards=false, .lead = 0.6, .minSpeed = 40},false);
    setIntakeState(0);
    base.tankTo(-143);
    setIntakeState(1);
    //chassis.moveToPose(-24,50,0,1500,{});
    chassis.moveToPose(-24,50,0,2000,{.minSpeed = 70,.earlyExitRange=3}, false);
    chassis.moveToPose(-75,68,270,1500,{.lead = 0.2, .minSpeed = 100},false);

    setMechState(0);
    chassis.moveToPose(-48,24,0,2000,{.forwards=false, .minSpeed = 45, .earlyExitRange = 3}); 
    base.tankTo(37,true);

    intakeLift.set_value(1);
    chassis.moveToPose(-49,0,180,1200,{.earlyExitRange = 3},false);
    intakeLift.set_value(0);
    chassis.moveToPose(-49,15,180,1200,{.forwards = false, .earlyExitRange = 3},false);

    base.tankTo(7,true);
    chassis.moveToPose(-58,-60,200,2000,{.minSpeed = 50},true);
    chassis.waitUntil(6);
    setIntakeState(0);
    

        

}

void qualRedRight(){
    chassis.setPose(-58.33,-13.23,323);
    
    setMechState(3);
    pros::delay(500);
    ls.move_voltage(-6000);
    rs.move_voltage(-6000);
    pros::delay(250);
    ls.move_voltage(0);
    ls.move_voltage(0);
    chassis.moveToPose(-19,-24,280,1500,{.forwards=false, .maxSpeed = 80},true);
    chassis.waitUntil(22);
    setMechState(0);
    chassis.waitUntil(41);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(3);

    base.tankTo(143);

    chassis.moveToPose(-22,-50,180,2000,{.minSpeed = 50}, false);
    chassis.moveToPose(-22,-20,180,2000,{.forwards = false, .minSpeed = 50, .earlyExitRange=3}, false);
    base.tankTo(53);
    setIntakeState(1);
    chassis.moveToPose(-68,-72,180,2000,{.lead = 0.4, .minSpeed = 100},true);
    pros::delay(200);

    setMechState(0);
    chassis.moveToPose(-48,-24,180,2000,{.forwards=false, .minSpeed = 70 }, false);
    base.tankTo(-37,true);

    setIntakeState(1);
    chassis.moveToPose(-48,0,0,1500,{.minSpeed = 30, .earlyExitRange = 2},false);
    intakeLift.set_value(0);
    chassis.moveToPose(-48,-20,0,1500,{.forwards = false, .minSpeed = 30,.earlyExitRange = 2});


    chassis.moveToPose(-20,-5,70,1000,{}, false);


    

    setMechState(5);

    /*
    chassis.moveToPose(-10,-10,75,1500,{.lead = 0.3},true);
    chassis.waitUntil(24);
    setIntakeState(0);
    chassis.waitUntilDone();
    doinker.set_value(1);
    pros::delay(300);

    chassis.moveToPose(-24,-28,30,1500,{.forwards = false,.earlyExitRange = 1},false);
    doinker.set_value(0);
    setIntakeState(1);
    chassis.moveToPose(-24,-12,0,1500,{.minSpeed = 30, .earlyExitRange = 1});
    */


}

void redRight(){
    /* kalahari redright auton
    chassis.setPose(49,60,270);
    chassis.moveToPose(15,57.5,260,2000,{.lead=0.23,.minSpeed=200},true);
    chassis.waitUntil(90);
    doinker.set_value(1);
    chassis.waitUntilDone();


    chassis.moveToPose(24,64,270,2000,{.forwards = false,.lead=0.3, .minSpeed = 80},true);
    chassis.waitUntil(40);
    chassis.waitUntilDone();

    doinker.set_value(0);
    pros::delay(250);

    base.tankTo(-170);
    chassis.moveToPose(9,53,84,2000,{.forwards = false, .lead = 0.2, .maxSpeed = 50},true);
    chassis.waitUntil(15);
    goalClamp.set_value(1);
    chassis.waitUntilDone();

    setIntakeState(1);
    pros::delay(200);

    chassis.moveToPose(50,24,180,2000,{.horizontalDrift = 10, .lead = 0.4 , .minSpeed = 50},true);
    chassis.waitUntil(20);
    goalClamp.set_value(0);
    chassis.waitUntilDone();
    goalClamp.set_value(0);
    pros::delay(100);
    base.tankTo(180);
    chassis.moveToPose(13,20.5,90,2000,{.forwards=false, .maxSpeed = 70});
    chassis.waitUntil(33);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);

    base.tankTo(-60);
    chassis.moveToPose(0,60,330,2000,{});
    chassis.moveToPose(9,22,330,1500,{.forwards = false});
    base.tankTo(45);

    setMechState(5);
    */


    chassis.setPose(48,36,290);
    setIntakeState(3);
    chassis.moveToPose(17,47,290,2000,{.minSpeed=200},true);
    chassis.waitUntil(40);
    doinker.set_value(1);
    chassis.waitUntilDone();


    chassis.moveToPose(48,36,290,2000,{.forwards=false, .minSpeed = 60},true);
    chassis.waitUntil(15);
    doinker.set_value(0);
    chassis.waitUntilDone();

    setIntakeState(0);

    

    base.tankTo(-135 ,true);
    chassis.moveToPose(20,23,60,2000,{.forwards =false, .maxSpeed = 70, .minSpeed = 20}, true);
    chassis.waitUntil(26);
    goalClamp.set_value(1);
    chassis.waitUntilDone();

    setIntakeState(1);

    base.tankTo(200);

    chassis.moveToPose(63,60,15,1500,{.lead = 0.6, .minSpeed = 100});
    chassis.moveToPose(65,75,0,1500,{.lead = 0.6, .minSpeed = 127});
    chassis.moveToPose(55,60,35, 1000, {.forwards = false, .minSpeed = 50},false);

    base.tankTo(140);
    pros::delay(600);
    goalClamp.set_value(0);
    

    chassis.moveToPose(48,40,180, 1200, {.minSpeed = 50});
    base.tankTo(200);

    chassis.moveToPose(10, 40,90, 2000, {.forwards = false, .maxSpeed = 80, .minSpeed = 20},true);
    chassis.waitUntil(35);
    goalClamp.set_value(1);
    chassis.waitUntilDone();

    chassis.moveToPose(22,22,135,2000, {.minSpeed = 50},false);

    base.tankTo(65);
    setMechState(5);


    /*

    chassis.moveToPose(24,24,45,1300,{.forwards = false},false);

    setIntakeState(0);

    base.tankTo(65);
    setMechState(5);
    */

}

void redRightElim(){
    chassis.setPose(48,36,290);
    setIntakeState(3);
    chassis.moveToPose(17,47,290,2000,{.minSpeed=200},true);
    chassis.waitUntil(40);
    doinker.set_value(1);
    chassis.waitUntilDone();


    chassis.moveToPose(48,36,290,2000,{.forwards=false, .minSpeed = 60},true);
    chassis.waitUntil(15);
    doinker.set_value(0);
    chassis.waitUntilDone();

    setIntakeState(0);

    

    base.tankTo(-135 ,true);
    chassis.moveToPose(20,23,60,2000,{.forwards =false, .maxSpeed = 70, .minSpeed = 20}, true);
    chassis.waitUntil(26);
    goalClamp.set_value(1);
    chassis.waitUntilDone();

    setIntakeState(1);

    base.tankTo(200);

    chassis.moveToPose(63,60,15,1500,{.lead = 0.6, .minSpeed = 100});
    chassis.moveToPose(65,75,0,1500,{.lead = 0.6, .minSpeed = 127});
    chassis.moveToPose(55,60,35, 1000, {.forwards = false, .minSpeed = 50},false);

    base.tankTo(140);
    pros::delay(600);
    goalClamp.set_value(0);
    

    chassis.moveToPose(48,40,180, 1200, {.minSpeed = 50});
    base.tankTo(200);

    chassis.moveToPose(10, 40,90, 2000, {.forwards = false, .maxSpeed = 80, .minSpeed = 20},true);
    chassis.waitUntil(35);
    goalClamp.set_value(1);
    chassis.waitUntilDone();

    base.tankTo(-70);
}

void qualBlueLeft(){
    chassis.setPose(-58.33,13.23,217);
    pros::delay(1500);
    setMechState(3);
    pros::delay(500);
    ls.move_voltage(-3000);
    rs.move_voltage(-3000);
    pros::delay(250);
    ls.move_voltage(0);
    ls.move_voltage(0);
    chassis.moveToPose(-24,22,255,1500,{.forwards=false, .maxSpeed = 80},true);
    chassis.waitUntil(20);
    setMechState(0);
    chassis.waitUntil(36);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);

    base.tankTo(217);
    chassis.moveToPose(-24,50,0,2000,{.minSpeed = 70,.earlyExitRange=3}, false);
    chassis.moveToPose(-75,68,270,1500,{.lead = 0.2, .minSpeed = 100},false);

    chassis.moveToPose(-48,24,60,2000,{.forwards=false, .minSpeed = 45}); 
    base.tankTo(37);

    intakeLift.set_value(1);
    chassis.moveToPose(-47,0,180,1200,{.earlyExitRange = 3});
    chassis.moveToPose(-47,15,180,1200,{.forwards = false, .earlyExitRange = 3});
    intakeLift.set_value(0);


    chassis.moveToPose(-20,5,110,1000,{},false);
    pros::delay(10);
    setMechState(5);


}


void blueLeft(){

    chassis.setPose(48,-60,290);
    setIntakeState(3);
    chassis.moveToPose(17,-49,290,2000,{.minSpeed=200},true);
    chassis.waitUntil(40);
    doinker.set_value(1);
    chassis.waitUntilDone();


    chassis.moveToPose(31,-55,290,2000,{.forwards=false, .minSpeed = 60},false);

    doinker.set_value(0);


    pros::delay(400);

    setIntakeState(0);



    base.tankTo(130);

    chassis.moveToPose(19,-16,160,2500, {.forwards = false, .maxSpeed = 70, .minSpeed = 20}, true);
    chassis.waitUntil(40);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);

    pros::delay(1000);

    chassis.moveToPose(60,-63,105, 2000, {.minSpeed = 80});
    chassis.moveToPose(75,-65,90, 1200, {.minSpeed = 127});
    chassis.moveToPose(60,-60,135, 2000, {.forwards = false, .minSpeed = 80});
    base.tankTo(-25);

    pros::delay(600);

    goalClamp.set_value(0);

    

    chassis.moveToPose(25,-50,270,3000, {.minSpeed = 60},true);

    base.tankTo(200);

    chassis.moveToPose(7,-50,90,3000, {.forwards = false, .maxSpeed = 80, .minSpeed = 20},true);
    chassis.waitUntil(20);
    goalClamp.set_value(1);
    chassis.waitUntilDone();

    chassis.moveToPose(22,-22,45, 2000, {.minSpeed = 50});
    base.tankTo(-25);
    setMechState(5);
    chassis.moveToPose(20,-20,315,2000, {.maxSpeed = 70});





    /*

    chassis.setPose(49,-36,270);
    chassis.moveToPose(15,-38.5,260,2000,{.lead=0.23,.minSpeed=200},true);
    chassis.waitUntil(90);
    doinker.set_value(1);
    chassis.waitUntilDone();


    //pros::delay(0);
    //setIntakeState(0);
    //base.tankTo(305,false);
    //base.tankTo(340,true);
    chassis.moveToPose(24,-32,280,2000,{.forwards = false,.lead=0.3, .minSpeed = 80},true);
    chassis.waitUntil(40);
    chassis.waitUntilDone();

    doinker.set_value(0);
    pros::delay(250);

    base.tankTo(-170);

    chassis.moveToPose(9,-43,85,2000,{.forwards = false, .lead = 0.2, .maxSpeed = 50},true);
    chassis.waitUntil(13);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);
    pros::delay(200);


    chassis.moveToPose(46,-24,0,2000,{.lead = 0},true);
    chassis.waitUntil(3);
    goalClamp.set_value(0);
    chassis.waitUntilDone();
    base.tankTo(180);

    chassis.moveToPose(10,-22,90,2000,{.forwards = false, .maxSpeed = 50}, true);
    chassis.waitUntil(30);
    goalClamp.set_value(1);
    chassis.waitUntilDone();

    base.tankTo(60);

    chassis.moveToPose(0,-65,210,1500,{},false);

    chassis.moveToPose(13,-24,210,1000,{.forwards = false});

    base.tankTo(-45);
    setMechState(5);
    */
}

void blueLeftElim(){
    chassis.setPose(48,-60,290);
    setIntakeState(3);
    chassis.moveToPose(17,-49,290,2000,{.minSpeed=200},true);
    chassis.waitUntil(40);
    doinker.set_value(1);
    chassis.waitUntilDone();


    chassis.moveToPose(31,-55,290,2000,{.forwards=false, .minSpeed = 60},false);

    doinker.set_value(0);


    pros::delay(400);

    setIntakeState(0);



    base.tankTo(130);

    chassis.moveToPose(19,-16,160,2500, {.forwards = false, .maxSpeed = 70, .minSpeed = 20}, true);
    chassis.waitUntil(40);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);

    pros::delay(1000);

    chassis.moveToPose(60,-63,105, 2000, {.minSpeed = 80});
    chassis.moveToPose(75,-65,90, 1200, {.minSpeed = 127});
    chassis.moveToPose(60,-60,135, 2000, {.forwards = false, .minSpeed = 80});
    base.tankTo(-25);

    pros::delay(600);

    goalClamp.set_value(0);

    

    chassis.moveToPose(25,-50,270,3000, {.minSpeed = 60},true);

    base.tankTo(200);

    chassis.moveToPose(7,-50,90,3000, {.forwards = false, .maxSpeed = 80, .minSpeed = 20},true);
    chassis.waitUntil(20);
    goalClamp.set_value(1);
    chassis.waitUntilDone();

    chassis.moveToPose(13,-50,90,3000, {.minSpeed = 50},true);
    base.tankTo(20);


}



void blueRight(){
    chassis.setPose(-58.33,-13.23,323);
    
    setMechState(3);
    pros::delay(500);
    ls.move_voltage(-6000);
    rs.move_voltage(-6000);
    pros::delay(250);
    ls.move_voltage(0);
    ls.move_voltage(0);
    chassis.moveToPose(-19,-24,280,1500,{.forwards=false, .maxSpeed = 80},true);
    chassis.waitUntil(22);
    setMechState(0);
    chassis.waitUntil(41);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);

    base.tankTo(-172);
    chassis.moveToPose(-6,-56,180,2000,{.maxSpeed = 60, .earlyExitRange = 3},false);
    chassis.moveToPose(-24,-24,135,1500,{.forwards=false, .lead = 0.6, .earlyExitRange = 3},true);
    base.tankTo(143);
    //chassis.moveToPose(-24,50,0,1500,{});
    chassis.moveToPose(-24,-50,180,2000,{.minSpeed = 50,.earlyExitRange=3}, false);
    chassis.moveToPose(-24,-20,180,2000,{.forwards = false, .minSpeed = 50, .earlyExitRange=3}, false);
    //base.tankTo(-53);
    chassis.moveToPose(-68,-72,225,2000,{.lead = 0.6, .minSpeed = 100},false);
    pros::delay(200);

    setMechState(0);
    chassis.moveToPose(-48,-24,180,2000,{.forwards=false, .minSpeed = 70 });
    base.tankTo(-37,true);
    intakeLift.set_value(1);
    chassis.moveToPose(-48,-3,0,1500,{.minSpeed = 30, .earlyExitRange = 2});
    chassis.moveToPose(-48,-15,0,1500,{.forwards = false, .minSpeed = 30, .earlyExitRange = 2});


    chassis.moveToPose(-20,-5,70,1000,{}, false);
    intakeLift.set_value(0);

    setMechState(5);

}


void blueRightElim(){
    chassis.setPose(-58.33,-13.23,323);
    
    setMechState(3);
    pros::delay(500);
    ls.move_voltage(-6000);
    rs.move_voltage(-6000);
    pros::delay(250);
    ls.move_voltage(0);
    ls.move_voltage(0);
    chassis.moveToPose(-19,-24,280,1500,{.forwards=false, .maxSpeed = 80},true);
    chassis.waitUntil(22);
    setMechState(0);
    chassis.waitUntil(41);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);

    base.tankTo(-172);
    chassis.moveToPose(-5,-56,180,2000,{.maxSpeed = 60, .earlyExitRange = 3},false);
    chassis.moveToPose(-24,-24,135,1500,{.forwards=false, .lead = 0.6, .earlyExitRange = 3},true);
    base.tankTo(143);
    //chassis.moveToPose(-24,50,0,1500,{});
    chassis.moveToPose(-24,-50,180,2000,{.minSpeed = 50,.earlyExitRange=3}, false);
    chassis.moveToPose(-24,-20,180,2000,{.forwards = false, .minSpeed = 50, .earlyExitRange=3}, false);
    //base.tankTo(-53);
    chassis.moveToPose(-68,-72,225,2000,{.lead = 0.6, .minSpeed = 100},false);
    pros::delay(200);

    setMechState(0);
    chassis.moveToPose(-48,-24,180,2000,{.forwards=false, .minSpeed = 70 });
    base.tankTo(-37,true);
    intakeLift.set_value(1);
    chassis.moveToPose(-48,0,0,1500,{.maxSpeed = 50, .minSpeed = 30, .earlyExitRange = 2},false);
    intakeLift.set_value(0);
    chassis.moveToPose(-48,-15,0,1500,{.forwards = false, .minSpeed = 30, .earlyExitRange = 2},false);

    base.tankTo(-7,true);
    chassis.moveToPose(-58,60,340,2000,{.minSpeed = 80},true);
    chassis.waitUntil(6);
    setIntakeState(5);
    chassis.waitUntilDone();



}





void AWPRed(){
    chassis.setPose(-58.33,13.23,217);
    
    setMechState(3);
    pros::delay(500);
    ls.move_voltage(-4000);
    rs.move_voltage(-4000);
    pros::delay(250);
    ls.move_voltage(0);
    ls.move_voltage(0);
    chassis.moveToPose(-20,24,255,1500,{.forwards=false, .maxSpeed = 80},true);
    chassis.waitUntil(20);
    setMechState(0);
    chassis.waitUntil(36);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);

    base.tankTo(-188);
    chassis.moveToPose(-5,56,0,2000,{.maxSpeed = 60, .minSpeed=10},false);
    chassis.moveToPose(-24,24,45,1500,{.forwards=false, .lead = 0.6, .minSpeed = 40},true);
    base.tankTo(-143);
    //chassis.moveToPose(-24,50,0,1500,{});
    chassis.moveToPose(-24,50,0,2000,{.minSpeed = 50,.earlyExitRange=3}, false);
    chassis.moveToPose(-48,24,60,2000,{.minSpeed = 50,.earlyExitRange = 5});
    base.tankTo(37, true);
    goalClamp.set_value(0);

    setIntakeState(7);

    chassis.moveToPose(-48,-24,180,5000,{.maxSpeed = 60, .minSpeed = 30, .earlyExitRange = 3},false);


    base.tankTo(-53,true);
    chassis.moveToPose(-22,-24,270,2000,{.forwards = false}, true);
    chassis.waitUntil(22);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);
    base.tankTo(37,true);
    chassis.moveToPose(-24,-48,180,1500,{.minSpeed = 50, .earlyExitRange = 3});
    chassis.moveToPose(-27,-23,135, 1200, {.forwards = false, .minSpeed = 50, .earlyExitRange = 3});
    base.tankTo(180);
    setMechState(5);






}

void AWPBlue(){
    chassis.setPose(-58.33,-13.23,323);
    
    setMechState(3);
    pros::delay(500);
    ls.move_voltage(-4000);
    rs.move_voltage(-4000);
    pros::delay(250);
    ls.move_voltage(0);
    ls.move_voltage(0);
    chassis.moveToPose(-20,-23,285,1500,{.forwards=false, .maxSpeed = 80},true);
    chassis.waitUntil(20);
    setMechState(0);
    chassis.waitUntil(36);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);

    base.tankTo(-172);
    chassis.moveToPose(-5,-56,180,2000,{.maxSpeed = 60, .earlyExitRange = 3 },false);
    chassis.moveToPose(-24,-24,135,1500,{.forwards=false, .lead = 0.6, .minSpeed = 40},true);
    base.tankTo(143);
    //chassis.moveToPose(-24,50,0,1500,{});
    chassis.moveToPose(-24,-50,180,2000,{.minSpeed = 50,.earlyExitRange=3}, false);
    chassis.moveToPose(-44,-24,120,2000,{.minSpeed = 50,.earlyExitRange = 5});
    base.tankTo(-37);
    goalClamp.set_value(0);

    setIntakeState(8);

    chassis.moveToPose(-48,24,0,5000,{.maxSpeed = 60, .minSpeed = 30, .earlyExitRange = 3},false);

    

    base.tankTo(53,true);
    chassis.moveToPose(-20,27,270,2000,{.forwards=false, .maxSpeed = 60, .minSpeed = 10},true);
    chassis.waitUntil(22);
    goalClamp.set_value(1);
    chassis.waitUntilDone();

    

    base.tankTo(-37,true);

    setIntakeState(1);
    chassis.moveToPose(-20,50,0,1500,{.minSpeed = 60, .earlyExitRange = 3});
    chassis.moveToPose(-20,25,0,1500,{.forwards=false,.minSpeed=40});
    setMechState(7);
    base.tankTo(180,true);
    setMechState(5);


}












