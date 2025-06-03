#include "main.h"
#include <vector>

PIDTWO::PIDTWO(double kP, double kI, double kD, double skP, double skI, double skD, double windupRange, bool signFlipReset) {
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->skP = skP;
    this->skI = skI;
    this->skD = skD;
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

PIDTWO drivePID(0.0026, 0.000000, 0.0000, 0, 0, 0, 30.0, true);

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

    setSortState(1);

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

    chassis.moveToPose(-52,62.5,290,3000,{.horizontalDrift = 8, .lead = 0.65, .maxSpeed = 80, .minSpeed = 10},false);

    


    chassis.moveToPose(-4,40,290,2000,{.forwards=false, .minSpeed = 30}, true);
    chassis.waitUntil(7);
    setMechState(1);
    chassis.waitUntilDone();

    chassis.waitUntilDone();

    


    base.tankTo(-90);


    chassis.moveToPose(-4,66,0,1500,{.lead = 0, .maxSpeed = 80, .minSpeed = 40},false);

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
    setIntakeState(6);
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
    chassis.moveToPose(48, -23, 0, 1200, {.forwards = false, .maxSpeed = 50}, true);
    chassis.waitUntil(22);
    goalClamp.set_value(1);
    chassis.waitUntilDone();


    

    base.tankTo(0);
    setIntakeState(1);
    chassis.moveToPose(26,-22,270,1500,{.minSpeed = 1});
    base.tankTo(47);


    chassis.moveToPose(-49,-56,250,3000,{.horizontalDrift = 8, .lead = 0.5, .maxSpeed = 80},false);

    
    chassis.moveToPose(-7,-40,250,2000,{.forwards=false, .minSpeed = 30}, true);
    chassis.waitUntil(1);
    setMechState(1);
    chassis.waitUntilDone();



    base.tankTo(90);
    chassis.moveToPose(-5.7,-64,180,1500,{.lead = 0, .maxSpeed = 80, .minSpeed = 20},false);
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
    setIntakeState(6);
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

    chassis.moveToPose(-57,-25,236,2000,{.lead = 0.35, .maxSpeed = 80, .minSpeed = 55}, true);

    chassis.moveToPose(-72,-62,180,2500,{.lead = 0.3, .minSpeed = 50},false);

    chassis.setPose(-63 + ( (float) distLeft.get_distance() / (float) 25.4), chassis.getPose().y, chassis.getPose().theta);

    chassis.moveToPose(-42,3,200,2000,{.forwards = false, .maxSpeed = 60, .minSpeed = 20}, true);
    chassis.waitUntil(54);
    goalClamp.set_value(1);
    chassis.waitUntilDone();

    //chassis.moveToPose(-50,0,200,1500, {});

    base.tankTo(0);

    chassis.moveToPose(-70,-2,265,1500,{.minSpeed = 70}, false);    

    DrivePID(-7,1000);

    setIntakeState(4);
    setMechState(8);





    pros::delay(600);

    DrivePID(-7,500);

    setIntakeState(6);
    setMechState(0);



    base.tankTo(90);
    chassis.moveToPose(-48,-50,180,1500, {.minSpeed = 50});
    chassis.moveToPose(3, -11,45,3000,{.minSpeed = 50}, true);
    chassis.waitUntil(45);
    setIntakeState(3);
    chassis.waitUntilDone();
    
    //base.tankTo(315, true);

    chassis.moveToPose(-38,45,315,2000,{.maxSpeed = 100, .minSpeed = 70}, true);
    chassis.waitUntil(25);
    setIntakeState(1);
    chassis.waitUntilDone();


    setIntakeState(1);
    chassis.moveToPose(-55,50,270,1500,{.maxSpeed = 80, .minSpeed = 50},false);
    setIntakeState(1);
    chassis.moveToPose(-40,50,270,1000,{.forwards = false, .minSpeed = 90},true);
    chassis.waitUntil(10);
    doinkerLeft.set_value(1);
    setIntakeState(1);
    chassis.waitUntilDone();

    base.tankTo(210, true, shortest, 20);
    base.tankTo(135, true);

    doinkerLeft.set_value(0);

    goalClamp.set_value(0);

    DrivePID(-100,600);

    chassis.moveToPose(-30,30,135,2000,{.minSpeed=50});

    setIntakeState(11);
    setMechState(4);
    base.tankTo(-45);

   //DrivePID(-2000,15000);
    

    chassis.moveToPose(0,-5,135,2000,{.forwards =false, .maxSpeed = 80, .minSpeed=70});

    //chassis.moveToPose(-10,10,45,4000, {.forwards =false, .maxSpeed = 100, .minSpeed = 100});


}


ASSET(redLeftGrabGoal_txt);



void sixRingRedLeftElim(){
    chassis.setPose(-60.39,13.84,216);
    setMechState(4);
    pros::delay(400);
    
    ls.move_voltage(-6000);
    rs.move_voltage(-6000);
    pros::delay(250);
    ls.move_voltage(0);
    ls.move_voltage(0);
    setIntakeState(2);
    
    //chassis.follow(redLeftGrabGoal_txt,8,3000,false,true);
    chassis.moveToPose(-19,25,270,1500,{.forwards=false, .maxSpeed = 80},true);
    //chassis.moveToPose(-24,24,270,2500,{.forwards=false, .horizontalDrift = 1.5, .lead = 0.7, .maxSpeed = 80, .minSpeed = 50},true);
    chassis.waitUntil(20);
    setMechState(0);
    chassis.waitUntil(40);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);

    base.tankTo(-188, true);
    chassis.moveToPose(-8.5,60,0,2700,{.maxSpeed = 55, .minSpeed = 1},false);

    chassis.moveToPose(-24,33,60,1500,{.forwards=false, .lead = 0.25, .minSpeed = 40},false);
    //setIntakeState(0);
    base.tankTo(-143,true);
    setIntakeState(1);
    //chassis.moveToPose(-24,50,0,1500,{});
    chassis.moveToPose(-24,52,0,2000,{.minSpeed = 70,.earlyExitRange=3}, false);
    chassis.moveToPose(-24,22,0,2000,{.forwards = false, .minSpeed = 50}, false);

    base.tankTo(261);

    chassis.moveToPose(-62,57,315,1500,{.lead = 0.2, .minSpeed = 60},false);

    rs.move_voltage(5000);
    ls.move_voltage(5000);
    pros::delay(500);

    chassis.moveToPose(-40,40,315,2000,{.forwards = false, .lead = 0.2, .minSpeed = 30},false);

    chassis.moveToPose(-60,60,315,700,{.lead = 0.2, .maxSpeed = 100, .minSpeed = 30},false);
    chassis.moveToPose(-48,48,315,700,{.forwards = false, .maxSpeed = 100, .minSpeed = 30},false);


    intakeLift.set_value(1);
    base.tankTo(37,true);

    
    chassis.moveToPose(-48,-2,180,1200,{.minSpeed = 50},false);
    intakeLift.set_value(0);
    chassis.moveToPose(-48,15,180,1200,{.forwards = false, .minSpeed = 40});


    chassis.moveToPose(-62,-36,20,2000,{ .minSpeed = 30});


    
}

void redLeftElim(){
    chassis.setPose(-58.33,13.23,217);
    setMechState(4);
    pros::delay(500);
    /*
    ls.move_voltage(-6000);
    rs.move_voltage(-6000);
    pros::delay(250);
    ls.move_voltage(0);
    ls.move_voltage(0);
    */
    chassis.moveToPose(-20,24,270,2500,{.forwards=false, .lead = 0.7, .maxSpeed = 80},true);
    chassis.waitUntil(20);
    setMechState(0);
    chassis.waitUntil(45);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);

    base.tankTo(-188);
    chassis.moveToPose(-8,60,0,2000,{.maxSpeed = 60},false);

    chassis.moveToPose(-24,24,45,1500,{.forwards=false, .lead = 0.6, .minSpeed = 40},false);
    setIntakeState(0);
    base.tankTo(-143);
    setIntakeState(1);
    //chassis.moveToPose(-24,50,0,1500,{});
    chassis.moveToPose(-24,50,0,2000,{.minSpeed = 70,.earlyExitRange=3}, false);
    chassis.moveToPose(-75,68,270,1500,{.lead = 0.2, .minSpeed = 100},false);

    setMechState(0);
    chassis.moveToPose(-46.5,28,0,2000,{.forwards=false, .minSpeed = 45, .earlyExitRange = 3}); 
    intakeLift.set_value(1);
    base.tankTo(37,true);

    
    chassis.moveToPose(-47.8,0,180,1200,{.earlyExitRange = 3},false);
    intakeLift.set_value(0);
    chassis.moveToPose(-47.8,20,180,1200,{.forwards = false, .earlyExitRange = 3});
    

    base.tankTo(7,true);
    chassis.moveToPose(-58,-60,200,2000,{.minSpeed = 100},true);
    chassis.waitUntil(6);
    setIntakeState(0);

    base.tankTo(180);
    
    

        

}
/*
void qualRedRight(){
    chassis.setPose(-58.33,-13.23,323);
    
    setMechState(4);
    pros::delay(500);
    ls.move_voltage(-6000);
    rs.move_voltage(-6000);
    pros::delay(250);
    ls.move_voltage(0);
    ls.move_voltage(0);
    chassis.moveToPose(-19,-24,280,1500,{.forwards=false, .maxSpeed = 80},true);
    chassis.waitUntil(20);
    setMechState(0);
    chassis.waitUntil(38);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(3);

    base.tankTo(143);

    chassis.moveToPose(-22,-50,180,2000,{.minSpeed = 50}, false);
    chassis.moveToPose(-22,-20,180,2000,{.forwards = false, .minSpeed = 50, .earlyExitRange=3}, false);
    base.tankTo(53);
    setIntakeState(1);
    chassis.moveToPose(-68,-72,180,2000,{.lead = 0.4, .minSpeed = 100},true);

    chassis.moveToPose(-45,-65,180,2000,{.forwards = false, .lead = 0.4, .minSpeed = 100},false);

    pros::delay(1000);
    goalClamp.set_value(0);



    setMechState(0);


    base.tankTo(-120);
    goalClamp.set_value(0);

    chassis.moveToPose(-24, -50, 270, 2000, {.minSpeed = 30});
    base.tankTo(53);






}*/

void qualRedRightTanner(){
    chassis.setPose(60,24,90);
    
    chassis.moveToPose(22, 24, 90, 2000, {.forwards = false, .maxSpeed = 70, .minSpeed = 30},true);
    chassis.waitUntil(37);
    goalClamp.set_value(1);
    chassis.waitUntilDone();

    setIntakeState(0);


    
    base.tankTo(235);

    setIntakeState(1);

    chassis.moveToPose(13,5.5,227,1500,{.lead = 0.2},true);

    chassis.waitUntil(4);
    setIntakeState(0);
    chassis.waitUntilDone();
    
    doinkerRight.set_value(1);
    pros::delay(50);

    chassis.moveToPose(6,5,257,1300,{},false); //time is 1000
    
    
    doinkerLeft.set_value(1);

    pros::delay(50);

    chassis.moveToPose(45,32,225,1500,{.forwards = false},false);



    base.tankTo(180);

    doinkerLeft.set_value(0);
    doinkerRight.set_value(0);
    pros::delay(200);
    setIntakeState(1);


    base.tankTo(-100);
    chassis.swingToHeading(0,DriveSide::RIGHT,1200,{.direction = AngularDirection::CW_CLOCKWISE, .maxSpeed = 90, .minSpeed = 70});

    chassis.moveToPose(22,50,0,2500,{.maxSpeed = 80, .minSpeed = 30});

    chassis.moveToPose(22,27,0,1500,{.forwards = false, .minSpeed = 50});

    base.tankTo(225);

    setMechState(6);






    /*
    chassis.moveToPose(-10,-10,75,1500,{.lead = 0.3},true);
    chassis.waitUntil(24);
    setIntakeState(0);
    chassis.waitUntilDone();
    doinkerLeft.set_value(1);
    pros::delay(300);

    chassis.moveToPose(-24,-28,30,1500,{.forwards = false,.earlyExitRange = 1},false);
    doinkerLeft.set_value(0);
    setIntakeState(1);
    chassis.moveToPose(-24,-12,0,1500,{.minSpeed = 30, .earlyExitRange = 1});
    */


}

ASSET(goalSideMidR_txt);
ASSET(redRightStack_txt);


void rushPlusSixRingRedRight(){
    chassis.setPose(51.40,34.085,292);
    setIntakeState(7);
    chassis.moveToPose(19, 47.175 ,292, 2000,{.minSpeed = 127},0);
    doinkerLeft.set_value(1);
    chassis.moveToPose(36, 40.307,292, 2000,{.forwards = false, .minSpeed = 30},0);
    doinkerLeft.set_value(0);
    base.tankTo(-90);
    setIntakeState(0);
    chassis.moveToPose(22,19.05,22, 1500, {.forwards = false, .maxSpeed = 60, .minSpeed = 30}, 1);
    chassis.waitUntil(18);
    goalClamp.set_value(1);
    chassis.waitUntilDone();

    base.tankTo(22);
    setIntakeState(1);
    chassis.moveToPose(5,9,200,1500,{.lead = 0.4, .maxSpeed = 45},1);
    chassis.waitUntil(10);
    setIntakeState(0);
    chassis.waitUntilDone();
    doinkerRight.set_value(1);
    pros::delay(200);
    //chassis.swingToHeading(230,DriveSide::LEFT, 1400,{.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 60},0);
    base.tankTo(62);

    chassis.moveToPose(27,19,230,1500,{.forwards = false, .minSpeed = 110},0);

    

    base.tankTo(140, true, shortest, 20);
    base.tankTo(242);
    //chassis.turnToHeading(70,1000,{.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .earlyExitRange = 10},0);

    pros::delay(250);
    doinkerRight.set_value(0);
    pros::delay(200);
    
    base.tankTo(202);
    

    setIntakeState(1);

    chassis.moveToPose(60,24,90,2000,{.lead = 0.75, .minSpeed = 60});
    base.tankTo(-68);

    chassis.moveToPose(62,55,0,2000,{.minSpeed=80},0);


    base.tankTo(-131, true);
    setIntakeState(1);


    rs.move_voltage(12000);
    ls.move_voltage(12000);
    pros::delay(700);
    //chassis.moveToPose(72,72,45,1500,{ .minSpeed = 127});


    chassis.moveToPose(45,45,45,2000,{.forwards = false, .minSpeed = 60},1);
    chassis.waitUntil(7);
    setIntakeState(0);
    chassis.waitUntilDone();


    setIntakeState(1);
    chassis.moveToPose(67,65,45,1000,{.minSpeed = 60},0);

    chassis.moveToPose(48,46,45,2000,{.forwards = false, .minSpeed = 60});

    base.tankTo(67);
    goalClamp.set_value(0);
    setMechState(5);
    chassis.moveToPose(24,25,225,2000,{.minSpeed = 127},0);
    

    
}




void redRightElim(){
    chassis.setPose(48,36,290);
    setIntakeState(3);
    chassis.moveToPose(17,47,290,2000,{.minSpeed=200},true);
    chassis.waitUntil(40);
    doinkerLeft.set_value(1);
    chassis.waitUntilDone();


    chassis.moveToPose(48,36,290,2000,{.forwards=false, .minSpeed = 60},true);
    chassis.waitUntil(15);
    doinkerLeft.set_value(0);
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




void blueLeft(){

    chassis.setPose(48,-60,290);
    setIntakeState(3);
    chassis.moveToPose(17,-49,290,2000,{.minSpeed=200},true);
    chassis.waitUntil(40);
    doinkerLeft.set_value(1);
    chassis.waitUntilDone();


    chassis.moveToPose(31,-55,290,2000,{.forwards=false, .minSpeed = 60},false);

    doinkerLeft.set_value(0);


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
    setMechState(6);
    chassis.moveToPose(20,-20,315,2000, {.maxSpeed = 70});





    /*

    chassis.setPose(49,-36,270);
    chassis.moveToPose(15,-38.5,260,2000,{.lead=0.23,.minSpeed=200},true);
    chassis.waitUntil(90);
    doinkerLeft.set_value(1);
    chassis.waitUntilDone();


    //pros::delay(0);
    //setIntakeState(0);
    //base.tankTo(305,false);
    //base.tankTo(340,true);
    chassis.moveToPose(24,-32,280,2000,{.forwards = false,.lead=0.3, .minSpeed = 80},true);
    chassis.waitUntil(40);
    chassis.waitUntilDone();

    doinkerLeft.set_value(0);
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
    doinkerLeft.set_value(1);
    chassis.waitUntilDone();


    chassis.moveToPose(31,-55,290,2000,{.forwards=false, .minSpeed = 60},false);

    doinkerLeft.set_value(0);


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






void blueRightElim(){
    chassis.setPose(-58.33,-13.23,323);
    
    setMechState(4);
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
    
    setMechState(4);
    pros::delay(500);
    ls.move_voltage(-6000);
    rs.move_voltage(-6000);
    pros::delay(250);
    ls.move_voltage(0);
    ls.move_voltage(0);
    chassis.moveToPose(-20,25,255,1500,{.forwards=false, .maxSpeed = 80},true);
    chassis.waitUntil(20);
    setMechState(0);
    chassis.waitUntil(36);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);

    base.tankTo(-188);
    chassis.moveToPose(-5,60,0,2000,{.maxSpeed = 60, .minSpeed=10},false);
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
    chassis.moveToPose(-20,-24,270,2000,{.forwards = false, .maxSpeed = 60, .minSpeed = 20}, true);
    chassis.waitUntil(28);
    goalClamp.set_value(1);
    chassis.waitUntilDone();


    

    pros::delay(250);

    setIntakeState(1);
     
    pros::delay(450);
    
    base.tankTo(37,true);
    

    chassis.moveToPose(-24,-48,180,1500,{.minSpeed = 50, .earlyExitRange = 3}, true);
    chassis.waitUntil(0.1);
    setIntakeState(1);
    chassis.waitUntilDone();
    chassis.moveToPose(-24,-27,135, 1200, {.forwards = false, .minSpeed = 50});
    base.tankTo(180);
    setMechState(6);






}

void AWPBlue(){
    chassis.setPose(-58.33,-13.23,323);
    
    setMechState(4);
    pros::delay(500);
    ls.move_voltage(-4000);
    rs.move_voltage(-4000);
    pros::delay(250);
    ls.move_voltage(0);
    ls.move_voltage(0);
    setIntakeState(2);
    chassis.moveToPose(-18,-22,280,1500,{.forwards=false, .maxSpeed = 80},true);
    chassis.waitUntil(23);
    setMechState(0);
    chassis.waitUntil(41);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);

    base.tankTo(-172);
    chassis.moveToPose(-7,-53,180,2000,{.maxSpeed = 60},false);
    chassis.moveToPose(-24,-24,135,1500,{.forwards=false, .lead = 0.3, .earlyExitRange = 3},true);
    base.tankTo(143);
    //chassis.moveToPose(-24,50,0,1500,{});
    chassis.moveToPose(-24,-50,180,2000,{.minSpeed = 50,.earlyExitRange=3}, false);


    chassis.moveToPose(-44,-24,120,2000,{.minSpeed = 50,.earlyExitRange = 5});
    base.tankTo(-37);
    goalClamp.set_value(0);

    setIntakeState(8);

    chassis.moveToPose(-48,24,0,5000,{.maxSpeed = 60, .minSpeed = 30, .earlyExitRange = 3},false);

    

    base.tankTo(53,true);
    chassis.moveToPose(-17,27,270,2300,{.forwards=false, .maxSpeed = 60, .minSpeed = 10},true);
    chassis.waitUntil(25);
    goalClamp.set_value(1);
    chassis.waitUntilDone();

    

    base.tankTo(-37,true);

    
    chassis.moveToPose(-20,50,0,1500,{.minSpeed = 60}, true);
    chassis.waitUntil(1);
    setIntakeState(1);
    chassis.waitUntilDone();
    chassis.moveToPose(-17,25,0,1500,{.forwards=false,.minSpeed=60});
    base.tankTo(180,true);
    setMechState(6);


}


ASSET(circle_txt);
void testingPID(){
    chassis.turnToHeading(90,1000,{},0);
}






void eightRingBlueRight(){
    chassis.setPose(-58.33,-13.23,323);
    
    setMechState(4);
    pros::delay(400);
    ls.move_voltage(-3000);
    rs.move_voltage(-3000);
    pros::delay(250);
    ls.move_voltage(0);
    ls.move_voltage(0);
    setIntakeState(2);
    chassis.moveToPose(-18,-24,280,1500,{.forwards=false, .maxSpeed = 60},true);
    chassis.waitUntil(23);
    setMechState(0);
    chassis.waitUntil(40);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(0);
    base.tankTo(-90);
    chassis.moveToPose(-4.5,-9,18,2500,{.maxSpeed = 45},false);
    doinkerRight.set_value(1);
    pros::delay(200);
    base.tankTo(-98.23,true,shortest,2);


    chassis.moveToPose(-35,-21,80,2000,{.forwards = false, .minSpeed = 1},false);
    pros::delay(100);

    base.tankTo(-130);
    doinkerRight.set_value(0);
    pros::delay(50);
    base.tankTo(-150);


    setIntakeState(1);
    setEdgeState(1);
    
    chassis.moveToPose(-1,-63,180,4000,{.horizontalDrift = 2, .maxSpeed = 70},1);
    chassis.waitUntil(41);
    setIntakeState(3);
    chassis.waitUntil(43);
    setMechState(1);
    chassis.waitUntilDone();



    base.tankTo(-135-13.23-15);
    //chassis.swingToHeading(140,DriveSide::LEFT,1200,{.direction = AngularDirection::CCW_COUNTERCLOCKWISE},1);
    setEdgeState(0);
    setIntakeState(2);
    setMechState(3);





}


//==============================================    Working Autons    =================================================================

void redLeft(){
    setSortState(1);
    chassis.setPose(-60.39,13.84,216);
    setMechState(4);
    pros::delay(300);
    
    ls.move_voltage(-6000);
    rs.move_voltage(-6000);
    pros::delay(250);
    ls.move_voltage(0);
    ls.move_voltage(0);


    setIntakeState(2);
    
    //chassis.follow(redLeftGrabGoal_txt,8,3000,false,true);
    chassis.moveToPose(-19,25,270,1500,{.forwards=false, .maxSpeed = 80, .minSpeed = 30},true);
    //chassis.moveToPose(-24,24,270,2500,{.forwards=false, .horizontalDrift = 1.5, .lead = 0.7, .maxSpeed = 80, .minSpeed = 50},true);
    chassis.waitUntil(20);
    setMechState(0);
    chassis.waitUntil(40);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);

    base.tankTo(-188, true);
    chassis.moveToPose(-9.5,60,0,2700,{.maxSpeed = 55, .minSpeed = 1},false);

    chassis.moveToPose(-24,33,60,1500,{.forwards=false, .lead = 0.25, .minSpeed = 40},false);

    setSortState(0);
    //setIntakeState(0);
    base.tankTo(-143,true);
    setIntakeState(1);
    //chassis.moveToPose(-24,50,0,1500,{});
    chassis.moveToPose(-26,52,0,2000,{.minSpeed = 70,.earlyExitRange=3}, false);
    chassis.moveToPose(-36,34,30,2000,{.forwards = false, .minSpeed = 20}, false);

    base.tankTo(261);

    setSortState(1);

    chassis.moveToPose(-74,71,315,1500,{.lead = 0.2, .maxSpeed = 110, .minSpeed = 110},false);
    pros::delay(150);
    

    chassis.moveToPose(-48,48,315,700,{.forwards = false, .minSpeed = 70},false);

    //intakeLift.set_value(1);

    chassis.moveToPose(-65,63,315,700,{.lead = 0.2, .minSpeed = 60},false);

    chassis.moveToPose(-48,48,315,700,{.forwards = false, .minSpeed = 0.5},false);


    intakeLift.set_value(1);
    base.tankTo(37,true);

    
    chassis.moveToPose(-48,-4,180,1200,{.minSpeed = 65},false);
    intakeLift.set_value(0);
    chassis.moveToPose(-48,15,180,1200,{.forwards = false, .minSpeed = 40});
    
    chassis.moveToPose(-26,5,110,1000,{}, 1);
    chassis.waitUntil(4);
    setMechState(10);
    chassis.waitUntilDone();
    
}


void blueRight(){

    setSortState(2);
    chassis.setPose(-58.33,-13.23,323);
    
    setMechState(4);
    pros::delay(300);
    ls.move_voltage(-4000);
    rs.move_voltage(-4000);
    pros::delay(250);
    ls.move_voltage(0);
    ls.move_voltage(0);
    setIntakeState(2);
    chassis.moveToPose(-18,-26,280,1500,{.forwards=false, .maxSpeed = 80, .minSpeed = 30},true);
    chassis.waitUntil(23);
    setMechState(0);
    chassis.waitUntil(41);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    

    base.tankTo(-172);
    setIntakeState(1);
    chassis.moveToPose(-7,-60,180,2000,{.maxSpeed = 55, .minSpeed = 1},false);
    chassis.moveToPose(-24,-24,135,1500,{.forwards=false, .lead = 0.3, .minSpeed = 30},true);
    base.tankTo(143);
    //chassis.moveToPose(-24,50,0,1500,{});
    chassis.moveToPose(-24,-50,180,2000,{.minSpeed = 70}, false);


    chassis.moveToPose(-36,-36,160,2000,{.forwards = false, .minSpeed = 1}, false);
    base.tankTo(98);

    chassis.moveToPose(-78,-75,225,1500,{.maxSpeed = 100, .minSpeed = 90},false);


    chassis.moveToPose(-43,-43,225,900,{.forwards = false, .minSpeed = 127},false);
    setIntakeState(1);
    chassis.moveToPose(-73,-67,225,1000,{.minSpeed = 110},false);


    setMechState(0);
    //chassis.moveToPose(-51,-44,225,1000,{.forwards=false, .minSpeed = 1}, false);
    //base.tankTo(-37,true);

    chassis.moveToPose(-48,-46,225,1500,{.forwards = false, .minSpeed = 1},0);
    setIntakeState(1);
    base.tankTo(-37,true);

    intakeLift.set_value(1);
    
    chassis.moveToPose(-47,6,0,1800,{.minSpeed = 60}, 0);

    


    chassis.moveToPose(-48,-15,0,1500,{.forwards = false, .minSpeed = 100});
    intakeLift.set_value(0);


    chassis.moveToPose(-20,-5,70,1000,{}, false);
    chassis.waitUntil(4);
    setMechState(10);
    chassis.waitUntilDone();

}

void qualRedRight(){
    chassis.setPose(-58.33,-13.23,323);
    
    setMechState(4);
    pros::delay(400);
    ls.move_voltage(-3000);
    rs.move_voltage(-3000);
    pros::delay(250);
    ls.move_voltage(0);
    ls.move_voltage(0);
    setIntakeState(2);
    chassis.moveToPose(-22,-24,280,1500,{.forwards=false, .maxSpeed = 60},true);
    chassis.waitUntil(23);
    setMechState(0);
    chassis.waitUntil(40);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(0);

    base.tankTo(-90);
    chassis.moveToPose(-4,-10.5,17.5,2500,{.maxSpeed = 45},false);
    doinkerRight.set_value(1);
    pros::delay(200);

    base.tankTo(-87);
    chassis.moveToPose(-31,-20,30,1500,{.forwards = false, .lead = 0.2},0);
    pros::delay(500);
    base.tankTo(-165);
    pros::delay(200);
    doinkerRight.set_value(0);
    pros::delay(200);
    base.tankTo(-172);
    setIntakeState(1);
    chassis.moveToPose(-24,-50,180,2000,{.maxSpeed = 60});
    chassis.moveToPose(-24,-24,180,2000,{.forwards = false, .minSpeed = 30});
    base.tankTo(-90);
    setMechState(6);
}

void qualBlueLeft(){
    setSortState(2);
    chassis.setPose(-58.33,13.23,217);
    setMechState(4);
    pros::delay(400);
    ls.move_voltage(-3000);
    rs.move_voltage(-3000);
    pros::delay(250);
    ls.move_voltage(0);
    ls.move_voltage(0);
    chassis.moveToPose(-22,24,260,1500,{.forwards=false, .maxSpeed = 80},true);
    chassis.waitUntil(20);
    setMechState(0);
    chassis.waitUntil(40);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(0);

    base.tankTo(90);
    chassis.moveToPose(-3,6,160,2000,{.lead = 0.4, .maxSpeed = 55},false);
    doinkerLeft.set_value(1);
    pros::delay(200);

    base.tankTo(87);
    chassis.moveToPose(-31,9,90,2000,{.forwards = false, .lead = 0, .maxSpeed = 60},0);
    pros::delay(500);
    base.tankTo(180);
    pros::delay(200);
    doinkerLeft.set_value(0);
    pros::delay(200);
    base.tankTo(172);
    setIntakeState(1);
    chassis.moveToPose(-24,53,0,2000,{.maxSpeed = 70});
    chassis.moveToPose(-24,20,340,2000,{.forwards = false, .minSpeed = 30});
    base.tankTo(90);
    setMechState(6);
}

void sixRingRedRight(){
    setSortState(1);
    chassis.setPose(-52.5,-24,270);

    chassis.moveToPose(-20,-24,270,1500,{.forwards=false, .maxSpeed = 80, .minSpeed = 30},true);
    chassis.waitUntil(27);
    goalClamp.set_value(1);
    chassis.waitUntilDone();

    pros::delay(100);
    setIntakeState(0);
    base.tankTo(-170);


    chassis.moveToPose(-6,-8,17,1800,{.lead = 0.2, .maxSpeed = 55},true);
    chassis.waitUntil(1);
    setIntakeState(1);
    chassis.waitUntil(9);
    setIntakeState(4);
    chassis.waitUntil(15);
    setIntakeState(0);
    chassis.waitUntilDone();


    doinkerRight.set_value(1);
    pros::delay(200);

    base.tankTo(-160);
    setIntakeState(5);
    
    chassis.moveToPose(-40,-21,70,1500,{.forwards = false, .minSpeed = 60},0);
    base.setTankConstants(0.72, 0.008, 7.2, 7, 8, 3, 3000, 0); // TUNED W/ STL @ 200 
    base.tankTo(-205);
    pros::delay(200);
    doinkerRight.set_value(0);
    pros::delay(200);
    base.setTankConstants(0.72, 0.008, 7.2, 7, 12, 3, 3000, 0); // TUNED W/ STL @ 200 
    base.tankTo(-240);
    setIntakeState(1);
    chassis.moveToPose(-24,-52,180,2000,{.lead = 0.35, .maxSpeed = 50, .minSpeed = 35});

    chassis.moveToPose(-24,-24,180,1500,{.forwards = false, .minSpeed = 60},0);

    chassis.moveToPose(-48,-52,225,2500,{.maxSpeed = 90, .minSpeed = 70},false);

    pros::delay(300);

    chassis.moveToPose(-66,-74,225,1200,{.maxSpeed = 110, .minSpeed = 110},false);


    chassis.moveToPose(-44,-44,225,700,{.forwards = false, .minSpeed = 60},1);
    chassis.moveToPose(-60,-64,225,1200,{.minSpeed = 60},false);

    setMechState(10);


    chassis.moveToPose(-24,-24,225,1200,{.forwards = false, .minSpeed = 60},false);
    base.tankTo(-135,true, shortest);

    chassis.moveToPose(-22,-22,45,1500,{},1);
    chassis.waitUntil(1);
    setMechState(6);
    chassis.waitUntilDone();
}


void sixRingBlueLeft(){
    setSortState(2);
    chassis.setPose(-52.5,24,270);
    
    chassis.moveToPose(-22, 24, 270,1500,{.forwards=false, .maxSpeed = 80, .minSpeed = 30},true);
    chassis.waitUntil(29);
    goalClamp.set_value(1);
    chassis.waitUntilDone();


    base.tankTo(170);
    

    chassis.moveToPose(-6.5,8,162,2000,{.lead = 0.2, .maxSpeed = 45},true);
    chassis.waitUntil(1);
    setIntakeState(1);
    chassis.waitUntil(9);
    setIntakeState(4);
    chassis.waitUntil(15);
    setIntakeState(0);
    chassis.waitUntilDone();


    doinkerLeft.set_value(1);
    pros::delay(200);

    base.tankTo(160);
    setIntakeState(5);
    
    chassis.moveToPose(-40,21,110,1500,{.forwards = false, .minSpeed = 60},0);
    base.setTankConstants(0.72, 0.008, 7.2, 7, 8, 3, 3000, 0); // TUNED W/ STL @ 200 
    base.tankTo(221);
    pros::delay(200);
    doinkerLeft.set_value(0);
    pros::delay(350);
    base.setTankConstants(0.72, 0.008, 7.2, 7, 12, 3, 3000, 0); // TUNED W/ STL @ 200 
    base.tankTo(240);
    setIntakeState(1);

    chassis.moveToPose(-24,57,0,2000,{.lead = 0.35, .maxSpeed = 60, .minSpeed = 45});

    chassis.moveToPose(-24,24,0,1500,{.forwards = false, .minSpeed = 60},0);

    chassis.moveToPose(-48,56,315,2500,{.maxSpeed = 90, .minSpeed = 70},false);

    pros::delay(300);

    chassis.moveToPose(-66,74,315,1200,{.maxSpeed = 90, .minSpeed = 90},false);


    chassis.moveToPose(-43,44.4,315,700,{.forwards = false, .minSpeed = 127},0);


    pros::delay(500);

    chassis.moveToPose(-72,85,315,550,{.maxSpeed = 90, .minSpeed = 90},false);



    chassis.moveToPose(-24,24,315,1200,{.forwards = false, .minSpeed = 85},false);

    base.tankTo(135,true);
    setMechState(6);


   
}

void sixRingRedRightElims(){
    setSortState(1);
    chassis.setPose(-52.5,-24,270);

    chassis.moveToPose(-20,-24,270,1500,{.forwards=false, .maxSpeed = 80, .minSpeed = 50},true);
    chassis.waitUntil(27);
    goalClamp.set_value(1);
    chassis.waitUntilDone();

    pros::delay(100);
    setIntakeState(0);
    base.tankTo(-170);


    chassis.moveToPose(-6,-8,17,1800,{.lead = 0.2, .maxSpeed = 55},true);
    chassis.waitUntil(1);
    setIntakeState(1);
    chassis.waitUntil(9);
    setIntakeState(4);
    chassis.waitUntil(15);
    setIntakeState(0);
    chassis.waitUntilDone();


    doinkerRight.set_value(1);
    pros::delay(200);

    base.tankTo(-160);
    setIntakeState(5);
    
    chassis.moveToPose(-40,-21,70,1500,{.forwards = false, .minSpeed = 60},0);
    base.setTankConstants(0.72, 0.008, 7.2, 7, 8, 3, 3000, 0); // TUNED W/ STL @ 200 
    base.tankTo(-205);
    pros::delay(200);
    doinkerRight.set_value(0);
    pros::delay(200);
    base.setTankConstants(0.72, 0.008, 7.2, 7, 12, 3, 3000, 0); // TUNED W/ STL @ 200 
    base.tankTo(-240);
    setIntakeState(1);
    chassis.moveToPose(-24,-52,180,2000,{.lead = 0.35, .maxSpeed = 50, .minSpeed = 35});

    chassis.moveToPose(-24,-24,180,1500,{.forwards = false, .minSpeed = 60},0);

    chassis.moveToPose(-48,-52,225,2500,{.maxSpeed = 90, .minSpeed = 70},false);

    pros::delay(300);

    chassis.moveToPose(-66,-74,225,1200,{.maxSpeed = 90, .minSpeed = 90},false);


    chassis.moveToPose(-44,-46,225,700,{.forwards = false, .minSpeed = 90},1);
    chassis.moveToPose(-64,-68,225,1200,{.minSpeed = 60},false);

    chassis.moveToPose(-48,-68,290,1000,{.forwards = false, .minSpeed = 60});
    chassis.waitUntil(18);
    goalClamp.set_value(0);
    chassis.waitUntilDone();


    base.tankTo(30,true,shortest,15);

    chassis.moveToPose(-18,-48,270,1500,{.forwards = false},0);




}


void sixRingBlueLeftElims(){
    setSortState(2);
    chassis.setPose(-52.5,24,270);
    
    chassis.moveToPose(-22, 24, 270,1500,{.forwards=false, .maxSpeed = 80, .minSpeed = 50},true);
    chassis.waitUntil(29);
    goalClamp.set_value(1);
    chassis.waitUntilDone();


    base.tankTo(170);
    

    chassis.moveToPose(-6.5,8,162,1800,{.lead = 0.2, .maxSpeed = 45},true);
    chassis.waitUntil(1);
    setIntakeState(1);
    chassis.waitUntil(9);
    setIntakeState(4);
    chassis.waitUntil(15);
    setIntakeState(0);
    chassis.waitUntilDone();


    doinkerLeft.set_value(1);
    pros::delay(200);

    base.tankTo(160);
    setIntakeState(5);
    
    chassis.moveToPose(-40,21,110,1500,{.forwards = false, .minSpeed = 60},0);
    base.setTankConstants(0.72, 0.008, 7.2, 7, 8, 3, 3000, 0); // TUNED W/ STL @ 200 
    base.tankTo(221);
    pros::delay(200);
    doinkerLeft.set_value(0);
    pros::delay(350);
    base.setTankConstants(0.72, 0.008, 7.2, 7, 12, 3, 3000, 0); // TUNED W/ STL @ 200 
    base.tankTo(240);
    setIntakeState(1);

    chassis.moveToPose(-24,57,0,2000,{.lead = 0.35, .maxSpeed = 70, .minSpeed = 45});

    chassis.moveToPose(-24,24,0,1500,{.forwards = false, .minSpeed = 60},0);

    chassis.moveToPose(-48,56,315,2500,{.maxSpeed = 90, .minSpeed = 70},false);

    pros::delay(300);

    chassis.moveToPose(-66,74,315,800,{.maxSpeed = 90, .minSpeed = 90},false);


    chassis.moveToPose(-43,46,315,700,{.forwards = false, .minSpeed = 127},0);


    pros::delay(500);

    chassis.moveToPose(-72,79,315,550,{.maxSpeed = 90, .minSpeed = 90},false);

    pros::delay(600);

    base.tankTo(90,true, shortest, 15);
    goalClamp.set_value(0);
    pros::delay(50);
    base.tankTo(-10,true, shortest, 6);

    chassis.moveToPose(-12,48,270,1500,{.forwards = false},false);
   
}