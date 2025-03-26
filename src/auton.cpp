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

    chassis.setPose(-63 + ( (float) distX.get_distance() / (float) 25.4), chassis.getPose().y, chassis.getPose().theta);

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
    setMechState(3);
    base.tankTo(-45);

   //DrivePID(-2000,15000);
    

    chassis.moveToPose(0,-5,135,2000,{.forwards =false, .maxSpeed = 80, .minSpeed=70});

    //chassis.moveToPose(-10,10,45,4000, {.forwards =false, .maxSpeed = 100, .minSpeed = 100});


}


ASSET(redLeftGrabGoal_txt);
void redLeft(){
    chassis.setPose(-60.39,13.84,216);
    setMechState(3);
    pros::delay(400);
    /*
    ls.move_voltage(-6000);
    rs.move_voltage(-6000);
    pros::delay(250);
    ls.move_voltage(0);
    ls.move_voltage(0);
    */
    chassis.follow(redLeftGrabGoal_txt,8,3000,false,true);
    //chassis.moveToPose(-24,24,270,2500,{.forwards=false, .horizontalDrift = 1.5, .lead = 0.7, .maxSpeed = 80, .minSpeed = 50},true);
    chassis.waitUntil(20);
    setMechState(0);
    chassis.waitUntil(40);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);

    base.tankTo(-188, true);
    chassis.moveToPose(-7,55,0,2000,{.maxSpeed = 60},false);

    chassis.moveToPose(-24,33,90,1500,{.forwards=false, .lead = 0.4, .minSpeed = 40},false);
    //setIntakeState(0);
    base.tankTo(-143,true);
    setIntakeState(1);
    //chassis.moveToPose(-24,50,0,1500,{});
    chassis.moveToPose(-24,50,0,2000,{.minSpeed = 70,.earlyExitRange=3}, false);
    chassis.moveToPose(-24,24,0,2000,{.forwards = false, .minSpeed = 50}, false);

    base.tankTo(261);

    chassis.moveToPose(-59,59,315,1500,{.earlyExitRange = 3},false);
    chassis.moveToPose(-72,72,315,900,{.lead = 0.2, .maxSpeed = 40, .minSpeed = 40},false);

    chassis.moveToPose(-59,59,315,700,{.forwards = false, .lead = 0.2, .minSpeed = 70},false);



    intakeLift.set_value(1);

    chassis.moveToPose(-72,72,315,700,{.lead = 0.2, .maxSpeed = 60, .minSpeed = 60},false);
    intakeLift.set_value(0);




    setMechState(0);
    chassis.moveToPose(-48,24,0,2000,{.forwards=false, .minSpeed = 40},false); 
    intakeLift.set_value(1);
    base.tankTo(37,true);

    
    chassis.moveToPose(-48,-2,180,1200,{.minSpeed = 50},false);
    intakeLift.set_value(0);
    chassis.moveToPose(-48,15,180,1200,{.forwards = false, .minSpeed = 40});


    chassis.moveToPose(-20,5,110,1000,{},false);
    pros::delay(10);
    setMechState(5);
    
}

void redLeftElim(){
    chassis.setPose(-58.33,13.23,217);
    setMechState(3);
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






    /*
    chassis.moveToPose(-10,-10,75,1500,{.lead = 0.3},true);
    chassis.waitUntil(24);
    setIntakeState(0);
    chassis.waitUntilDone();
    doinkerLeftLeft.set_value(1);
    pros::delay(300);

    chassis.moveToPose(-24,-28,30,1500,{.forwards = false,.earlyExitRange = 1},false);
    doinkerLeftLeft.set_value(0);
    setIntakeState(1);
    chassis.moveToPose(-24,-12,0,1500,{.minSpeed = 30, .earlyExitRange = 1});
    */


}

void qualRedRightTanner(){
    chassis.setPose(-58.33,-13.23,323);
    
    setMechState(3);
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

    chassis.moveToPose(-5, -63, 90, 2000, {.maxSpeed = 80},false);

    doinkerLeft.set_value(1);
    pros::delay(200);

    chassis.moveToPose(-48, -63, 90, 200, {.forwards = false, .minSpeed = 90}, false);

    doinkerLeft.set_value(0);

    pros::delay(200);
    base.tankTo(53);






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
void sixRingRedRight(){
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

    chassis.moveToPose(7,5,252,1000,{},false);
    
    
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

    chassis.moveToPose(48,24,315,2500,{.forwards = false, .minSpeed = 30});

    base.tankTo(80, true);

    
    chassis.moveToPose(42.5,58,0,2000, {.lead = 0.5, .minSpeed=80},true);
    chassis.waitUntil(15);
    setIntakeState(2);
    chassis.moveToPoint(42.5,51,1000, {.forwards = false});


    setIntakeState(1);
    base.tankTo(40);

    setIntakeState(1);
    chassis.moveToPose(67,81,45,1000, {.lead = 0.37, .maxSpeed = 90, .minSpeed = 90});

    chassis.moveToPose(35,47,45,1000, {.forwards = false, .lead = 0.37, .minSpeed = 50});
    chassis.moveToPose(57,69,45,1000, {.forwards = 1});
}


void sixRingBlueLeft(){
    chassis.setPose(-60,24,270);
    
    chassis.moveToPose(-22, 24, 270, 2000, {.forwards = false, .maxSpeed = 70, .minSpeed = 30},true);
    chassis.waitUntil(37);
    goalClamp.set_value(1);
    chassis.waitUntilDone();

    setIntakeState(0);


    
    base.tankTo(360-235);

    setIntakeState(1);

    chassis.moveToPose(-13,5.5,133,1500,{.lead = 0.2},true);

    chassis.waitUntil(4);
    setIntakeState(0);
    chassis.waitUntilDone();
    
    doinkerRight.set_value(1);
    pros::delay(50);

    chassis.moveToPose(-7,5,108,1000,{},false);
    
    
    doinkerLeft.set_value(1);

    pros::delay(50);

    chassis.moveToPose(-45,32,135,1500,{.forwards = false},false);



    base.tankTo(180);

    doinkerLeft.set_value(0);
    doinkerRight.set_value(0);
    pros::delay(200);
    setIntakeState(1);


    base.tankTo(100);
    chassis.swingToHeading(0,DriveSide::LEFT,1200,{.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 90, .minSpeed = 70});

    chassis.moveToPose(-22,50,0,2500,{.maxSpeed = 80, .minSpeed = 30});

    chassis.moveToPose(-48,24,45,2500,{.forwards = false, .minSpeed = 30});

    base.tankTo(-80, true);

    
    chassis.moveToPose(-42.5,58,0,2000, {.lead = 0.5, .minSpeed=80},true);
    chassis.waitUntil(15);
    setIntakeState(2);
    chassis.moveToPoint(-42.5,51,1000, {.forwards = false});


    setIntakeState(1);
    base.tankTo(-40);

    setIntakeState(1);
    chassis.moveToPose(-67,81,315,1000, {.lead = 0.37, .maxSpeed = 90, .minSpeed = 90});

    chassis.moveToPose(-35,47,315,1000, {.forwards = false, .lead = 0.37, .minSpeed = 50});
    chassis.moveToPose(-57,69,315,1000, {.forwards = 1});
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

void qualBlueLeft(){
    chassis.setPose(-58.33,13.23,217);
    pros::delay(1500);
    setMechState(3);
    pros::delay(500);
    ls.move_voltage(-6000);
    rs.move_voltage(-6000);
    pros::delay(250);
    ls.move_voltage(0);
    ls.move_voltage(0);
    chassis.moveToPose(-24,24,255,1500,{.forwards=false, .maxSpeed = 80},true);
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
    chassis.moveToPose(-45.2,0,180,1200,{.earlyExitRange = 3});
    chassis.moveToPose(-45.2,15,180,1200,{.forwards = false, .earlyExitRange = 3});
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
    setMechState(5);
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
    chassis.moveToPose(-6,-58,180,2000,{.maxSpeed = 60},false);
    chassis.moveToPose(-24,-24,135,1500,{.forwards=false, .lead = 0.6, .earlyExitRange = 3},true);
    base.tankTo(143);
    //chassis.moveToPose(-24,50,0,1500,{});
    chassis.moveToPose(-24,-50,180,2000,{.minSpeed = 50,.earlyExitRange=3}, false);
    chassis.moveToPose(-24,-20,180,2000,{.forwards = false, .minSpeed = 50, .earlyExitRange=3}, false);
    //base.tankTo(-53);
    chassis.moveToPose(-68,-75,225,2000,{.lead = 0.6, .minSpeed = 100},false);
    pros::delay(200);

    setMechState(0);
    chassis.moveToPose(-48,-24,180,2000,{.forwards=false, .minSpeed = 70 }, false);
    base.tankTo(-37,true);
    intakeLift.set_value(1);
    chassis.moveToPose(-48,0,0,1500,{.minSpeed = 30});
    chassis.moveToPose(-48,-15,0,1500,{.forwards = false, .minSpeed = 30, .earlyExitRange = 1});
    intakeLift.set_value(0);


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
    chassis.moveToPose(-8,-60,180,2000,{.maxSpeed = 60},false);
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

    
    chassis.moveToPose(-20,50,0,1500,{.minSpeed = 60, .earlyExitRange = 3}, true);
    chassis.waitUntil(1);
    setIntakeState(1);
    chassis.waitUntilDone();
    chassis.moveToPose(-20,25,0,1500,{.forwards=false,.minSpeed=40});
    setMechState(7);
    base.tankTo(180,true);
    setMechState(5);


}


ASSET(circle_txt);
void testingPID(){
    chassis.turnToHeading(90,1000,{},0);
}







