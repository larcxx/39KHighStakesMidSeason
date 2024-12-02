#include "main.h"

void skills(){
    chassis.setPose(0,-61.5,0);
    setIntakeState(1);
    pros::delay(800);
    ls.move_voltage(8000);
    rs.move_voltage(8000);
    pros::delay(80);
    ls.move_voltage(0);
    rs.move_voltage(0);    
    base.tankTo(250);
    goalClamp.set_value(0);
    chassis.moveToPose(-23,-45,0,3000,{.forwards = false, .lead = 0, .maxSpeed = 80, .minSpeed = 30}, true);
    chassis.waitUntil(22);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    chassis.moveToPose(-24,-28,315,1000,{.lead = 0, .minSpeed = 70, .earlyExitRange = 7});
    //chassis.moveToPose(-50,9,320,2000,{.lead = 0.8, .minSpeed = 100, .earlyExitRange = 5}, false);
    //base.tankTo(0);
    chassis.moveToPose(-48,43,0,5000,{.horizontalDrift = 2},true);
    chassis.waitUntil(73);
    setMechState(1);
    chassis.waitUntilDone();

    chassis.moveToPose(-40, -5, 345,3000,{.forwards = false, .minSpeed = 40}, false);
    base.setTankConstants(0.72, 0.008, 7.2, 7, 12, 0.5, 1300, 300);
    base.tankTo(90);
    setMechState(3);
    setIntakeState(2);
    pros::delay(200);
    setIntakeState(1);
    chassis.moveToPose(-63,-5,270,2000,{.lead = 0,.maxSpeed = 50},true);
    base.setTankConstants(0.72, 0.008, 7.2, 7, 12, 3, 1300, 0);
    chassis.waitUntil(16);
    setIntakeState(0);
    chassis.waitUntilDone();
    setMechState(4);
    pros::delay(500);
    setMechState(1);
    pros::delay(700);
    setIntakeState(1);
    pros::delay(1500);
    setMechState(4);
    setIntakeState(2);

    pros::delay(1000);
    setMechState(0);
    setIntakeState(1);
    pros::delay(1000);


    
    ls.move_voltage(-11000);
    rs.move_voltage(-7000);
    pros::delay(200);
    ls.move_voltage(0);
    rs.move_voltage(0);

    base.tankTo(180);




    
    //chassis.moveToPose(-40,-3,270,1000,{.forwards = false, .minSpeed = 30, .earlyExitRange = 10},true);
    //base.tankTo(180);
    chassis.moveToPose(-40,-60,180,2000,{.minSpeed = 70});
    base.tankTo(18);
    chassis.moveToPose(-60,-50,0,2000,{.minSpeed = 40,.earlyExitRange = 5});
    pros::delay(150);
    chassis.moveToPose(-63,-63,5,3000,{.forwards = false, .minSpeed = 35, .earlyExitRange = 5}, true);

    
    chassis.waitUntil(6);
    goalClamp.set_value(0);
    chassis.waitUntilDone();
    base.tankTo(307);
    setIntakeState(3);
    chassis.moveToPose(0,-2,45,3000,{},false);
    base.tankTo(225);
    chassis.moveToPose(24,-28,225,1500,{.minSpeed = 50, .earlyExitRange = 5}, false);
    setIntakeState(0);
    base.tankTo(0);
    chassis.moveToPose(22,-48,0,2000,{.forwards = false, .minSpeed = 30}, true);
    chassis.waitUntil(24);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);
    base.tankTo(70);
    chassis.moveToPose(65,-65,280,2000,{.forwards = false, .minSpeed = 70}, true);
    chassis.waitUntil(30);
    goalClamp.set_value(0);
    chassis.waitUntilDone();
    base.tankTo(45);
    
    chassis.moveToPose(24,-24,315,3000,{.minSpeed = 45}, false);
    setIntakeState(0);
    base.tankTo(225);
    setMechState(4);
    pros::delay(1000);
    chassis.moveToPose(10,-10,45,3000,{.forwards = false, .minSpeed = 90}, false);

}

void blueLeft(){

    chassis.setPose(53,-60,270);
    chassis.moveToPose(20,-60,280,2000,{.lead= .15,.minSpeed = 127},false);
    doinker.set_value(1);
    base.tankTo(305,false);
    base.tankTo(340,true);        
    chassis.moveToPose(40,-55,270,2000,{.forwards = false,.lead=0.3},false);
    doinker.set_value(0);
    base.tankTo(180,false);

    /*
    pros::delay(2000);

    base.tankTo(180,false);
    chassis.moveToPose(25,-60,270,2000,{.forwards = false,.minSpeed = 90},true);
    pros::delay(2000);

    goalClamp.set_value(1);
    base.tankTo(270,true);
    setIntakeState(1);
    chassis.moveToPose(25,-50,0,2000,{.forwards = true,.minSpeed = 90},true);
    base.tankTo(135,true);
    chassis.moveToPose(60,-65,90,2000,{.forwards = true,.minSpeed = 90},true);
    */
}

void blueRight(){
    chassis.setPose(53,24,90);
    chassis.moveToPose(25,24,90,2000,{.forwards = false,.minSpeed = 50},true);
    chassis.waitUntil(25);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    base.tankTo(120, true);
    setIntakeState(1);
    chassis.moveToPose(4,55,0,2000,{.forwards = true,.lead=0.80,.maxSpeed =70,.minSpeed = 30},true);
    base.tankTo(115, true);
    chassis.moveToPose(36,43,115,2000,{.forwards = true,.lead=0.80,.maxSpeed =80,.minSpeed = 60},true);
    //add forward backward motion to intake the rings
    chassis.moveToPose(58,58,45,2000,{.forwards = true,.lead=0.80,.maxSpeed =127,.minSpeed = 65},true);
    base.tankTo(135, true);
    
    chassis.moveToPose(52,2,180,3000,{.forwards = true,.lead=0.80,.maxSpeed =127,.minSpeed = 80},true);

    //lift up the intake while going forward and move lb up
    base.tankTo(0);
    //forward 5 inches
    //scoring the ring with lb
    //back up a few inches
    base.tankTo(180);

    ls.move_voltage(11000);
    rs.move_voltage(11000);
    pros::delay(500);
    ls.move_voltage(0);
    rs.move_voltage(0);

}

void redRight(){
    chassis.setPose(-24,53.25,0);
    chassis.moveToPose(-24,24,0,2000,{.forwards = false,.minSpeed = 60},true);
    chassis.waitUntil(23);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);
    base.tankTo(90);
    chassis.moveToPose(-51,23,270,4800,{.minSpeed = 30},false);
    pros::delay(800);
    chassis.moveToPose(-24,24,270,1500,{.forwards = false, .minSpeed = 50},false);
    setIntakeState(2);
    goalClamp.set_value(0);
    base.tankTo(270);
    setIntakeState(2);
    chassis.moveToPose(-48, 24, 100,1300,{.forwards = false, .lead = 0, .minSpeed = 30}, false);
    base.tankTo(0);
    chassis.moveToPose(-45.5,2,0,2000,{.forwards = false, .maxSpeed = 50, .minSpeed = 40}, true);
    chassis.waitUntil(20);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    ls.move_voltage(12000);
    rs.move_voltage(12000);
    pros::delay(65);
    base.tankTo(-30);
    chassis.moveToPose(-24,47,30,3000,{.minSpeed = 70},false);
    base.tankTo(-90);
    setIntakeState(1);
    chassis.moveToPose(0,46,90,1000,{.minSpeed = 40},false);
    ls.move_voltage(-12000);
    rs.move_voltage(-12000);
    pros::delay(100);
    ls.move_voltage(0);
    rs.move_voltage(0);
    base.tankTo(315);
    setIntakeState(2);
    pros::delay(500);
    base.tankTo(-90);
    setIntakeState(1);
    setMechState(4);
    chassis.moveToPose(20,47,90,1200,{.minSpeed = 40},false);
    
    base.tankTo(150);
    

    ls.move_voltage(11000);
    rs.move_voltage(11000);
    
    pros::delay(200);
    pros::delay(300);
    ls.move_voltage(0);
    rs.move_voltage(0);

}


void blueLeftElim(){
    chassis.setPose(-24,-53.25,180);
    chassis.moveToPose(-24,-22,180,2000,{.forwards = false,.minSpeed = 60},true);
    chassis.waitUntil(25);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    setIntakeState(1);
    base.tankTo(270);
    chassis.moveToPose(-51,-27,270,5000,{.minSpeed = 30},false);
    pros::delay(800);
    chassis.moveToPose(-24,-24,270,1500,{.forwards = false, .minSpeed = 50},false);
    base.tankTo(-45);
    chassis.moveToPose(-58,-58,225,6000,{},false);
    setDoinkerState();
}

void redRightElim(){
    chassis.setPose(-24,53.25,0);
    chassis.moveToPose(-24,22,0,5000,{.forwards = false,.minSpeed = 60},true);
    chassis.waitUntil(25);
    goalClamp.set_value(1);
    chassis.waitUntilDone();
    pros::delay(1000);
    setIntakeState(1);
    base.tankTo(90);
    chassis.moveToPose(-51,23,270,4800,{.minSpeed = 30},false);
    pros::delay(800);
    chassis.moveToPose(-21,24,270,1500,{.forwards = false, .minSpeed = 50},false);
    base.tankTo(45);
    chassis.moveToPose(-58,58,315,6000,{}, false);
    setDoinkerState();
}